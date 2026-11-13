#!/usr/bin/env python3
"""Offline command regression model, not a hardware/timing validation.

Run with --dumper /path/to/mcu_command_dump, or --self-test for model controls.
Only Python's standard library is used. Generated commands run unchanged with
in-memory imports, firmware bytes, pins, pad registers and virtual time. No
USB, serial, camera, real firmware file or real pulse operation is available.
"""

import argparse
import ast
import binascii
import builtins
import errno
import io
import json
import subprocess
import sys
import types
import unittest
from dataclasses import dataclass


APPROVED_SHA256 = "06b46f189635824b2ded6baeccf2794e3e52a78b004f1413fb8c00aac4a3651b"
FIRMWARE_BYTES = b"offline approved rp2350_burst firmware model; not board data"
GP2_PAD = 0x4003800C
DRIVES = (2, 4, 8, 12)
ABSENT_PROGRAM = "prog not in instruction memory"
PIO_INSTRUCTIONS = {"jmp", "wait", "in_", "out", "push", "pull", "mov", "irq", "set", "nop"}

# The approved finite firmware has 15 instructions. Labels/wrap directives do
# not occupy instruction memory. Its host-side burst semantics are below.
FINITE_ASSEMBLY = """
@rp2.asm_pio(set_init=rp2.PIO.OUT_LOW)
def _finite_burst():
    pull(block)
    mov(y, osr)
    pull(block)
    mov(isr, osr)
    pull(block)
    label('pulse')
    set(pins, 1)
    mov(x, isr)
    label('high_delay')
    jmp(x_dec, 'high_delay')
    set(pins, 0)
    mov(x, osr)
    label('low_delay')
    jmp(x_dec, 'low_delay')
    jmp(y_dec, 'pulse')
    mov(isr, null)
    push(block)
    label('done')
    jmp('done')
"""

# A negative control intentionally reproduces the old reload/stop-only path.
# Fixed-path tests never substitute this for the generated application code.
LEGACY_BURST = """
import sys
sys.modules.pop('rp2350_burst', None)
import rp2350_burst as b
b.burst(1, 1000000, None)
"""


@dataclass(eq=False)
class Program:
    name: str
    words: int


class Pio:
    OUT_LOW = 0

    def __init__(self, board, index):
        self.board = board
        self.index = index
        self.loaded = {}
        self.peak_words = 0
        self.allocations = 0
        self.remove_attempts = []
        self.clear_calls = 0

    @property
    def used_words(self):
        return sum(program.words for program in self.loaded)

    def add_program(self, program):
        self.board.record("pio%d.add" % self.index, program)
        self.allocations += 1
        if program in self.loaded:
            return self.loaded[program]
        for offset in range(33 - program.words):
            if all(offset + program.words <= start or start + other.words <= offset
                   for other, start in self.loaded.items()):
                self.loaded[program] = offset
                self.peak_words = max(self.peak_words, self.used_words)
                return offset
        raise OSError(errno.ENOMEM, "ENOMEM")

    def remove_program(self, program=None):
        self.remove_attempts.append(program)
        self.board.trip("pio%d.remove" % self.index, program)
        if program is None:
            self.clear_calls += 1
            self.loaded.clear()
        elif program not in self.loaded:
            # MicroPython raises this, rather than silently accepting a miss.
            raise ValueError(ABSENT_PROGRAM)
        else:
            del self.loaded[program]


class StateMachine:
    def __init__(self, board, index):
        self.board = board
        self.index = index
        self.running = False
        self.program = None
        self.fifo = []

    def init(self, program, **kwargs):
        self.board.trip("sm%d.init" % self.index)
        self.board.pios[self.index // 4].add_program(program)
        self.program = program
        self.running = False
        self.fifo = []

    def active(self, value=None):
        if value is None:
            self.board.record("sm%d.read" % self.index)
            return int(self.running)
        self.running = bool(value)
        if value:
            self.board.starts += 1
        # A stop fault is injected after deactivation. active(0) never frees
        # the program, including on this injected error path.
        self.board.trip("sm%d.%s" % (self.index, "start" if value else "stop"))

    def put(self, value):
        self.board.trip("sm%d.put" % self.index)
        self.fifo.append(value)


class PadRegisters:
    def __init__(self, board):
        self.board = board
        self.registers = {GP2_PAD: 1 << 4}  # Original drive: 4 mA.

    def __getitem__(self, address):
        if address != GP2_PAD:
            raise AssertionError("Unexpected modeled register address")
        return self.registers[address]

    def __setitem__(self, address, value):
        if address != GP2_PAD:
            raise AssertionError("Unexpected modeled register address")
        self.board.trip("drive.write", value)
        self.registers[address] = value


class Board:
    def __init__(self):
        self.events = []
        self.faults = {}
        self.pios = [Pio(self, i) for i in range(3)]
        self.sms = [StateMachine(self, i) for i in range(12)]
        self.pins = {2: 0}
        self.mem32 = PadRegisters(self)
        self.now_ms = 0
        self.timers = []
        self.starts = 0
        self.burst_calls = 0
        self.module_imports = 0
        self.firmware_reads = 0
        self.firmware_available = True
        self.approved_firmware = True
        self.reset_calls = []
        self.assembly_sizes = {}
        self.output = io.StringIO()
        self.modules = {}
        self._install_modules()
        self.globals = {"__builtins__": dict(vars(builtins), __import__=self._import,
                                              open=self._open, print=self._print)}

    @property
    def drive_ma(self):
        return DRIVES[(self.mem32[GP2_PAD] >> 4) & 3]

    def record(self, event, detail=None):
        self.events.append((event, detail))

    def fail_once(self, event, error):
        self.faults.setdefault(event, []).append(error)

    def trip(self, event, detail=None):
        self.record(event, detail)
        pending = self.faults.get(event)
        if pending:
            raise pending.pop(0)

    def _install_modules(self):
        board = self

        class Pin:
            OUT = 1

            def __init__(self, number, mode=None, value=None):
                if number != 2:
                    raise AssertionError("Only fake GP2 is modeled")
                self.number = number
                if mode is not None:
                    self.init(mode, value=value)

            def init(self, mode, value=None):
                board.trip("pin.init")
                if value is not None:
                    board.pins[self.number] = value

            def value(self, value=None):
                if value is not None:
                    board.pins[self.number] = value
                return board.pins[self.number]

        class Timer:
            PERIODIC = 1

            def __init__(self, index):
                if index != -1:
                    raise AssertionError("Expected a virtual soft Timer")
                self.running = False
                self.callback = None
                board.timers.append(self)

            def init(self, *, period, mode, callback):
                board.trip("timer.init")
                if period != 250 or mode != self.PERIODIC:
                    raise AssertionError("Unexpected watchdog configuration")
                self.callback = callback
                self.running = True

            def deinit(self):
                self.running = False
                board.trip("timer.deinit")

        def state_machine(index, program=None, **kwargs):
            sm = board.sms[index]
            if program is not None:
                sm.init(program, **kwargs)
            return sm

        def asm_pio(**kwargs):
            def decorate(function):
                return Program(function.__name__, board.assembly_sizes[function.__name__])
            return decorate

        def forbidden_reset(*args, **kwargs):
            board.reset_calls.append("reset/reboot")
            raise AssertionError("Generated code attempted an automatic reset/reboot")

        def sha256(data):
            if data != FIRMWARE_BYTES:
                raise AssertionError("Only in-memory firmware bytes may be hashed")
            digest = bytes.fromhex(APPROVED_SHA256) if board.approved_firmware else bytes(32)
            return types.SimpleNamespace(digest=lambda: digest)

        self.modules.update({
            "json": json,
            "binascii": binascii,
            "hashlib": types.SimpleNamespace(sha256=sha256),
            "os": types.SimpleNamespace(uname=lambda: types.SimpleNamespace(machine="RP2350 (fake)"),
                                        chdir=lambda path: self.record("chdir", path)),
            "sys": types.SimpleNamespace(modules=self.modules, path=["/"]),
            "rp2": types.SimpleNamespace(PIO=lambda index: self.pios[index],
                                         StateMachine=state_machine, asm_pio=asm_pio),
            "machine": types.SimpleNamespace(Pin=Pin, mem32=self.mem32, Timer=Timer,
                                             reset=forbidden_reset, soft_reset=forbidden_reset,
                                             bootloader=forbidden_reset),
            "time": types.SimpleNamespace(ticks_ms=lambda: self.now_ms,
                                          ticks_us=lambda: self.now_ms * 1000,
                                          ticks_add=lambda a, b: a + b,
                                          ticks_diff=lambda a, b: a - b,
                                          sleep_ms=self.advance),
        })
        self.modules["rp2"].PIO.OUT_LOW = Pio.OUT_LOW

    def _open(self, path, mode="r", *args, **kwargs):
        if path != "/rp2350_burst.py" or mode != "rb":
            raise AssertionError("No real filesystem access in generated code")
        self.firmware_reads += 1
        if not self.firmware_available:
            raise FileNotFoundError(path)
        return io.BytesIO(FIRMWARE_BYTES)

    def _import(self, name, globals=None, locals=None, fromlist=(), level=0):
        if level:
            raise AssertionError("Unexpected relative import")
        if name == "rp2350_burst" and name not in self.modules:
            if not self.firmware_available:
                raise ImportError("No fake firmware after USB reset")
            self.modules[name] = self._new_firmware_module()
        if name not in self.modules:
            raise AssertionError("No real module import allowed: " + name)
        return self.modules[name]

    def _new_firmware_module(self):
        self.record("firmware.import")
        self.module_imports += 1
        module = types.ModuleType("rp2350_burst")
        module.__dict__.update(__builtins__=self.globals["__builtins__"], rp2=self.modules["rp2"])
        self.execute(FINITE_ASSEMBLY, module.__dict__)
        module.GPIO = 2
        module._pin = self.modules["machine"].Pin(2, self.modules["machine"].Pin.OUT, value=0)
        module._drive_ma = lambda: self.drive_ma

        def set_drive(value):
            self.mem32[GP2_PAD] = (self.mem32[GP2_PAD] & ~0x30) | (DRIVES.index(value) << 4)

        def burst(count, half_period_us, drive_ma):
            # This models the approved firmware's stop/LOW/drive restoration,
            # deliberately without adding the application's new PIO release.
            self.burst_calls += 1
            self.trip("finite.call")
            before = self.drive_ma
            sm = self.modules["rp2"].StateMachine(0, module._finite_burst,
                                                  freq=1000000, set_base=module._pin)
            try:
                if drive_ma is not None:
                    set_drive(drive_ma)
                during = self.drive_ma
                sm.put(count - 1)
                sm.put(half_period_us - 3)
                sm.put(half_period_us - 4)
                sm.active(1)
                self.trip("finite.run")
            finally:
                sm.active(0)
                module._pin.init(self.modules["machine"].Pin.OUT, value=0)
                if drive_ma is not None:
                    set_drive(before)
            return {"gpio": 2, "pulses": count, "expected_edges": 2 * count,
                    "high_us": half_period_us, "low_us": half_period_us, "output": "low",
                    "drive_strength_mA_before": before, "drive_strength_mA_during": during,
                    "drive_strength_mA_after": self.drive_ma}

        module._set_drive_ma = set_drive
        module.burst = burst
        return module

    def _print(self, *args, **kwargs):
        self.trip("stdout")
        builtins.print(*args, file=self.output, **kwargs)

    def execute(self, source, namespace=None):
        tree = ast.parse(source)
        for node in ast.walk(tree):
            if isinstance(node, ast.FunctionDef) and any(
                    isinstance(d, ast.Call) and isinstance(d.func, ast.Attribute)
                    and d.func.attr == "asm_pio" for d in node.decorator_list):
                self.assembly_sizes[node.name] = sum(
                    isinstance(call, ast.Call) and isinstance(call.func, ast.Name)
                    and call.func.id in PIO_INSTRUCTIONS for call in ast.walk(node))
        offset = self.output.tell()
        exec(compile(tree, "<actual generated MCU command>", "exec"),
             self.globals if namespace is None else namespace)
        return [json.loads(line[len("EBPLUS_MCU "):])
                for line in self.output.getvalue()[offset:].splitlines()
                if line.startswith("EBPLUS_MCU ")]

    def advance(self, milliseconds, fire_timers=True):
        self.now_ms += milliseconds
        if fire_timers:
            for timer in list(self.timers):
                if timer.running:
                    timer.callback(timer)


class ModelControls(unittest.TestCase):
    def test_stop_does_not_free_and_missing_remove_raises_exact_error(self):
        board = Board()
        program = Program("control", 15)
        sm = board.modules["rp2"].StateMachine(0, program)
        sm.active(1)
        sm.active(0)
        self.assertEqual(board.pios[0].used_words, 15)
        board.pios[0].remove_program(program)
        with self.assertRaisesRegex(ValueError, "^" + ABSENT_PROGRAM + "$"):
            board.pios[0].remove_program(program)

    def test_old_reload_path_fails_on_third_burst_not_second(self):
        board = Board()
        programs = []
        for expected_words in (15, 30):
            board.execute(LEGACY_BURST)
            programs.append(board.globals["b"]._finite_burst)
            self.assertEqual(board.pios[0].used_words, expected_words)
        self.assertIsNot(programs[0], programs[1])
        with self.assertRaises(OSError) as caught:
            board.execute(LEGACY_BURST)
        self.assertEqual(caught.exception.errno, errno.ENOMEM)
        self.assertEqual(board.burst_calls, 3)
        self.assertEqual(board.starts, 2)
        self.assertEqual(board.pios[0].used_words, 30)

    def test_pio_capacity_is_per_block_and_program_identity(self):
        board = Board()
        a, b, c = (Program("same_name", 15) for _ in range(3))
        board.pios[0].add_program(a)
        board.pios[0].add_program(a)
        board.pios[0].add_program(b)
        board.pios[1].add_program(c)
        self.assertEqual(board.pios[0].used_words, 30)
        self.assertEqual(board.pios[1].used_words, 15)
        with self.assertRaises(OSError):
            board.pios[0].add_program(Program("continuous", 8))
        board.pios[0].remove_program(a)
        self.assertEqual(board.pios[1].used_words, 15)


class GeneratedCommands(unittest.TestCase):
    commands = {}

    def setUp(self):
        self.board = Board()

    def tearDown(self):
        self.assertFalse(self.board.reset_calls, "No automatic reset/reboot is permitted")
        for pio in self.board.pios:
            self.assertEqual(pio.clear_calls, 0, "No whole-PIO removal is permitted")
            self.assertNotIn(None, pio.remove_attempts)
            self.assertLessEqual(pio.peak_words, 32)

    def command(self, name):
        replies = self.board.execute(self.commands[name])
        self.assertEqual(len(replies), 1, name + " must emit one MCU JSON response")
        return replies[0]

    def assert_stopped(self, words=0):
        self.assertFalse(self.board.sms[0].running)
        self.assertEqual(self.board.pins[2], 0)
        self.assertEqual(self.board.drive_ma, 4)
        self.assertFalse(any(timer.running for timer in self.board.timers))
        self.assertEqual(self.board.pios[0].used_words, words)

    def test_commands_compile_and_assembly_sizes_match(self):
        for name, code in self.commands.items():
            with self.subTest(command=name):
                compile(code, "<" + name + ">", "exec")
        self.command("prepare")
        self.assertEqual(self.board.globals["b"]._finite_burst.words, 15)
        self.command("continuous_start")
        self.assertEqual(self.board.globals["_ebplus_continuous_program"].words, 8)
        self.command("cleanup")
        self.assert_stopped()

    def test_100_prepare_finite_continuous_switches_do_not_leak(self):
        finite_names = ("finite_1_default", "finite_3_default", "finite_1_drive12", "finite_3_drive12")
        for iteration in range(100):
            with self.subTest(iteration=iteration):
                self.command("prepare")
                self.assert_stopped()
                for name in (finite_names[iteration % 4], finite_names[(iteration + 1) % 4]):
                    reply = self.command(name)
                    self.assertEqual(reply["pulses"], 1 if "_1_" in name else 3)
                    self.assertEqual(reply["drive_strength_mA_during"], 12 if "drive12" in name else 4)
                    self.assert_stopped()
                self.command("continuous_start")
                self.assertEqual(self.board.pios[0].used_words, 8)
                self.board.advance(1000)
                self.command("continuous_keepalive")
                if iteration % 2:
                    self.board.advance(8000)
                else:
                    self.command("cleanup")
                self.assert_stopped()
                self.command("prepare")
                self.command("finite_3_drive12")
                self.assert_stopped()
        self.assertEqual(self.board.pios[0].peak_words, 15)

    def test_prepare_releases_cached_programs_before_reimport(self):
        self.board.execute(LEGACY_BURST)
        old_module = self.board.modules["rp2350_burst"]
        self.command("continuous_start")
        self.assertEqual(self.board.pios[0].used_words, 23)
        self.board.sms[0].active(0)
        session = self.board.globals["_ebplus_continuous"]
        session["timer"].deinit()
        session.update(running=False, watchdog_active=False)
        old_module._set_drive_ma(4)
        events_before = len(self.board.events)
        self.command("prepare")
        self.assertIsNot(self.board.modules["rp2350_burst"], old_module)
        events = self.board.events[events_before:]
        imported = next(i for i, event in enumerate(events) if event[0] == "firmware.import")
        removals = [i for i, event in enumerate(events) if event[0] == "pio0.remove"]
        self.assertEqual(len(removals), 2)
        self.assertLess(max(removals), imported)
        self.assert_stopped()

    def test_prepare_preserves_unrelated_inactive_programs(self):
        unrelated = Program("unrelated", 7)
        other_pio = Program("other_pio", 11)
        self.board.modules["rp2"].StateMachine(1, unrelated)
        self.board.modules["rp2"].StateMachine(4, other_pio)
        for _ in range(5):
            self.command("prepare")
            self.command("finite_3_drive12")
            self.command("continuous_start")
            self.command("cleanup")
            self.assert_stopped(7)
            self.assertIn(unrelated, self.board.pios[0].loaded)
            self.assertIn(other_pio, self.board.pios[1].loaded)
        self.assertNotIn(unrelated, self.board.pios[0].remove_attempts)
        self.assertEqual(self.board.pios[1].remove_attempts, [])

    def test_prepare_checks_all_four_sms_before_removal_or_import(self):
        for busy_sm in range(4):
            for cached in (False, True):
                with self.subTest(busy_sm=busy_sm, cached=cached):
                    self.board = Board()
                    if cached:
                        self.board.execute(LEGACY_BURST)
                    continuous = Program("_ebplus_continuous_program", 8)
                    self.board.globals["_ebplus_continuous_program"] = continuous
                    self.board.pios[0].add_program(continuous)
                    self.board.sms[busy_sm].active(1)
                    loaded = dict(self.board.pios[0].loaded)
                    imports = self.board.module_imports
                    with self.assertRaisesRegex(ValueError, "busy"):
                        self.command("prepare")
                    self.assertEqual(self.board.pios[0].loaded, loaded)
                    self.assertEqual(self.board.pios[0].remove_attempts, [])
                    self.assertEqual(self.board.module_imports, imports)
                    self.assertTrue(self.board.sms[busy_sm].running)

    def test_prepare_rejects_armed_watchdog_without_removing_programs(self):
        self.command("prepare")
        self.command("continuous_start")
        self.board.sms[0].active(0)
        imports = self.board.module_imports
        attempts = len(self.board.pios[0].remove_attempts)
        with self.assertRaisesRegex(ValueError, "watchdog is busy"):
            self.command("prepare")
        self.assertEqual(len(self.board.pios[0].remove_attempts), attempts)
        self.assertEqual(self.board.module_imports, imports)
        self.assertEqual(self.board.pios[0].used_words, 8)
        self.command("cleanup")

    def test_only_exact_missing_program_valueerror_is_ignored(self):
        self.command("prepare")
        self.board.globals["_ebplus_continuous_program"] = Program("not_installed", 8)
        self.command("prepare")  # Both cached objects are currently uninstalled.
        self.command("cleanup")
        self.assert_stopped()
        for command in ("prepare", "finite_1_default", "cleanup"):
            for error in (ValueError("different PIO error"),
                          ValueError(ABSENT_PROGRAM + " extra"), RuntimeError(ABSENT_PROGRAM),
                          OSError(errno.EIO, "injected remove failure")):
                with self.subTest(command=command, error=repr(error)):
                    self.board = Board()
                    self.board.execute(LEGACY_BURST)
                    self.board.fail_once("pio0.remove", error)
                    with self.assertRaises(type(error)) as caught:
                        self.command(command)
                    self.assertIs(caught.exception, error)
                    self.assertEqual(self.board.pios[0].used_words, 15)

    def test_finite_finally_releases_on_execution_error_or_interrupt(self):
        for error in (RuntimeError("finite failed"), KeyboardInterrupt()):
            with self.subTest(error=repr(error)):
                self.board = Board()
                self.command("prepare")
                self.board.fail_once("finite.run", error)
                with self.assertRaises(type(error)) as caught:
                    self.command("finite_3_drive12")
                self.assertIs(caught.exception, error)
                self.assertEqual(self.board.burst_calls, 1)
                self.assertEqual(self.board.starts, 1)
                self.assert_stopped()

    def test_continuous_watchdog_releases_saved_program_reference(self):
        self.command("prepare")
        self.command("continuous_start")
        del self.board.globals["_ebplus_continuous_program"]
        self.board.advance(7999)
        self.assertEqual(self.board.pios[0].used_words, 8)
        self.board.advance(1)
        self.assertTrue(self.board.globals["_ebplus_continuous"]["expired"])
        self.assert_stopped()
        with self.assertRaisesRegex(ValueError, "not running"):
            self.command("continuous_keepalive")
        self.assertEqual(self.board.starts, 1)

    def test_keepalive_extends_watchdog_then_expiry_releases(self):
        self.command("prepare")
        self.command("continuous_start")
        for _ in range(3):
            self.board.advance(7000)
            self.command("continuous_keepalive")
            self.assertTrue(self.board.sms[0].running)
        self.board.advance(8000)
        self.assert_stopped()

    def test_keepalive_failure_stops_and_releases_without_restart(self):
        for fault in ("inactive", "drive", "late_heartbeat"):
            with self.subTest(fault=fault):
                self.board = Board()
                self.command("prepare")
                self.command("continuous_start")
                if fault == "inactive":
                    self.board.sms[0].running = False
                elif fault == "drive":
                    self.board.mem32[GP2_PAD] = 0
                else:
                    self.board.advance(8000, fire_timers=False)
                with self.assertRaises(ValueError):
                    self.command("continuous_keepalive")
                self.assert_stopped()
                self.assertEqual(self.board.starts, 1)

    def test_continuous_start_failure_releases_and_does_not_retry(self):
        for event in ("timer.init", "drive.write", "sm0.put", "sm0.start", "stdout"):
            with self.subTest(event=event):
                self.board = Board()
                self.command("prepare")
                error = RuntimeError("injected " + event)
                self.board.fail_once(event, error)
                with self.assertRaises(RuntimeError) as caught:
                    self.command("continuous_start")
                self.assertIs(caught.exception, error)
                self.assert_stopped()
                self.assertLessEqual(self.board.starts, 1)

    def test_stop_finally_releases_even_when_stop_low_or_restore_fails(self):
        for event in ("sm0.stop", "timer.deinit", "pin.init", "drive.write"):
            with self.subTest(event=event):
                self.board = Board()
                self.command("prepare")
                self.command("continuous_start")
                error = RuntimeError("injected " + event)
                self.board.fail_once(event, error)
                with self.assertRaises(RuntimeError) as caught:
                    self.board.globals["_ebplus_continuous_stop"]()
                self.assertIs(caught.exception, error)
                self.assertEqual(self.board.pios[0].used_words, 0)
                self.assertFalse(self.board.sms[0].running)
                self.assertEqual(self.board.starts, 1)

    def test_continuous_stop_does_not_swallow_unexpected_remove_errors(self):
        for error in (ValueError("not the expected message"), RuntimeError(ABSENT_PROGRAM)):
            with self.subTest(error=repr(error)):
                self.board = Board()
                self.command("prepare")
                self.command("continuous_start")
                self.board.fail_once("pio0.remove", error)
                with self.assertRaises(type(error)) as caught:
                    self.board.globals["_ebplus_continuous_stop"]()
                self.assertIs(caught.exception, error)
                self.assert_stopped(8)
                self.command("cleanup")
                self.assert_stopped()

    def test_cleanup_without_firmware_or_session_after_usb_reset(self):
        self.board.firmware_available = False
        self.board.approved_firmware = False
        self.board.pins[2] = 1
        self.board.mem32[GP2_PAD] = 3 << 4  # A stale host still requests original 4 mA.
        initial = self.command("state")
        self.assertFalse(initial["session_present"])
        self.assertEqual(initial["drive"], 12)
        for _ in range(2):
            reply = self.command("cleanup")
            self.assertEqual((reply["level"], reply["drive"], reply["active"]), (0, 4, False))
            self.assertFalse(reply["watchdog_active"])
            self.assert_stopped()
        self.assertEqual(self.board.firmware_reads, 0)
        self.assertEqual(self.board.module_imports, 0)
        self.assertNotIn("rp2350_burst", self.board.modules)
        final = self.command("state")
        self.assertFalse(final["session_present"])
        self.assertEqual(final["drive"], 4)

    def test_cleanup_releases_cached_finite_without_session(self):
        self.board.execute(LEGACY_BURST)
        self.assertNotIn("_ebplus_continuous", self.board.globals)
        self.board.firmware_available = False
        self.command("cleanup")
        self.assert_stopped()
        self.assertEqual(self.board.firmware_reads, 0)

    def test_old_finite_30_plus_generated_continuous_8_fails_safely(self):
        self.board.execute(LEGACY_BURST)
        self.board.execute(LEGACY_BURST)
        starts = self.board.starts
        with self.assertRaises(OSError) as caught:
            self.command("continuous_start")
        self.assertEqual(caught.exception.errno, errno.ENOMEM)
        self.assert_stopped(30)
        self.assertEqual(self.board.starts, starts)

    def test_lost_historical_references_fail_without_clear_reset_or_retry(self):
        self.board.execute(LEGACY_BURST)
        self.board.execute(LEGACY_BURST)
        historical = set(self.board.pios[0].loaded)
        self.board.modules.pop("rp2350_burst")
        self.board.globals.pop("b")
        self.command("prepare")
        calls, starts = self.board.burst_calls, self.board.starts
        allocations = self.board.pios[0].allocations
        with self.assertRaises(OSError) as caught:
            self.command("finite_1_drive12")
        self.assertEqual(caught.exception.errno, errno.ENOMEM)
        self.assertEqual(self.board.burst_calls - calls, 1)
        self.assertEqual(self.board.pios[0].allocations - allocations, 1)
        self.assertEqual(self.board.starts, starts)
        self.command("cleanup")
        self.assertEqual(set(self.board.pios[0].loaded), historical)
        self.assertTrue(historical.isdisjoint(self.board.pios[0].remove_attempts))
        self.assert_stopped(30)

    def test_unapproved_hash_prevents_prepare_and_continuous_start(self):
        self.board.approved_firmware = False
        for command in ("prepare", "continuous_start"):
            with self.subTest(command=command):
                with self.assertRaisesRegex(ValueError, "Unapproved finite-burst firmware"):
                    self.command(command)
                self.assertEqual(self.board.module_imports, 0)
                self.assertEqual(self.board.starts, 0)
                self.assert_stopped()


def load_commands(dumper):
    result = subprocess.run([dumper], check=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                            text=True, encoding="utf-8", timeout=30)
    document = json.loads(result.stdout)
    expected = {"prepare", "finite_1_default", "finite_3_default", "finite_1_drive12",
                "finite_3_drive12", "continuous_start", "continuous_keepalive", "cleanup", "state"}
    commands = document["commands"]
    if document.get("schema_version") != 1 or set(commands) != expected:
        raise ValueError("Unexpected MCU command-dump schema")
    if not all(isinstance(code, str) and code.strip() for code in commands.values()):
        raise ValueError("Command dumper returned an empty or non-string command")
    return commands


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument("--dumper", help="built mcu_command_dump executable (no hardware access)")
    source.add_argument("--self-test", action="store_true", help="only run model negative controls")
    args = parser.parse_args()
    suite = unittest.defaultTestLoader.loadTestsFromTestCase(ModelControls)
    if args.dumper:
        GeneratedCommands.commands = load_commands(args.dumper)
        suite.addTests(unittest.defaultTestLoader.loadTestsFromTestCase(GeneratedCommands))
    return 0 if unittest.TextTestRunner(verbosity=2).run(suite).wasSuccessful() else 1


if __name__ == "__main__":
    sys.exit(main())
