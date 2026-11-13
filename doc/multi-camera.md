# Three-Camera Workspace

This fork adds **Tools > Multi-Camera...** to the existing Qt Widgets app. It is
independent of the single-camera controller and recorder. Closing its toolbox
does not discard the workspace: restore **Multi-Camera Toolbox** from **View**.
Use **Single camera** to release the rig and restore the previous dock layout.
Stop an active recording, synchronization operation or continuous Trigger first.

## Equipment And Project Conditions

Use exactly three SilkyEvCam cameras, a rigid mounting, a clock synchronization
connection, and the already provisioned RP2350 controller. Assign serials to
Left / Center / Right independently of the clock Master and geometry reference.
The app verifies complete HAL identity, resolution, clock role and Trigger In
readback; discovery never selects the first camera automatically.

Enter your project's maximum Trigger time difference (microseconds) and maximum
reprojection RMS (pixels). Neither has an assumed acceptance default. The
default observation targets are 50 common training poses and 10 NEW validation
poses. These are collection targets, not a substitute for quality validation.

Use two screens. Keep the main GUI outside all three camera fields of view.
Only the independent calibration-pattern window belongs on the movable second
screen. Move that screen or the entire rigid camera rig, never one camera
independently. A software overlay cannot remove optical interference caused by
the main display being visible to a camera.

## Five Steps

1. **Devices & project:** choose three distinct serials, clock Master and
   geometry reference; enter project conditions; confirm the physical layout
   and wiring. Connect the configured rig, then check the bound controller.
2. **Synchronization:** confirm the GP2 wiring/current limiting, then run the finite Trigger check. The three vertically
   arranged plots share original camera timestamps and show decoded Trigger In
   polarity, not voltage. Six formal edges must match uniquely on every camera
   within the project limit. Extra decoded records are an independent warning;
   timing approval does not certify record completeness. Alternatively, use
   **Start continuous Trigger** to output a 0.5 Hz square wave (1 s high / 1 s
   low), inspect the plots, then **Stop Trigger**. Hover near an edge for its
   exact decoded timestamp/polarity. Stopped traces remain visible for review.
   After a completed check with confirmed LOW and at least two observed Main
   edges per camera, **Accept synchronization manually...** allows an explicit
   operator decision with a required reason. The original automatic result is
   retained; the status says **Accepted manually**, never automatic pass.
   Approval is tied to the current epoch AND attempt and expires on retry,
   reconnection, or relevant wiring/timing configuration changes. It cannot
   approve unknown output state, absent traces, calibration or RAW integrity.
3. **Guided calibration:** open the 9 x 6 inner-corner blinking board on the
   second display. Windowed or full-screen is supported. Set its final size,
   physically measure one square, enter millimetres, then lock. Resizing,
   changing display geometry or pixel density requires remeasurement.
   Follow one image-space movement target at a time. Common capture requires
   all 54 corners in all cameras, adequate board size, a new pose and a stable
   one-second interval. The initial stability gate is 2 px corner RMS at
   1280 x 720, scaled with resolution; it still needs real-rig validation.
   Capture runs in a short, explicitly started round (five seconds by default),
   with a visible countdown. The round ends automatically without discarding
   accepted observations. Resolve the reported rejection reason, or move to a
   new pose after an acceptance, then start another round. Use intrinsic-only
   scope for supplemental single-camera views; they do not
   increase the common count. Select an observation to delete it, or delete
   the last observation. Solving freezes training parameters. Collect fresh
   hold-out poses afterwards, then validate. Inspect per-camera/per-pose RMS
   and the projected-versus-observed error overlay.
4. **Recording:** confirm the lens, focus and rigid assembly have not changed,
   choose an output directory, and optionally set a formal recording duration.
   Every Take reopens the rig in a new epoch and repeats Trigger preflight.
   If its automatic check fails with a reviewable trace, the Take stays in
   preflight until fresh manual acceptance or cancellation. Formal time remains
   zero while waiting; preflight RAW is retained and disk space is monitored.
   The RAW files retain that preflight. Formal time begins at a measured common
   decoder boundary, not a fixed 14-second offset. The pattern window closes
   when preparing a Take; prepare the physical recording scene before formal
   capture. During recording the only workflow interruption is **Stop**.
5. **Playback & report:** open a closed Take directory without connecting any
   cameras. Play, pause, change speed, drag the shared timeline, or enter an
   absolute timestamp in seconds. Enter performs only the seek, not Play.
   Include the preflight segment when it was recorded. Older TriCapture Takes
   preserve file order and timestamps; missing spatial positions, calibration
   or formal boundaries remain **not recorded**, not inferred from Master.

**Run / resume SOP** automates prepared steps, pauses for a measured/locked
display or new poses, solves training, collects independent validation and
preflights the recording. Pause retains observations. Resume does not bypass
blockers. Reuse requires an identity/resolution/board-matching validated profile
and a fresh human confirmation that optics and the rigid mounting are unchanged.
Reconnecting invalidates synchronization, not unchanged geometric calibration.

Navigation uses selection highlighting plus text, green completion and red
failure. Every page lists blockers. Selecting a blocker takes you to its field
or action. Red input borders represent invalid fields; pending operations are
listed separately from validation errors.

### Workflow Guidance

The toolbox uses a top-to-bottom task journey on all five pages: a numbered
current task, its necessary fields, the result, then one primary action. The
normal journey uses subpages rather than a long scrolling form. Task selection
allows backtracking; detailed device evidence, manual exceptions, project files
and calibration management remain available as secondary tasks. Long reports
and advanced lists may scroll within their own detail views.

The stage title, status and next action stay visible. The primary action moves
to the next unfinished task or executes the currently prepared command. The
pending-details menu lists every blocker; selecting one reveals its owning
stage and subpage and focuses the exact field or action. **Fix** means an
invalid input and gives that field a red border. **To do** means an operation
or human confirmation, without an error border. **Running** explains which
operation must finish or be stopped. Navigation selection remains independent
of completion/failure text.

Calibration guidance advances through seven concrete phases: select a secondary
display/open the board; measure a square/lock the display; collect N/M common
training poses; solve training; collect N/M NEW independent validation poses;
validate against the project RMS limit; confirm unchanged optics/rig. Matching
geometry, finite RMS within the limit and optics confirmation are all required
before Next. A validated imported profile does not require reopening the board.
The existing `blockers(Step::Calibration)` remains the collection-entry gate;
`stage_gate()` supplies the separate UI transition/completion contract.

An unconfirmed Trigger stop is always the first blocker, including during
offline-review navigation. A failed finite check points to a bounded wiring /
current-limiting check and explicit retry, or to retained-trace review/manual
acceptance when the existing controller evidence/LOW gates allow it. No retry,
Trigger command or acceptance is performed by the guidance UI itself.

Recording prerequisites and new-Take preflight are distinct: readiness enables
**New preflight + record**, but each Take still needs fresh synchronization and
formal time remains zero during preflight. Next opens review only after Take
closure; failed/incomplete closed Takes remain reviewable without claiming
successful completion. Offline playback requires no connected rig.
The recording task retains a separate, readiness-gated **New preflight + record**
action after a closed Take. Playback's opening task retains **Open another Take**
after one has loaded; the guided Next action does not replace these repeat-use
commands.

Detailed hidden-page text/report generation is deferred until that page is
selected. Observation labels, profile arrays/selected validation overlays and
report JSON are cached by content; unchanged labels/tooltips do not call
`setText`, and field borders are repolished only when invalidity changes.
`WorkflowContract.*` and `WorkflowUI.*` add hardware-free headless coverage.
With `EBPLUS_TEST_SCREENSHOT_DIR` set, the compact-page test exports
`workflow-compact-1.png` through `workflow-compact-5.png` at 1180 x 740.
These checks do not establish dual-display or hardware acceptance.

## Controller Boundary

Only USB VID `2e8a`, PID `0005`, serial `36f9f437460d3ba8` is accepted. Native
Qt SerialPort speaks bounded MicroPython raw REPL commands, with framing,
timeouts and cancellation. It checks the existing `/rp2350_burst.py` source
before importing it; the approved SHA-256 is:

```text
06b46f189635824b2ded6baeccf2794e3e52a78b004f1413fb8c00aac4a3651b
```

The sequence is a five-second settle, one preamble pulse, spacing, then three
formal pulses with one-second high/low periods. The GUI requests the existing
finite burst function's explicit **12 mA** drive setting after operator wiring
confirmation; each burst restores the previous setting. The diagnostic CLI
still preserves drive by default unless `--drive-ma 12` is explicitly supplied.
There is no firmware installation, host Python, uv or ROS dependency. Drive
strength is not a current limit. Physical wiring and current limiting must be
confirmed independently before running a burst. Stop/failure attempts LOW;
missing readback is reported as unknown and blocks approval.

Continuous mode installs only a RAM-resident PIO program and a MicroPython soft
Timer watchdog, without modifying boot files. The host sends a heartbeat about
once per second; after about eight seconds without one, the watchdog attempts
PIO stop, LOW and original-drive restoration. This is a software safeguard,
not a hard-real-time electrical interlock. Stop, page departure, disconnect and
application shutdown also attempt explicit cleanup; an unconfirmed stop stays
blocking and can be retried. Stop retries resolve the fixed USB identity again,
including after a port-path change, without restarting the waveform or changing
the saved drive baseline. Residual RAM sessions are restored before a new
baseline is accepted. The physical waveform and watchdog behavior still
need operator hardware testing; no automatic test emits physical pulses.
See [MicroPython RP2 timers](https://docs.micropython.org/en/latest/rp2/quickref.html#timers)
for the soft-callback scheduling limitations.

PIO instruction memory is separate from the Python heap. Stopping a state
machine alone does not unload its program. The host releases only the known
finite/continuous programs after stopping, and before replacing an imported
program object; it never clears an entire PIO instance. Other active PIO0 state
machines block preparation/release. If an older app has already lost references
to loaded programs, `ENOMEM` can persist: stop Trigger, close EBplus, power-cycle
the controller, then check it again. The app does not reset the board or retry
pulses automatically. No firmware reinstallation is required.
See [MicroPython PIO](https://docs.micropython.org/en/latest/library/rp2.PIO.html)
for program-memory allocation and removal.

## Live Preview And Calibration Load

Each camera has a dedicated consumer thread. The HAL device, decoder, RAW
logger and full-resolution counters are owned by that thread; a slow decode
does not serialize the other cameras or the coordinator. A consumer drains
ready HAL data in bursts (at most 64 buffers with a four-millisecond budget
checked between buffers), and continues immediately while data progresses.
Only an idle consumer waits briefly. Every pulled RAW buffer still has one
consumer: this does not sample RAW, discard queued RAW or change timestamps.
A single unusually expensive decode can exceed that budget. OpenEB's internal
RAW ingress queue is not bounded by our preview mailbox; sustained overload
still requires operator investigation, not a claim that no backlog can occur.

Live previews retain at most 8192 events in contiguous storage and reuse the
rendered image until its input changes. Each consumer samples a latest-only
preview at up to 10 Hz; the coordinator publishes status at approximately 15 Hz.
Preview replacement cannot enqueue a growing sequence of stale GUI frames;
when the previous GUI notification is pending, only lightweight statistics are
refreshed. Hidden toolbox pages defer report and observation rendering, and
unchanged text, lists and report JSON are not regenerated on every preview.

Calibration still counts all events at the original resolution in common
100 ms windows. Its hot loop caches the current time-window pixel arrays instead
of searching a container per event. Completed windows share sealed storage with
the detector; late writes fail rather than mutate a detector's input. Storage
is bounded to four completed windows plus one writing window per camera, with in-flight sealed windows kept
alive by reference counting. Preview images and overlays never enter this path.
Calibration status/report serialization is cached until observations, results
or relevant configuration changes.

Only one detection is in flight. Pause/restart discards a previous operation's
late result or exception, preserves observations and resets the one-second
stability countdown. A cancelled solver is allowed to finish in the background,
but its result is discarded rather than applied to a later operation.

Calibration capture is a bounded host-clock round (default five seconds,
configurable from three to thirty). The full-resolution common window is still
100 ms, and the corner-based stability requirement remains one second. These
are different durations. On expiry the round stops, any late detection is
discarded, observations remain in memory, and SOP waits for the operator to
reposition and explicitly resume. Expiry alone never completes calibration.
Camera decode progress that differs by more than 300 ms or proceeds below
0.8x real time blocks new candidates; a detection more than 400 ms behind the
furthest decoded watermark, or taking over 400 ms of host time, is discarded. These are freshness safeguards, not
hardware synchronization acceptance thresholds.

Every hardware capture request first disables counting and drains the existing
HAL ingress. Counting starts only after that camera observes an empty HAL
queue, excluding the partial window at the barrier. Until all three barriers
are ready, the GUI says which camera is draining old data. An empty HAL queue
does not measure or remove sensor/driver/USB transport latency, and a 1x decode
rate by itself does not establish freshness.
Sensor-time advancement is also compared with host elapsed time since that
post-click barrier. Falling over 300 ms behind it blocks dispatch and acceptance
even if the instantaneous decode ratio has returned to 1x. This is a relative
processing-lag estimate; transport latency before the barrier remains unmeasured.

Preview telemetry reports decoded CD events/s and consumed RAW bytes/s per
host-clock second, plus sensor-time advancement per host second. It does not
measure the USB link's physical speed or camera-to-screen latency. The age of
the last decoded buffer likewise does not prove that the ingress queue is empty.

Software regressions compare full-resolution counts against a per-event
reference, check sealed-window and preview immutability, GUI backpressure and
delayed detection cancellation. Synthetic timings are not live-camera frame-rate
or memory-usage acceptance; sustained three-camera calibration still needs
operator testing at the real event rate.

### Video Diagnosis, 2026-10-20

The supplied 33.3 s screen recording was inspected at exact video frames 0,
300, 600 and 900 (30 fps). Over the first 30 s, displayed camera timestamps
advanced 15.764 / 15.630 / 16.022 s for Left / Center / Right, while decoded
event counters advanced 599,184,490 / 638,632,903 / 591,485,182. That is roughly
19.97 / 21.29 / 19.72 million decoded events per host second, or 38.01 / 40.86 /
36.92 million per sensor-time second. The latter describes the recorded event
density, not measured USB throughput. The timestamp progress ratio was only
0.52-0.53x, with 3.7-3.9 s between displayed camera decode positions.

This demonstrates a processing backlog in the old serial-consumer version;
it is not proof of clock desynchronization or a particular cable/Hub fault.
With only a few completed calibration windows retained, such decode skew can
prevent common groups despite apparently visible boards. The video stays at
zero accepted common poses; it does not show successful calibration. Screen
recording alone does not establish exact motion-to-preview latency.

The three per-camera detection jobs now run concurrently and join before the
single group completes. The full-resolution serial detector remains the test
oracle. An isolated local synthetic benchmark (1280x720, OpenCV 4.13, median of
seven alternating rounds) measured clean groups at roughly 9.2 ms serial vs
3.4 ms parallel, and fallback recovery around 126 ms vs 45 ms. These figures
exclude HAL decoding and do not claim a measured whole-app speedup. OpenCV's
default internal thread pool can still compete for CPU under dense clutter;
the production change does not alter global OpenCV settings or single-camera
algorithm defaults.

### Storage And Cleanup

Normal calibration does not create RAW files. It keeps bounded count windows,
the latest preview, accepted corner observations and solved parameters in
memory. Export calibration writes JSON to the chosen path; the calibration
page lists that path and file size after a successful export. Resetting
observations frees the in-memory calibration after confirmation, but never
deletes exported files or Takes. Export before quitting if observations must be
preserved.

Only recording / Take preflight creates three RAW files and its session,
Trigger evidence, calibration snapshot and report. The default output folder
is `~/TriEventCaptures`; the project may select another location. Open the
output folder from the toolbox to review and remove unwanted closed Takes in
Finder. Keep each Take's relative files together. The application never removes
RAWs, exports or incomplete Takes automatically. RAW size depends on the
measured byte rate and duration, not the number of accepted calibration poses.

## Implementation And Files

- `gui/multicam/`: contracts and gates, dedicated per-camera HAL consumers, native
  MCU transport, bounded detection/solver work, guidance, native Take reader,
  read-only verification, and the Qt workspace.
- `algo/calibration/rig_calibration.*`: existing two-pass intrinsic calibration,
  raw-distorted pinhole projection, PnP initialization and robust joint Ceres
  optimization using `QuaternionManifold`. Intrinsics stay fixed during the
  joint solve. The reference transform is identity; all other transforms are
  `T_camera_from_reference`, in metres.
- Preview images are disposable copies. Raw counts and 100 ms simultaneous
  detection windows never contain GUI overlays. Only preview/calibration
  candidates can be dropped under load; RAW is consumed/logged once.
- `session.json` schema 2 records camera identity/readback, clock roles, spatial
  positions, original timestamp bounds, counts, epoch, preflight Trigger
  evidence, actual formal bounds and completion/failure reason. Calibration
  snapshots, three RAW files, decoded Trigger CSVs and `report.json` accompany
  it. Failures retain the incomplete Take instead of deleting it.
- Closed RAWs are decoded read-only after the consumer stops. Counts, bounds,
  timestamp regressions and protocol violations are reported separately. The
  current HAL does not expose residual decoder bytes: the decoder-tail field
  remains **unknown**, even when counts match. That is not a full integrity
  certification. No post-stop second consumer is used.

The single-camera calibration defaults are unchanged. This implementation does
not apply the single-camera wizard's forced AutoBias behavior to the rig.

## Build And Software Checks

Dependencies add Ceres 2.2 or newer (modern Manifold API), Eigen and Qt SerialPort.
See [macOS packaging](macos.md) for the complete build. The existing macOS CI
installs these dependencies and runs the new GTest/Qt tests through CTest.

```bash
brew install qtserialport ceres-solver googletest
cmake -S . -B build/macos-tests \
  -DGUI_FOR_OPENEB_BUILD_TESTS=ON -DGUI_BUILD_TESTS=ON \
  -DCMAKE_PREFIX_PATH="$PWD/build/openeb-install;$(brew --prefix);$(brew --prefix qtbase);$(brew --prefix qtsvg);$(brew --prefix qtserialport)"
cmake --build build/macos-tests --parallel 3
ctest --test-dir build/macos-tests --output-on-failure
```

Optional local fixtures are not copied into the repository or required in CI:

```bash
EBPLUS_TEST_TAKE=/path/to/closed-tricapture-take \
  build/macos-tests/gui/tests/test_multicam \
  --gtest_filter=MultiCameraRuntime.NativeReplayOfExistingTriCaptureTakeIsReadOnly
EBPLUS_TEST_SCREENSHOT_DIR="$PWD/build/multicam-screenshots" \
  build/macos-tests/gui/tests/test_multicam \
  --gtest_filter=MultiCameraUI.RenderAllWorkflowPages
```

Tests cover known-geometry recovery (0.1 mm / 0.01 degree limits), noise,
reversed corners, degenerate intrinsic views, independent validation,
stability/repeats/missing common views, overlay isolation, finite Trigger
matching, raw-REPL fragments/errors, gates, injected rig/clock/MCU lifecycle,
precise seek, legacy paths and restoring the single-camera workspace.

When a Python 3 interpreter is available in the test environment, CTest also
runs `mcu_pio_model`. A test-only executable exports the actual native command
builders to an in-memory PIO model. It reproduces the old allocation leak and
checks 100 finite/continuous transitions, stop/watchdog/error paths and unrelated
program preservation. This standard-library-only test does not access hardware
or validate electrical timing; Python is not an application dependency.

### Local Software Evidence (2026-10-20)

- Full CTest: 542 registered, 538 passed, zero failed, four skipped. The skipped
  cases require external AEDAT4/ALPDATA fixtures or a DAVIS camera; they are not
  evidence of hardware support. The new rig solver's 15 cases and multi-camera
  controller/guidance/UI suite's 64 cases also passed separately.
- An existing three-camera TriCapture RAW Take passed read-only native decoding,
  count/boundary checks, original-time seeking and unchanged-file checks. This
  verifies compatibility with that recorded fixture, not a new live capture.
- All five real Qt workspace pages were rendered and inspected with the
  offscreen test backend. Blocker location, navigation state, toolbox restoration
  and return to the single-camera layout passed. This is not native-window,
  dual-display or live guidance acceptance.
- The vendored OpenEB clean-build baseline uses OpenEB 5.2.0, Qt 6.11.2,
  OpenCV 4.13.0 and Ceres 2.2.0 on Apple Silicon. CI dependency/test wiring is
  updated locally; no remote CI run is claimed before a future authorized push.
- The local Silky test package at
  `build/multicam-silky-local-20261020/dist/` passed deep/strict ad-hoc signature
  verification, bundled dependency checks and DMG checksum verification. The
  app launched from that package. It includes the locally installed vendor
  plugin only for local testing, with no vendor binary added to Git or uploaded.
  The DMG SHA-256 is
  `6fc43666448dea324012e8a61fc8bd265159e4452385683db4035611477474dc`.

### Initial Live Diagnostic (2026-10-20)

The user confirmed Left `00000327`, Center/apex `00000330`, Right `00000328`,
an isosceles arrangement, and `00000330` as both clock Master and geometry
reference. Dimensions and geometric extrinsics have not been measured.

The native backend opened all three cameras, read back the assigned clock
roles and Trigger In, verified the bound RP2350 firmware, and executed one
preamble plus three formal pulses after explicit wiring/current-limit
confirmation. GP2 LOW was confirmed afterwards; drive strength read back as
4 mA before/during/after, unchanged. That value is not a measured input current.

**Timing was not measurable from this run.** RAW decoding found 4,969,860 /
699 / 5,126,694 Trigger records in Left / Center / Right, including dense
opposite-polarity records 0-2 us apart. The required unique six-edge formal
sequence could not be matched. No project tolerance or approval was assigned.
The diagnostic Take is incomplete, preserves three RAWs/Trigger CSVs and report
under `build/hardware-validation/20261020/take-20261020-081001-819`, and has no
formal recording range. All three online CD counts and time bounds matched
read-only decoding, with no reported CD timestamp regression or protocol error;
decoder tails remain unknown. These checks do not establish capture coverage
of the complete MCU sequence or electrical signal quality.

The SDK also warned that Left `00000327` was not on USB 3. The user confirmed
a Hub connection; actual link speed is still to be checked. Wiring/waveform
inspection is needed before further pulse trials. This test does not identify
the root cause as wiring, electronics, USB or driver behavior.

It exposed two software fixes: collapse macOS callout/dial-in aliases of the
same serial port, and display/serialize unmatched timing as unknown rather
than a measured zero. The optional `multicam_hardware_probe` target is excluded
from the default build and CTest; `--list` is read-only and physical pulses
require `--allow-trigger`. It never grants calibration or production-capture
approval.

After these fixes, full CTest registered 544 cases: 540 passed, zero failed,
four external-fixture/hardware cases skipped. Native read-only replay was also
checked against the newly captured diagnostic RAWs, not only the older
TriCapture fixture. The revised local package is at
`build/multicam-hardware-20261020/dist/`; bundle signature/dependency checks and
DMG checksum verification passed. Its DMG SHA-256 is
`76e220939544e5d9e2922788e85d16e55d2e2f12dd8db4cb562315ce42eed950`.

### Explicit 12 mA Diagnostic (2026-10-20)

After the operator confirmed independent 330 ohm Trigger branches and expressly
authorized the drive override, one further finite diagnostic used the existing
firmware's 12 mA burst option. Both the preamble and three-pulse formal burst
read back 4 -> 12 -> 4 mA; final cleanup confirmed LOW and restoration to 4 mA.
The override is available only through explicit diagnostic construction/CLI
`--drive-ma 12`; the GUI still preserves drive strength by default. No firmware
was flashed. A drive setting is not a current measurement.

The preserved Take at
`build/hardware-validation/20261020/drive12/take-20261020-085038-269` has five
Trigger records per camera, without the earlier dense extra-record bursts in
the captured range. Recorded corresponding edges differ by 0-1 us, but full
six-edge timing is still unknown: CD ranges end around 11.3 s, before the
last two expected formal edges. The post-burst wait does not prove ingress has
caught up before HAL stop discards pending buffers. Also, observed formal-prefix
polarities begin 0,1, while the matcher currently assumes 1,0. Both coverage and
polarity need explicit verification; no hardware root cause is established.

All online CD counts and bounds match read-only RAW replay, with zero reported
CD regressions/protocol errors and unknown decoder tails. The Take remains
incomplete and does not unlock recording. Thirty focused MCU/runtime tests,
including native replay of this exact Take, passed. No further pulses were sent.
This source update has not been repackaged into the earlier DMG.

### Operator-Accepted Observed Synchronization

After reviewing the 12 mA diagnostic, the operator explicitly accepted this
run's synchronization based on correspondence of the observed post-startup
edges, allowing the first edge to be missing. All four recorded formal edges
have Left/Master and Right/Master offsets of -1 us, and Left/Right offsets of
0 us, over 2.999615 s. This is synchronization acceptance for that observed
window, not a claim of exact zero offset or validated long-duration drift.

The per-run decision is preserved in that Take's operator-assessment.json.
Original automatic diagnostics, RAWs and CSVs are unchanged. Missing final
coverage remains a separate recording warning, not a reversal of the operator's
observed-window synchronization acceptance. No additional pulses were sent.
Automatic GUI gates/project-wide tolerance have not been changed by this
per-run assessment; geometry calibration and complete recording acceptance
remain outstanding.

### GUI Trigger Controls: Software Validation

The revised local GUI is in `build/multicam-trigger-ui-20261020/dist/`.
It contains the 12 mA finite mode, explicit continuous-wave start/stop controls,
and per-epoch/per-attempt manual acceptance with a required reason. The automatic
result is preserved in the Take's synchronization evidence; unknown extra-record
counts remain unknown. New Takes require a fresh preflight/acceptance and still
require valid calibration and independent RAW checks.

The multi-camera suite passed 109/109 tests, including simulated USB recovery,
residual-drive restoration, stop/cancel/heartbeat failure, page departure,
disconnect/destruction, stale/blank approval rejection and fresh per-Take
approval. Full CTest registered 587 tests: 583 passed, none failed, four external
AEDAT4/ALPDATA/DAVIS cases skipped. The existing real 12 mA diagnostic Take passed
read-only replay again. Qt pages and Trigger states were rendered and inspected
at 1440 x 900 and 1180 x 740. These are software/Qt rendering checks, not new
physical continuous-wave, USB unplug or watchdog timing acceptance.

The GUI implementation did not send new physical pulses, modify board firmware,
commit, push, publish, tag or create a pull request. The local Silky plugin is
included only for local testing, not cleared for redistribution.

## Hardware Acceptance Still Required

Software tests and a signed app/verified DMG do **not** validate the rig.
Before using recordings as research data, perform and retain evidence for:

- Three physical cameras with identity, clock-role and Trigger readback.
- The bound board's native serial exchange, finite pulses and LOW cleanup.
- Dual-display placement, full-resolution detections and understandable
  non-conflicting live movement targets.
- Real stable/duplicate thresholds, diverse common poses and fresh hold-out
  RMS/error overlays satisfying the project's conditions.
- Manual/timed captures, device loss, output/write failure, retained incomplete
  Takes, read-only RAW count checks and offline shared replay.

Items not covered by retained evidence remain **unverified**. The unchanged-4-mA
diagnostic failed synchronization. The subsequent 12 mA run has the limited
operator synchronization acceptance described above, not calibration or
complete-capture acceptance. This is a local ad-hoc-signed testing package, not a notarized production release. No commits,
pushes, pull requests, tags or releases are created by the implementation task.

## References

- Existing EBplus blinking-board detection and two-pass intrinsic calibration.
- The user's TriCapture HAL consumer boundary, session format and shared-time
  replay architecture, adapted to this C++17 codebase without its host-Python
  controller path.
- [MPL calibration toolbox](https://github.com/mgaoling/mpl_calibration_toolbox):
  joint fixed-intrinsics rig/board optimization as an algorithmic reference,
  not a ROS runtime dependency or a copy of its older Ceres parameterization.
- [Ceres Manifold API](https://ceres-solver.org/nnls_modeling.html#manifold).
- [MicroPython raw REPL protocol](https://docs.micropython.org/en/latest/reference/repl.html#raw-mode-and-raw-paste-mode).
