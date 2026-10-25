# Performance notes (3.2.0)

This page documents the display/algorithm-path performance work shipped in
3.2.0, how to reproduce the measurements, and the optional deployment-side
tuning knobs.

## What was optimized (all bit-identical outputs)

| Path | Before | After | Change |
|---|---|---|---|
| TimeSurface render, 1280×720 | 5.8 ms/frame | 2.1 ms/frame | 256-entry palette LUT built from the same `map_color` |
| Mat → QImage frame conversion | 1482 µs/frame | 373 µs/frame | cvtColor writes into the QImage-owned buffer |
| `pull_result()` per-backend copy | 30k-event memcpy/frame | none | dead `filtered_events` field removed (never read) |
| XYT 3D view idle | 60 VBO rebuilds/s | 0 | dirty-flag rebuild |
| SensorSelfTest render / pull | 2.66 ms / 7.1 ms | 2.50 ms / 5.5 ms | row pointers, dead report recompute removed |
| background_mask pull (frozen) | 801 µs | 453 µs | frozen-phase frame cache |
| hot_pixel pull | 477 µs | 376 µs | output span without copy-back |
| APS preview idle ticks | full resample each tick | skipped | same-frame + same-size guard |

Offline RAW decode throughput is NOT a bottleneck on a modern machine
(>100 Mevents/s end-to-end through the SDK callback path) — the wins above
are on the rendering/algorithm path the frame rate actually depends on.

## Benchmark harness

The `benchmarks/` directory (build with `-DEBPLUS_BUILD_BENCHMARKS=ON`,
never built in CI) reproduces every number above. Two of the benches have
a golden mode that guards bit-exactness across future changes:

```
bench_algo save <dir>      # record output hashes on a known-good build
bench_algo check <dir>     # fail on any output-byte difference
bench_backends save|check <dir>
bench_playback <file.raw>  # Mevents/s offline decode throughput
bench_frame_conv           # Mat->QImage A/B with equality assert
```

## Deployment-side knobs (optional, no code changes)

- **Trusted EVT3 recordings only**: setting `MV_FLAGS_EVT3_UNSAFE_DECODER=1`
  skips the per-event syntax validator and speeds up EVT3 decoding by
  ~10–40%. Only use for files you trust — corrupted streams are no longer
  rejected event-by-event.
- **High event rates over USB**: `MV_PSEE_DEBUG_PLUGIN_USB_ASYNC_TRANSFER`
  (default 20) and `MV_PSEE_DEBUG_PLUGIN_USB_PACKET_SIZE` (default 128 KiB)
  tune the libusb flight depth; `MV_PSEE_PLUGIN_DATA_TRANSFER_BUFFER_POOL_BYTE_SIZE`
  bounds the transfer buffer pool (unbounded by default).
- **One HAL plugin per plugin directory**: keep only the plugin .so/.dll
  you intend to use in the Metavision plugin directory — two plugins
  registering the same devices re-enumerate and can claim against each
  other.

## What was measured and deliberately NOT changed

- **LTO/IPO** on the application code: measured no gain on the hot paths
  (they live in already-inlined headers) — not enabled.
- **GPU rendering**: at ~1M events/s the CPU accumulation costs <2% of a
  core and the texture upload is ~0.002% of PCIe — GPU would not pay for
  its complexity today. The Qt OpenGL link is in place if a future
  high-rate sensor changes the calculus.
- **Upstream-intentional OpenEB behaviour** (real-time playback default,
  1024-event decode granularity, EVT2/EVT3 forwarder batching) is left as
  upstream designed it.
