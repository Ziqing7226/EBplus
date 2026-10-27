# benchmarks/ — local performance measurement harness.
#
# NOT built by default and NOT run in CI: enable with
#   cmake -DEBPLUS_BUILD_BENCHMARKS=ON ...
# Every bench prints median timings and doubles as a bit-exactness guard:
# the "save" mode records golden hashes of algorithm outputs, the "check"
# mode re-verifies them, so an optimization that changes a single output
# byte fails loudly. Golden files live OUTSIDE the repository (pass the
# directory as an argument) — never commit them.

if(NOT MSVC)
    # Match the repo's strict-warning standard used by algo/tests.
    set(BENCH_COMPILE_OPTIONS -Wall -Wextra -Werror)
else()
    set(BENCH_COMPILE_OPTIONS "")
endif()

# End-to-end offline RAW read+decode throughput (needs a .raw file argument).
add_executable(bench_playback bench_playback.cpp)
target_link_libraries(bench_playback PRIVATE gui_algo)
target_compile_options(bench_playback PRIVATE ${BENCH_COMPILE_OPTIONS})

# Mat -> QImage conversion chain A/B (old cvtColor+copy vs direct write),
# including a byte-for-byte equality assert.
add_executable(bench_frame_conv bench_frame_conv.cpp)
target_link_libraries(bench_frame_conv PRIVATE gui_core)
target_compile_options(bench_frame_conv PRIVATE ${BENCH_COMPILE_OPTIONS})

# Header-only algorithm micro benches (TimeSurface render, XYT point cloud)
# with golden save/check for cross-build bit-exactness.
add_executable(bench_algo bench_algo.cpp)
target_link_libraries(bench_algo PRIVATE gui_algo)
target_compile_options(bench_algo PRIVATE ${BENCH_COMPILE_OPTIONS})

# AlgoBackend push/pull benches via the public factory (time_surface,
# hot_pixel_filter, freq_detector, background_mask, sensor_self_test) with
# golden save/check over frame bytes + status + output span.
add_executable(bench_backends bench_backends.cpp)
target_link_libraries(bench_backends PRIVATE gui_core)
target_compile_options(bench_backends PRIVATE ${BENCH_COMPILE_OPTIONS})

# RAW index build timing + seek bit-exact continuity (HAL level; needs a
# .raw file argument). Exercises the pure file-side pipeline: no camera.
add_executable(bench_index bench_index.cpp)
target_link_libraries(bench_index PRIVATE gui_algo)
target_compile_options(bench_index PRIVATE ${BENCH_COMPILE_OPTIONS})

# Live camera enumeration + streaming probe (SDK level). The live-camera
# counterpart of bench_playback: a failure here isolates driver/plugin
# issues from UI issues.
add_executable(bench_camera bench_camera.cpp)
target_link_libraries(bench_camera PRIVATE gui_algo)
target_compile_options(bench_camera PRIVATE ${BENCH_COMPILE_OPTIONS})

# Headless FilterChain flip verification: mirrors the exact
# set_stage_enabled/process sequence the preprocessing panel drives.
add_executable(bench_flip bench_flip.cpp)
target_link_libraries(bench_flip PRIVATE gui_core)
target_compile_options(bench_flip PRIVATE ${BENCH_COMPILE_OPTIONS})

# Reproduces the GUI's own file-open path (CameraController::connect_file,
# external-source routing included) outside the UI. Run it with the same
# environment as the GUI to prove/disprove "file won't open".
add_executable(bench_guiopen bench_guiopen.cpp)
target_link_libraries(bench_guiopen PRIVATE gui_core)
target_compile_options(bench_guiopen PRIVATE ${BENCH_COMPILE_OPTIONS})
