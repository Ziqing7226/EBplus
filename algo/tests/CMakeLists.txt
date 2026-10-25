# algo/tests/ — GTest-based unit tests for self-developed algorithm modules.

find_package(GTest REQUIRED)

# GCC/Clang-only flags: MSVC rejects them (and -Werror would be unknown).
if(MSVC)
    set(ALGO_TEST_COMPILE_OPTIONS "")
else()
    set(ALGO_TEST_COMPILE_OPTIONS -Wall -Wextra -Werror)
endif()
if(APPLE)
    # AppleClang diagnoses these existing header-only implementation details
    # more aggressively than the Linux toolchains used for the project.
    list(APPEND ALGO_TEST_COMPILE_OPTIONS
        -Wno-error=unused-private-field
        -Wno-error=unused-lambda-capture
        -Wno-error=unused-but-set-variable
    )
endif()

# Test executable for Phase 6 (algo/common/) modules.
add_executable(test_phase6_common
    test_phase6_common.cpp
)

target_link_libraries(test_phase6_common PRIVATE
    gui_algo
    GTest::GTest
    GTest::Main
)

target_compile_options(test_phase6_common PRIVATE ${ALGO_TEST_COMPILE_OPTIONS})

include(GoogleTest)
ebplus_gtest_discover_tests(test_phase6_common)

# Test executable for Phase 7-9 (algo/cv/) modules.
# Depends on algo/cv/ headers (noise_filter, hot_pixel_filter, orientation_filter,
# direction_selective_filter, sparse_optical_flow, blob_detector, object_tracker,
# corner_detector, cluster_interface, cluster_path_point).
add_executable(test_phase7_cv
    test_phase7_cv.cpp
)

target_link_libraries(test_phase7_cv PRIVATE
    gui_algo
    GTest::GTest
    GTest::Main
)

target_compile_options(test_phase7_cv PRIVATE ${ALGO_TEST_COMPILE_OPTIONS})

ebplus_gtest_discover_tests(test_phase7_cv)

# Test executable for Phase 8-10 (algo/cv/ + algo/analytics/) modules.
# Depends on algo/cv/ and algo/analytics/ headers (line_segment_detector,
# hough_*, orientation_cluster, cluster_lif, background_mask_filter,
# perspective_undistort, optical_gyro,
# xyt_visualizer, time_surface,
# event_to_video, auto_bias_controller, freq_detector).
add_executable(test_phase8_10
    test_phase8_10.cpp
)

target_link_libraries(test_phase8_10 PRIVATE
    gui_algo
    GTest::GTest
    GTest::Main
)

target_compile_options(test_phase8_10 PRIVATE ${ALGO_TEST_COMPILE_OPTIONS})

ebplus_gtest_discover_tests(test_phase8_10)

# Test executable for Phase 4 (algo/calibration/) — IntrinsicCalibration
# chessboard object-grid formula, two-pass calibration + outlier rejection,
# detect_only/accept split, duplicate-pose check.
add_executable(test_intrinsic
    test_intrinsic.cpp
)
target_link_libraries(test_intrinsic PRIVATE
    gui_algo
    GTest::GTest
    GTest::Main
    ${OpenCV_LIBS}
)
target_compile_options(test_intrinsic PRIVATE ${ALGO_TEST_COMPILE_OPTIONS})
ebplus_gtest_discover_tests(test_intrinsic)

# Test executable for the blinking chessboard calibration support
# (algo/calibration/blinking_detect.h): blink-frame construction, the
# min-blink-pixels / single-polarity-ratio validity gate, and chessboard
# detection on synthetic blink frames.
add_executable(test_blinking_detect
    test_blinking_detect.cpp
)
target_link_libraries(test_blinking_detect PRIVATE
    gui_algo
    GTest::GTest
    GTest::Main
    ${OpenCV_LIBS}
)
target_compile_options(test_blinking_detect PRIVATE ${ALGO_TEST_COMPILE_OPTIONS})
ebplus_gtest_discover_tests(test_blinking_detect)

# Raw-event integration tests: exercises every streaming algorithm against a
# real Metavision .raw recording (algo/tests/sparklers.raw, committed). Unlike
# the synthetic-event unit tests above, these assert real-world behaviour:
# non-flat reconstructed frames, no NaN divergence, sane filter rates, and
# in-bounds detected features. The .raw path is injected as an absolute
# compile definition so the test binary finds it regardless of CWD.
add_executable(test_raw_algos
    test_raw_algos.cpp
)

target_link_libraries(test_raw_algos PRIVATE
    gui_algo
    GTest::GTest
    GTest::Main
    MetavisionSDK::base
    MetavisionSDK::core
    MetavisionSDK::stream
    ${OpenCV_LIBS}
)

target_include_directories(test_raw_algos PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
)

target_compile_definitions(test_raw_algos PRIVATE
    GUI_TEST_RAW_FILE="${CMAKE_CURRENT_SOURCE_DIR}/sparklers.raw"
)

target_compile_options(test_raw_algos PRIVATE ${ALGO_TEST_COMPILE_OPTIONS})

ebplus_gtest_discover_tests(test_raw_algos)

# Standalone diagnostic: feed .raw events to EventToVideo and report stats.
# Registered with CTest (audit §三-S7): all 7 diagnostics below assert via
# their exit code and were verified to pass on algo/tests/sparklers.raw.
add_executable(test_raw_e2v
    test_raw_e2v.cpp
)
target_link_libraries(test_raw_e2v PRIVATE
    gui_algo
    MetavisionSDK::base
    MetavisionSDK::core
    MetavisionSDK::stream
    ${OpenCV_LIBS}
)
add_test(NAME raw_e2v COMMAND test_raw_e2v ${CMAKE_CURRENT_SOURCE_DIR}/sparklers.raw)

# Diagnostic: loads a REAL converted ONNX model (§4.4.2-GPU) and exercises
# the OpenVINO GPU / ONNX Runtime CPU runtime selection, recurrent-state
# inference, reset, and the dims-change recompile path. Skips (code 77) when
# models/e2vid_lightweight.onnx is not installed (models/ is user-installed,
# git-ignored — same policy as third_party/onnxruntime).
add_executable(test_e2vid_inference
    test_e2vid_inference.cpp
)
target_link_libraries(test_e2vid_inference PRIVATE
    gui_algo
    ${OpenCV_LIBS}
)
add_test(NAME e2vid_inference
    COMMAND test_e2vid_inference
        ${CMAKE_SOURCE_DIR}/models/e2vid_lightweight.onnx
        ${CMAKE_SOURCE_DIR}/models/hypere2vid.onnx
        ${CMAKE_SOURCE_DIR}/models/e2vid_plus.onnx
        ${CMAKE_SOURCE_DIR}/models/firenet_plus.onnx
        ${CMAKE_SOURCE_DIR}/models/evflownet.onnx)
set_tests_properties(e2vid_inference PROPERTIES SKIP_RETURN_CODE 77)

# Standalone diagnostic: open a .raw file with real_time_playback(true/false)
# and report event delivery + duration. Used to verify SDK behavior matches
# the GUI's default mode.
add_executable(test_raw_playback
    test_raw_playback.cpp
)
target_link_libraries(test_raw_playback PRIVATE
    MetavisionSDK::base
    MetavisionSDK::core
    MetavisionSDK::stream
)
add_test(NAME raw_playback COMMAND test_raw_playback ${CMAKE_CURRENT_SOURCE_DIR}/sparklers.raw)

# Standalone diagnostic: simulate GUI file-playback path for EventToVideo.
# Feeds events in small batches with scaled timestamps and reports frame
# statistics (min/max/mean) to diagnose black/flicker output during playback.
add_executable(test_playback_e2v
    test_playback_e2v.cpp
)
target_link_libraries(test_playback_e2v PRIVATE
    gui_algo
    MetavisionSDK::base
    MetavisionSDK::core
    MetavisionSDK::stream
    ${OpenCV_LIBS}
)
add_test(NAME playback_e2v COMMAND test_playback_e2v ${CMAKE_CURRENT_SOURCE_DIR}/sparklers.raw)

# Standalone diagnostic: verify FileFrameGenerator produces frames at the
# correct playback rate (fps * window / 1e6). Tests slow-motion, real-time,
# fast-forward, EOF, and loop behavior.
add_executable(test_file_frame_generator
    test_file_frame_generator.cpp
    ${CMAKE_SOURCE_DIR}/gui/app/file_frame_generator.cpp
    ${CMAKE_SOURCE_DIR}/gui/app/frame_mode_renderer.cpp
    ${CMAKE_SOURCE_DIR}/gui/app/stream_conditioner.cpp
    ${CMAKE_SOURCE_DIR}/gui/algo_bridge/filter_chain.cpp
    ${CMAKE_SOURCE_DIR}/gui/algo_bridge/backends/backend_common.cpp
)
target_include_directories(test_file_frame_generator PRIVATE
    ${CMAKE_SOURCE_DIR}/gui
    ${CMAKE_SOURCE_DIR}
)
target_link_libraries(test_file_frame_generator PRIVATE
    Qt6::Core
    Qt6::Gui
    MetavisionSDK::base
    MetavisionSDK::core
    MetavisionSDK::stream
    ${OpenCV_LIBS}
    gui_algo
)
add_test(NAME file_frame_generator COMMAND test_file_frame_generator ${CMAKE_CURRENT_SOURCE_DIR}/sparklers.raw)

# Diagnostic: verify FilterChain (flip_x) is applied in
# FileFrameGenerator::render_frame() so display reflects flip toggles
# immediately during file playback.
add_executable(test_filter_in_render
    test_filter_in_render.cpp
    ${CMAKE_SOURCE_DIR}/gui/app/file_frame_generator.cpp
    ${CMAKE_SOURCE_DIR}/gui/app/frame_mode_renderer.cpp
    ${CMAKE_SOURCE_DIR}/gui/app/stream_conditioner.cpp
    ${CMAKE_SOURCE_DIR}/gui/algo_bridge/filter_chain.cpp
    ${CMAKE_SOURCE_DIR}/gui/algo_bridge/backends/backend_common.cpp
    ${CMAKE_SOURCE_DIR}/algo/calibration/intrinsic.cpp
)
target_include_directories(test_filter_in_render PRIVATE
    ${CMAKE_SOURCE_DIR}/gui
    ${CMAKE_SOURCE_DIR}
)
target_link_libraries(test_filter_in_render PRIVATE
    Qt6::Core
    Qt6::Gui
    MetavisionSDK::base
    MetavisionSDK::core
    ${OpenCV_LIBS}
)
add_test(NAME filter_in_render COMMAND test_filter_in_render)

# Integration test: full file-playback flow with FilterChain using a real
# .raw file. Verifies flip_x toggled DURING playback takes effect.
add_executable(test_filter_integration
    test_filter_integration.cpp
    ${CMAKE_SOURCE_DIR}/gui/app/file_frame_generator.cpp
    ${CMAKE_SOURCE_DIR}/gui/app/frame_mode_renderer.cpp
    ${CMAKE_SOURCE_DIR}/gui/app/stream_conditioner.cpp
    ${CMAKE_SOURCE_DIR}/gui/algo_bridge/filter_chain.cpp
    ${CMAKE_SOURCE_DIR}/gui/algo_bridge/backends/backend_common.cpp
    ${CMAKE_SOURCE_DIR}/algo/calibration/intrinsic.cpp
)
target_include_directories(test_filter_integration PRIVATE
    ${CMAKE_SOURCE_DIR}/gui
    ${CMAKE_SOURCE_DIR}
)
target_link_libraries(test_filter_integration PRIVATE
    Qt6::Core
    Qt6::Gui
    MetavisionSDK::base
    MetavisionSDK::core
    MetavisionSDK::stream
    ${OpenCV_LIBS}
)
add_test(NAME filter_integration COMMAND test_filter_integration ${CMAKE_CURRENT_SOURCE_DIR}/sparklers.raw)

# Loop + flip test: verifies flip_x works during LOOP playback and survives
# across loop wraps. Simulates the exact GUI scenario.
add_executable(test_loop_flip
    test_loop_flip.cpp
    ${CMAKE_SOURCE_DIR}/gui/app/file_frame_generator.cpp
    ${CMAKE_SOURCE_DIR}/gui/app/frame_mode_renderer.cpp
    ${CMAKE_SOURCE_DIR}/gui/app/stream_conditioner.cpp
    ${CMAKE_SOURCE_DIR}/gui/algo_bridge/filter_chain.cpp
    ${CMAKE_SOURCE_DIR}/gui/algo_bridge/backends/backend_common.cpp
    ${CMAKE_SOURCE_DIR}/algo/calibration/intrinsic.cpp
)
target_include_directories(test_loop_flip PRIVATE
    ${CMAKE_SOURCE_DIR}/gui
    ${CMAKE_SOURCE_DIR}
)
target_link_libraries(test_loop_flip PRIVATE
    Qt6::Core
    Qt6::Gui
    MetavisionSDK::base
    MetavisionSDK::core
    MetavisionSDK::stream
    ${OpenCV_LIBS}
)
add_test(NAME loop_flip COMMAND test_loop_flip ${CMAKE_CURRENT_SOURCE_DIR}/sparklers.raw)

# Diagnostic: reproduce calibration capture on a .raw recording.
# Accumulates the last W us of CD events into per-polarity masks, builds the
# binary blink frame (matching CalibrationWizard + CalibrationWorker) and runs
# chessboard detection on it, saving annotated PNGs + a tally so we can see
# which windows succeed.
# Not registered with CTest (needs a user-supplied .raw with a calibration board).
add_executable(calib_capture_probe
    calib_capture_probe.cpp
)
target_link_libraries(calib_capture_probe PRIVATE
    gui_algo
    MetavisionSDK::base
    MetavisionSDK::core
    MetavisionSDK::stream
    ${OpenCV_LIBS}
)
target_compile_options(calib_capture_probe PRIVATE ${ALGO_TEST_COMPILE_OPTIONS})

# Diagnostic helper (Phase 4 D10): loads a PNG saved by calib_capture_probe and
# tries findCirclesGrid with many board sizes / symmetric vs asymmetric / tuned
# detector, so we can identify the actual calibration board geometry without
# re-reading the full .raw. Links OpenCV only.
add_executable(analyze_calib_png
    analyze_calib_png.cpp
)
target_include_directories(analyze_calib_png PRIVATE ${PROJECT_SOURCE_DIR})
target_link_libraries(analyze_calib_png PRIVATE
    ${OpenCV_LIBS}
)
target_compile_options(analyze_calib_png PRIVATE ${ALGO_TEST_COMPILE_OPTIONS})

