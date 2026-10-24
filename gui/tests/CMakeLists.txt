# gui/tests/ — GTest-based unit tests for GUI core logic (design §3.11).
#
# Each test links against the gui_core static library (defined in the parent
# CMakeLists.txt) so it can exercise AlgoBridge, ConfigManager, LayoutManager
# and the display strategies without a live camera or a display. Tests that
# instantiate Qt widgets set QT_QPA_PLATFORM=offscreen (via a custom main() and
# the ENVIRONMENT test property) so they run headless.

find_package(GTest REQUIRED)
include(GoogleTest)

# GCC/Clang-only flags: MSVC rejects them (and would misparse -Wall as /Wall).
if(MSVC)
    set(GUI_TEST_COMPILE_OPTS "")
else()
    set(GUI_TEST_COMPILE_OPTS -Wall -Wextra)
endif()

# --- test_theme_tokens: header-only token contrast check (no gui_core link). ---
add_executable(test_theme_tokens
    test_theme_tokens.cpp
)
target_include_directories(test_theme_tokens PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/..)
target_link_libraries(test_theme_tokens PRIVATE
    Qt6::Core
    GTest::GTest
    GTest::Main
)
target_compile_options(test_theme_tokens PRIVATE ${GUI_TEST_COMPILE_OPTS})
ebplus_gtest_discover_tests(test_theme_tokens)

# --- test_icon_provider: embedded SVGs render through QtSvg. ---
add_executable(test_icon_provider
    test_icon_provider.cpp
    ../resources/icons.qrc
)
target_link_libraries(test_icon_provider PRIVATE
    gui_core
    GTest::GTest
)
target_compile_options(test_icon_provider PRIVATE ${GUI_TEST_COMPILE_OPTS})
ebplus_gtest_discover_tests(test_icon_provider
    PROPERTIES ENVIRONMENT "QT_QPA_PLATFORM=offscreen"
)

# --- test_algo_bridge: registry, idempotent find_or_create, flood guard. ---
add_executable(test_algo_bridge
    test_algo_bridge.cpp
)
target_link_libraries(test_algo_bridge PRIVATE
    gui_core
    GTest::GTest
    GTest::Main
)
target_compile_options(test_algo_bridge PRIVATE ${GUI_TEST_COMPILE_OPTS})
ebplus_gtest_discover_tests(test_algo_bridge)

# --- test_config_manager: algorithm-state serialization round-trip. ---
add_executable(test_config_manager
    test_config_manager.cpp
)
target_link_libraries(test_config_manager PRIVATE
    gui_core
    GTest::GTest
    GTest::Main
)
target_compile_options(test_config_manager PRIVATE ${GUI_TEST_COMPILE_OPTS})
ebplus_gtest_discover_tests(test_config_manager)

# --- test_panel_visibility: Phase 1 capability framework — facility-backed
# hardware panels hide for sources lacking trigger/ESP (needs QApplication). ---
add_executable(test_panel_visibility
    test_panel_visibility.cpp
)
target_link_libraries(test_panel_visibility PRIVATE
    gui_core
    GTest::GTest
)
target_compile_options(test_panel_visibility PRIVATE ${GUI_TEST_COMPILE_OPTS})
ebplus_gtest_discover_tests(test_panel_visibility
    PROPERTIES ENVIRONMENT "QT_QPA_PLATFORM=offscreen"
)

# --- test_aedat4_playback: headless GUI-integration check for AEDAT4 replay
# (real recordings via EBPLUS_TEST_AEDAT4; skipped when unset). ---
add_executable(test_aedat4_playback
    test_aedat4_playback.cpp
)
target_link_libraries(test_aedat4_playback PRIVATE
    gui_core
    GTest::GTest
)
target_compile_options(test_aedat4_playback PRIVATE ${GUI_TEST_COMPILE_OPTS})
ebplus_gtest_discover_tests(test_aedat4_playback
    PROPERTIES ENVIRONMENT "QT_QPA_PLATFORM=offscreen"
)

# --- test_esp_panel_duty: the Anti-Flicker duty-cycle widget stays in the
# facility's percentage domain — the HAL min getter reports the 1/16 register
# granularity as a fraction while get/set values are percentages. ---
add_executable(test_esp_panel_duty
    test_esp_panel_duty.cpp
)
target_link_libraries(test_esp_panel_duty PRIVATE
    gui_core
    GTest::GTest
)
target_compile_options(test_esp_panel_duty PRIVATE ${GUI_TEST_COMPILE_OPTS})
ebplus_gtest_discover_tests(test_esp_panel_duty
    PROPERTIES ENVIRONMENT "QT_QPA_PLATFORM=offscreen"
)

# --- test_aedat4_writer: Phase 4 — writer output read back event-exact by
# our AEDAT4 file source (the reader is the byte-level format reference). ---
add_executable(test_aedat4_writer
    test_aedat4_writer.cpp
)
target_link_libraries(test_aedat4_writer PRIVATE
    gui_core
    GTest::GTest
    GTest::Main
)
target_compile_options(test_aedat4_writer PRIVATE ${GUI_TEST_COMPILE_OPTS})
ebplus_gtest_discover_tests(test_aedat4_writer)

# --- test_layout_manager: layout save/load symmetry (needs QApplication). ---
# Custom main() creates an offscreen QApplication before InitGoogleTest, so we
# link GTest::GTest (not GTest::Main) to avoid a duplicate main().
add_executable(test_layout_manager
    test_layout_manager.cpp
)
target_link_libraries(test_layout_manager PRIVATE
    gui_core
    GTest::GTest
)
target_compile_options(test_layout_manager PRIVATE ${GUI_TEST_COMPILE_OPTS})
ebplus_gtest_discover_tests(test_layout_manager
    PROPERTIES ENVIRONMENT "QT_QPA_PLATFORM=offscreen"
)

# --- test_display_strategy: per-strategy apply correctness (needs QApplication). ---
add_executable(test_display_strategy
    test_display_strategy.cpp
)
target_link_libraries(test_display_strategy PRIVATE
    gui_core
    GTest::GTest
)
target_compile_options(test_display_strategy PRIVATE ${GUI_TEST_COMPILE_OPTS})
ebplus_gtest_discover_tests(test_display_strategy
    PROPERTIES ENVIRONMENT "QT_QPA_PLATFORM=offscreen"
)

# --- test_calibration_tap: event-tap window retention (needs QCoreApplication). ---
add_executable(test_calibration_tap
    test_calibration_tap.cpp
)
target_link_libraries(test_calibration_tap PRIVATE
    gui_core
    GTest::GTest
)
target_compile_options(test_calibration_tap PRIVATE ${GUI_TEST_COMPILE_OPTS})
ebplus_gtest_discover_tests(test_calibration_tap
    PROPERTIES ENVIRONMENT "QT_QPA_PLATFORM=offscreen"
)

# --- test_coverage_hull: wizard coverage-overlay math (no Qt needed). ---
add_executable(test_coverage_hull
    test_coverage_hull.cpp
)
target_include_directories(test_coverage_hull PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/..)
target_link_libraries(test_coverage_hull PRIVATE
    ${OpenCV_LIBS}
    GTest::GTest
    GTest::Main
)
target_compile_options(test_coverage_hull PRIVATE ${GUI_TEST_COMPILE_OPTS})
ebplus_gtest_discover_tests(test_coverage_hull)

# --- test_preproc_routing: preproc params reach algorithm instances AND the
# display pipeline (single panel control, two routes). ---
add_executable(test_preproc_routing
    test_preproc_routing.cpp
)
target_link_libraries(test_preproc_routing PRIVATE
    gui_core
    GTest::GTest
    GTest::Main
)
target_compile_options(test_preproc_routing PRIVATE ${GUI_TEST_COMPILE_OPTS})
ebplus_gtest_discover_tests(test_preproc_routing
    PROPERTIES ENVIRONMENT "QT_QPA_PLATFORM=offscreen"
)

# --- test_stream_conditioner: the shared conditioning pipeline (ROI ->
# value stages -> noise filter -> thin -> undistort -> flips) ---
add_executable(test_stream_conditioner
    test_stream_conditioner.cpp
)
target_link_libraries(test_stream_conditioner PRIVATE
    gui_core
    GTest::GTest
    GTest::Main
)
target_compile_options(test_stream_conditioner PRIVATE ${GUI_TEST_COMPILE_OPTS})
ebplus_gtest_discover_tests(test_stream_conditioner
    PROPERTIES ENVIRONMENT "QT_QPA_PLATFORM=offscreen"
)

# --- test_external_file_sources: AEDAT4 / ALPDATA playback readers + the
# built-in LZ4 frame decoder. Real-recording smoke tests activate via the
# EBPLUS_TEST_AEDAT4 / EBPLUS_TEST_ALPDATA environment variables. ---
add_executable(test_external_file_sources
    test_external_file_sources.cpp
)
target_link_libraries(test_external_file_sources PRIVATE
    gui_core
    GTest::GTest
    GTest::Main
)
target_compile_options(test_external_file_sources PRIVATE ${GUI_TEST_COMPILE_OPTS})
ebplus_gtest_discover_tests(test_external_file_sources
    PROPERTIES ENVIRONMENT "QT_QPA_PLATFORM=offscreen"
)

# --- test_davis_protocol: DAVIS wire decoder + bias table/encoding (the USB
# transport itself requires hardware and is not covered here). ---
add_executable(test_davis_protocol
    test_davis_protocol.cpp
)
target_link_libraries(test_davis_protocol PRIVATE
    gui_core
    GTest::GTest
    GTest::Main
)
target_compile_options(test_davis_protocol PRIVATE ${GUI_TEST_COMPILE_OPTS})
ebplus_gtest_discover_tests(test_davis_protocol
    PROPERTIES ENVIRONMENT "QT_QPA_PLATFORM=offscreen"
)
