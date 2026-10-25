# gui/ — Qt 6 GUI application for openEB.

# Library sources (everything except main.cpp). Built once as a static library
# so the unit tests in tests/ can link the same code without recompiling it.
set(GUI_LIB_SOURCES
    main_window.cpp
    app/aedat4_file_source.cpp
    app/alpdata_file_source.cpp
    app/camera_controller.cpp
    app/external_file_source.cpp
    app/file_converter.cpp
    app/file_frame_generator.cpp
    app/frame_mode_renderer.cpp
    app/frame_pipeline.cpp
    app/icon_provider.cpp
    app/lz4_frame_decoder.cpp
    app/statistics_controller.cpp
    app/stream_conditioner.cpp
    app/theme_controller.cpp
    algo_bridge/algo_bridge.cpp
    algo_bridge/backends/cv_backends.cpp
    algo_bridge/backends/cv_vector_backends.cpp
    algo_bridge/backends/analytics_backends.cpp
    algo_bridge/backends/analytics_extra_backends.cpp
    algo_bridge/backends/display_backends.cpp
    algo_bridge/backends/filter_backends.cpp
    algo_bridge/backends/backend_factory.cpp
    algo_bridge/backends/backend_common.cpp
    algo_bridge/filter_chain.cpp
    config/config_manager.cpp
    display/event_display_widget.cpp
    display/display_strategy.cpp
    exporter/exporter_controller.cpp
    exporter/export_dialog.cpp
    panels/abstract_panel.cpp
    panels/algorithms_panel.cpp
    panels/biases_panel.cpp
    panels/devices_panel.cpp
    panels/display_panel.cpp
    panels/esp_panel.cpp
    panels/file_tools_panel.cpp
    panels/file_op_dialog.cpp
    panels/file_info_dialog.cpp
    recorder/record_dialog.cpp
    panels/information_panel.cpp
    panels/preprocessing_panel.cpp
    panels/roi_panel.cpp
    panels/settings_panel.cpp
    panels/statistics_panel.cpp
    panels/trigger_panel.cpp
    recorder/playback_controller.cpp
    recorder/playback_controls.cpp
    recorder/recorder_controller.cpp
    recorder/aedat4_writer.cpp
)

# Phase 9 / Phase 4 — calibration wizard + tools (built when sources are present).
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/calibration/calibration_wizard.cpp")
    list(APPEND GUI_LIB_SOURCES
        calibration/calibration_wizard.cpp
        calibration/blinking_chessboard_display.cpp
        calibration/calibration_event_tap.cpp
        calibration/calibration_worker.cpp
        calibration/focus_dialog.cpp
    )
endif()

# Phase 10 — layout manager.
list(APPEND GUI_LIB_SOURCES
    widgets/custom_title_bar.cpp
    widgets/activity_bar.cpp
    config/layout_manager.cpp
)

# Phase 11 — jAER-inspired display / widgets (design §1.6).
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/display/space_time_display.cpp")
    list(APPEND GUI_LIB_SOURCES
        widgets/algo_window.cpp
        widgets/aps_window.cpp
        widgets/imu_window.cpp
        widgets/unified_roi_dialog.cpp
        display/frame_annotator.cpp
        display/space_time_display.cpp
    )
endif()

# Static library shared by the application and the unit tests.
add_library(gui_core STATIC ${GUI_LIB_SOURCES})
target_include_directories(gui_core PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(gui_core PUBLIC
    Qt6::Widgets
    Qt6::OpenGL
    Qt6::OpenGLWidgets
    Qt6::Svg
    gui_algo
)

# Phase 2 panels include HAL facility headers directly
# (metavision/hal/facilities/i_*.h). The HAL target is exported by the
# MetavisionSDK CMake config as MetavisionSDK::hal; link it explicitly so
# the include paths propagate even if gui_algo only links base/core/stream.
if(TARGET MetavisionSDK::hal)
    target_link_libraries(gui_core PUBLIC MetavisionSDK::hal)
endif()

# Live DAVIS (inivation) camera support — optional, based on libusb-1.0 only
# (protocol ported from the dv-processing reference). Without libusb the
# parser/bias sources still build (used by unit tests) but live connection
# and the device layer are compiled out (GUI_HAVE_DAVIS=0).
target_sources(gui_core PRIVATE
    davis/davis_parser.cpp
    davis/davis_biases.cpp
    davis/dvxplorer_parser.cpp
)
find_path(LIBUSB_INCLUDE_DIR NAMES libusb.h PATH_SUFFIXES libusb-1.0)
find_library(LIBUSB_LIBRARY NAMES usb-1.0)
if(LIBUSB_INCLUDE_DIR AND LIBUSB_LIBRARY)
    target_compile_definitions(gui_core PUBLIC GUI_HAVE_DAVIS=1)
    target_include_directories(gui_core PUBLIC ${LIBUSB_INCLUDE_DIR})
    target_link_libraries(gui_core PUBLIC ${LIBUSB_LIBRARY})
    target_sources(gui_core PRIVATE davis/davis_device.cpp davis/dvxplorer_device.cpp)
    message(STATUS "inivation live camera support (DAVIS/DVXplorer): enabled")
else()
    target_compile_definitions(gui_core PUBLIC GUI_HAVE_DAVIS=0)
    message(STATUS "inivation live camera support: disabled (libusb-1.0 development files not found)")
endif()

# Main executable — entry point only; all logic lives in gui_core.
if(APPLE)
    add_executable(gui_for_openeb MACOSX_BUNDLE main.cpp)
    set_target_properties(gui_for_openeb PROPERTIES
        OUTPUT_NAME "EBplus"
        MACOSX_BUNDLE_BUNDLE_NAME "EBplus"
        MACOSX_BUNDLE_GUI_IDENTIFIER "org.ebplus.gui-for-openeb"
        MACOSX_BUNDLE_SHORT_VERSION_STRING "${PROJECT_VERSION}"
        MACOSX_BUNDLE_BUNDLE_VERSION "${PROJECT_VERSION}"
        MACOSX_BUNDLE_COPYRIGHT "Copyright EBplus contributors"
        INSTALL_RPATH "@executable_path/../Frameworks"
    )
else()
    add_executable(gui_for_openeb main.cpp)
endif()

# Qt resources — theme QSS template + Lucide-style SVG icons (design §3.1/§3.2).
# AUTORCC compiles these into the binary so the code can load ":/theme/..." and
# ":/icons/..." at runtime.
target_sources(gui_for_openeb PRIVATE
    resources/theme.qrc
    resources/icons.qrc
)

target_link_libraries(gui_for_openeb PRIVATE gui_core)

if(APPLE)
    install(TARGETS gui_for_openeb BUNDLE DESTINATION . RUNTIME DESTINATION bin)
else()
    install(TARGETS gui_for_openeb RUNTIME DESTINATION bin)
endif()

# --- Unit tests (design §3.11) ---------------------------------------------
option(GUI_BUILD_TESTS "Build GUI unit tests" ON)
if(GUI_BUILD_TESTS)
    add_subdirectory(tests)
endif()
