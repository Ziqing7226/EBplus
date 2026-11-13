# algo/ — self-developed event processing algorithms (C++17, no Qt dependency).
# Linked by gui/ via the gui_algo target.

add_library(gui_algo STATIC
    common/frame_generator.cpp
    calibration/intrinsic.cpp
    calibration/rig_calibration.cpp
)

target_include_directories(gui_algo PUBLIC
    ${PROJECT_SOURCE_DIR}
)

target_link_libraries(gui_algo PUBLIC
    MetavisionSDK::base
    MetavisionSDK::core
    MetavisionSDK::stream
    ${OpenCV_LIBS}
    Ceres::ceres
)

# Optional: ONNX Runtime for E2VID neural-network inference.
# When found, defines GUI_ALGO_HAS_ONNXRUNTIME and links against it.
# Detection: look for the header and shared library in standard paths
# and in the bundled third_party/onnxruntime (prebuilt CPU package).
set(_ONNXRUNTIME_INCLUDE_HINTS
    ${CMAKE_SOURCE_DIR}/third_party/onnxruntime/include
    /usr/include /usr/local/include /opt/onnxruntime/include
)
set(_ONNXRUNTIME_LIB_HINTS
    ${CMAKE_SOURCE_DIR}/third_party/onnxruntime/lib
    /usr/lib /usr/local/lib /opt/onnxruntime/lib
)
find_path(ONNXRUNTIME_INCLUDE_DIR
    NAMES onnxruntime_cxx_api.h
    PATHS ${_ONNXRUNTIME_INCLUDE_HINTS}
)
find_library(ONNXRUNTIME_LIBRARY
    NAMES onnxruntime
    PATHS ${_ONNXRUNTIME_LIB_HINTS}
)
if(ONNXRUNTIME_INCLUDE_DIR AND ONNXRUNTIME_LIBRARY)
    target_compile_definitions(gui_algo PUBLIC GUI_ALGO_HAS_ONNXRUNTIME)
    target_link_libraries(gui_algo PUBLIC ${ONNXRUNTIME_LIBRARY})
    target_include_directories(gui_algo PUBLIC ${ONNXRUNTIME_INCLUDE_DIR})
    # RPATH so the runtime loader finds libonnxruntime.so at execution time
    # without requiring LD_LIBRARY_PATH or ldconfig.
    set_target_properties(gui_algo PROPERTIES
        BUILD_RPATH "${_ONNXRUNTIME_LIB_HINTS}"
        INSTALL_RPATH "$ORIGIN/../lib")
    message(STATUS "ONNX Runtime found — E2VID neural-network inference enabled.")
    message(STATUS "   include: ${ONNXRUNTIME_INCLUDE_DIR}")
    message(STATUS "   library: ${ONNXRUNTIME_LIBRARY}")
else()
    message(STATUS "ONNX Runtime not found — E2VID will use heuristic fallback.")
endif()

# Optional: OpenVINO for E2VID inference on Intel GPUs (iGPU/dGPU), §4.4.2-GPU.
# Consumes the SAME .onnx models as the ONNX Runtime path; when present and an
# Intel GPU is enumerated at runtime, Auto/GPU device policies route the E2VID
# neural inference through the GPU plugin. Detection mirrors ONNX Runtime:
# bundled third_party/openvino/runtime (release archive layout:
# runtime/include + runtime/lib/intel64) or a system install.
set(_OPENVINO_PATH_HINTS
    ${CMAKE_SOURCE_DIR}/third_party/openvino/runtime
    /usr /usr/local /opt/openvino
)
find_path(OPENVINO_INCLUDE_DIR
    NAMES openvino/openvino.hpp
    PATHS ${_OPENVINO_PATH_HINTS}
    PATH_SUFFIXES include
)
find_library(OPENVINO_LIBRARY
    NAMES openvino
    PATHS ${_OPENVINO_PATH_HINTS}
    PATH_SUFFIXES lib/intel64 lib
)
if(OPENVINO_INCLUDE_DIR AND OPENVINO_LIBRARY)
    target_compile_definitions(gui_algo PUBLIC GUI_ALGO_HAS_OPENVINO)
    target_link_libraries(gui_algo PUBLIC ${OPENVINO_LIBRARY})
    # SYSTEM: OpenVINO headers emit -Wunused-parameter on GCC 15 — silence
    # them (project builds with -Werror; third-party headers are not ours).
    target_include_directories(gui_algo SYSTEM PUBLIC ${OPENVINO_INCLUDE_DIR})
    # RPATH: the OpenVINO plugins/frontends are discovered relative to
    # libopenvino.so, so one rpath entry for its directory is sufficient.
    get_filename_component(OPENVINO_LIB_DIR "${OPENVINO_LIBRARY}" DIRECTORY)
    set_property(TARGET gui_algo APPEND PROPERTY BUILD_RPATH "${OPENVINO_LIB_DIR}")
    message(STATUS "OpenVINO found — E2VID Intel GPU inference enabled.")
    message(STATUS "   include: ${OPENVINO_INCLUDE_DIR}")
    message(STATUS "   library: ${OPENVINO_LIBRARY}")
else()
    message(STATUS "OpenVINO not found — E2VID runs on ONNX Runtime CPU only.")
endif()


