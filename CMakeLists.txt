cmake_minimum_required(VERSION 3.16)

project(GUI_for_openEB LANGUAGES CXX VERSION 3.1.0)

# MetavisionSDK's installed config files call find_package(Boost) without a
# policy scope; on CMake >= 3.30 that emits a CMP0167 warning on every
# configure. NEW is what every supported toolchain already resolves to
# (vcpkg, Homebrew and Debian all ship BoostConfig.cmake), so setting it
# only removes the warning.
if(POLICY CMP0167)
    cmake_policy(SET CMP0167 NEW)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

# GCC 15+ no longer implicitly includes <cstdint>; add it globally so that
# uint8_t / uint16_t etc. used throughout the OpenEB headers stay available.
# See wiki/compile.md for details.
if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU" AND CMAKE_CXX_COMPILER_VERSION VERSION_GREATER_EQUAL "15")
    add_compile_options("-include;cstdint")
endif()

if(MSVC)
    # Sources carry UTF-8 comments; MSVC assumes the legacy codepage otherwise.
    add_compile_options(/utf-8)
    add_compile_definitions(_USE_MATH_DEFINES NOMINMAX)
endif()

if(APPLE)
    set(CMAKE_MACOSX_RPATH ON)
    set(CMAKE_INSTALL_RPATH "@executable_path/../Frameworks")
endif()

# --- External dependencies -------------------------------------------------
find_package(Qt6 REQUIRED COMPONENTS Widgets OpenGL OpenGLWidgets Svg)
find_package(MetavisionSDK 5.2.0 REQUIRED COMPONENTS base core stream)
# OpenCV — calib3d was split into calib/3d/stereo in OpenCV 5.0; probe the
# version first, then require the correct component names.
find_package(OpenCV REQUIRED)
if(OpenCV_VERSION VERSION_GREATER_EQUAL "5.0")
    # OpenCV 5.0 split calib3d into multiple modules (calib, 3d/geometry,
    # stereo, ptcloud), and moved the chessboard/circle-grid pattern detection
    # functions (findChessboardCorners, drawChessboardCorners, findCirclesGrid,
    # CALIB_CB_*) from calib3d to objdetect. undistortPoints moved to geometry.
    # The exact names vary across 5.x packaging — the "3d" module was renamed
    # to "geometry" before the final 5.0.0 release, and Homebrew's opencv 5.0.0
    # does not expose a "3d" component. Probe which components are actually
    # provided and only require those that exist, so the build works across
    # OpenCV 5.x packaging variants.
    set(_OPENCV_CALIB_COMPONENTS "")
    foreach(_comp calib 3d geometry stereo ptcloud objdetect)
        if(TARGET opencv_${_comp})
            list(APPEND _OPENCV_CALIB_COMPONENTS ${_comp})
        endif()
    endforeach()
else()
    set(_OPENCV_CALIB_COMPONENTS calib3d)
endif()
find_package(OpenCV REQUIRED COMPONENTS core imgproc imgcodecs videoio ${_OPENCV_CALIB_COMPONENTS})

# Qt MOC / UIC / RCC auto-processing
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)

# --- Testing ---------------------------------------------------------------
# enable_testing() must be called BEFORE add_subdirectory(gui) so that the
# gui/tests gtest_discover_tests() calls (design §3.11) register with CTest.
enable_testing()

# GTest discovery launches each test binary while the build is still running.
# Apple Silicon and cold Windows machines can take longer than CMake's
# five-second default to load the bundled OpenEB and Qt dependency graph.
set(EBPLUS_GTEST_DISCOVERY_TIMEOUT 60 CACHE STRING
    "Seconds allowed for GTest test-case discovery")
function(ebplus_gtest_discover_tests target)
    # Discovery runs the test binaries; their OpenEB/Qt runtime graph is not
    # locatable from the build tree before the environment is set up (Windows
    # PATH, Linux LD_LIBRARY_PATH), so defer discovery to the first ctest run.
    gtest_discover_tests(${target}
        DISCOVERY_TIMEOUT ${EBPLUS_GTEST_DISCOVERY_TIMEOUT}
        DISCOVERY_MODE PRE_TEST
        ${ARGN}
    )
endfunction()

add_subdirectory(algo)
add_subdirectory(gui)

option(GUI_FOR_OPENEB_BUILD_TESTS "Build algorithm unit tests" ON)
if(GUI_FOR_OPENEB_BUILD_TESTS)
    add_subdirectory(algo/tests)
endif()

# Local performance benchmark harness (never built by default, not run in
# CI). Enable with -DEBPLUS_BUILD_BENCHMARKS=ON. See benchmarks/CMakeLists.txt.
option(EBPLUS_BUILD_BENCHMARKS "Build the local performance benchmark harness" OFF)
if(EBPLUS_BUILD_BENCHMARKS)
    add_subdirectory(benchmarks)
endif()
