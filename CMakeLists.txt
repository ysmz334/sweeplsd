cmake_minimum_required(VERSION 3.15)
project(sweeplsd VERSION 3.0.3 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
include(GNUInstallDirs)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

# SIMD instruction-set target for the whole build. The kernels contain no
# intrinsics; they are written to auto-vectorize, so this only picks the ISA
# the compiler may use. Options: avx2 | sse2 | native | none.
# The avx2/sse2 values are x86-only and are ignored (with a note) on other
# architectures — e.g. ARM64, where NEON is baseline and needs no flag.
set(SWEEPLSD_SIMD "avx2" CACHE STRING "SIMD ISA target: avx2 | sse2 | native | none")
if(CMAKE_SYSTEM_PROCESSOR MATCHES "(x86_64|AMD64|amd64|i.86)")
  set(SWEEPLSD_ARCH_X86 TRUE)
else()
  set(SWEEPLSD_ARCH_X86 FALSE)
endif()
if(MSVC)
  # Sources are UTF-8 (comments contain §, ×, …); without /utf-8 MSVC assumes
  # the local code page and warns C4819 (or miscompiles string literals).
  add_compile_options(/utf-8)
  if(SWEEPLSD_SIMD STREQUAL "avx2" AND SWEEPLSD_ARCH_X86)
    add_compile_options(/arch:AVX2)
  endif()
else()
  if(SWEEPLSD_SIMD STREQUAL "native")
    add_compile_options(-march=native)
  elseif(SWEEPLSD_ARCH_X86)
    if(SWEEPLSD_SIMD STREQUAL "avx2")
      add_compile_options(-mavx2 -mfma)
    elseif(SWEEPLSD_SIMD STREQUAL "sse2")
      add_compile_options(-msse2)
    endif()
  elseif(NOT SWEEPLSD_SIMD STREQUAL "none")
    message(STATUS "SweepLSD: SWEEPLSD_SIMD=${SWEEPLSD_SIMD} is x86-only; "
                   "no ISA flag added for ${CMAKE_SYSTEM_PROCESSOR} (NEON etc. is baseline)")
  endif()
endif()
message(STATUS "SweepLSD SIMD target: ${SWEEPLSD_SIMD} (processor: ${CMAKE_SYSTEM_PROCESSOR})")

# On MinGW, statically link the GCC/libstdc++/winpthread runtime so the built
# executables run standalone — double-clicked or from a plain shell — without
# needing mingw64\bin (libstdc++-6.dll, libgcc_s_*.dll, libwinpthread-1.dll) on
# PATH. Behaviour and performance are unchanged.
if(MINGW)
  add_link_options(-static -static-libgcc -static-libstdc++)
endif()

# Core SweepLSD library: the pipeline stages plus the two drivers (multi-pass
# and streaming one-pass). No third-party dependency — C++ standard library only.
add_library(sweeplsd
  src/gaussian.cpp
  src/gradient.cpp
  src/edge.cpp
  src/feature.cpp
  src/labeling.cpp
  src/sweeplsd.cpp
  src/sweeplsd_onepass.cpp
)
add_library(sweeplsd::sweeplsd ALIAS sweeplsd)
target_include_directories(sweeplsd PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)

# Image I/O (stb). stb is a private compile dependency of io.cpp only — the
# public header (sweeplsd/io.hpp) does not include it, so consumers need nothing.
add_library(sweeplsd_io src/io.cpp)
add_library(sweeplsd::io ALIAS sweeplsd_io)
set_target_properties(sweeplsd_io PROPERTIES EXPORT_NAME io)
target_include_directories(sweeplsd_io PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)
target_include_directories(sweeplsd_io PRIVATE third_party/stb)
target_link_libraries(sweeplsd_io PUBLIC sweeplsd)

# OpenCV adapter (header-only, optional): cv::Mat in, cv::Vec4f out, drop-in
# for OpenCV's LSD. INTERFACE target — the core build needs no OpenCV; the
# consumer links OpenCV themselves.
add_library(sweeplsd_opencv INTERFACE)
add_library(sweeplsd::opencv ALIAS sweeplsd_opencv)
set_target_properties(sweeplsd_opencv PROPERTIES EXPORT_NAME opencv)
target_include_directories(sweeplsd_opencv INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)
target_link_libraries(sweeplsd_opencv INTERFACE sweeplsd)

# Command-line tools.
add_executable(sweeplsd_cli apps/sweeplsd_cli.cpp)
target_link_libraries(sweeplsd_cli PRIVATE sweeplsd sweeplsd_io)

add_executable(sweeplsd_profile apps/sweeplsd_profile.cpp)
target_link_libraries(sweeplsd_profile PRIVATE sweeplsd sweeplsd_io)

# Per-stage hotspot profiler for the streaming one-pass driver (mirrors
# sweeplsd_onepass.cpp with rdtsc counters); needs the internal kernels/stage
# headers in src/. rdtsc / <x86intrin.h> are x86-only, so this investigation
# tool is built on x86 targets only (skipped on ARM64 etc.).
if(SWEEPLSD_ARCH_X86)
  add_executable(sweeplsd_hotspots apps/sweeplsd_hotspots.cpp)
  target_include_directories(sweeplsd_hotspots PRIVATE src)
  target_link_libraries(sweeplsd_hotspots PRIVATE sweeplsd sweeplsd_io)
endif()

# Stage-dump tool (regenerates the figures used by docs/); needs the internal
# stage headers in src/.
add_executable(sweeplsd_dump_stages apps/sweeplsd_dump_stages.cpp)
target_include_directories(sweeplsd_dump_stages PRIVATE src)
target_link_libraries(sweeplsd_dump_stages PRIVATE sweeplsd sweeplsd_io)

# Examples using ONLY the public library.
option(SWEEPLSD_BUILD_EXAMPLES "Build example applications" ON)
if(SWEEPLSD_BUILD_EXAMPLES)
  add_executable(sweeplsd_vp examples/vanishing_points.cpp)
  target_link_libraries(sweeplsd_vp PRIVATE sweeplsd::sweeplsd sweeplsd::io)

  # Calibrated Manhattan-frame estimation with the estimator configuration
  # that measured best for SweepLSD (unit-vote / vertical prior / strong search).
  add_executable(sweeplsd_manhattan examples/manhattan_frame.cpp)
  target_link_libraries(sweeplsd_manhattan PRIVATE sweeplsd::sweeplsd sweeplsd::io)
endif()

# Benchmarks against LSD (AGPL) and EDLines/ED_Lib. Third-party detector code
# is NOT vendored in this repository — see bench/README.md; enabling this
# option fetches it at configure time.
option(SWEEPLSD_BUILD_BENCH "Build benchmark harness (fetches AGPL LSD + MIT ED_Lib)" OFF)
if(SWEEPLSD_BUILD_BENCH)
  add_subdirectory(bench)
endif()

# HLS (FPGA) core + its host-compiled parity testbench — see hls/DESIGN.md.
option(SWEEPLSD_BUILD_HLS "Build the HLS core's host testbench" OFF)
# RTL (phase 2) golden-vector tool — see rtl/DESIGN.md.
option(SWEEPLSD_BUILD_RTL "Build the RTL verification host tools" OFF)

# Tests.
enable_testing()
if(SWEEPLSD_BUILD_HLS)
  add_subdirectory(hls)
endif()
if(SWEEPLSD_BUILD_RTL)
  add_subdirectory(rtl)
endif()
add_executable(sweeplsd_tests tests/test_sweeplsd.cpp)
target_include_directories(sweeplsd_tests PRIVATE src)
target_link_libraries(sweeplsd_tests PRIVATE sweeplsd)
add_test(NAME sweeplsd_tests COMMAND sweeplsd_tests)

# ---- Install / package config: find_package(sweeplsd CONFIG) +
#      target_link_libraries(sweeplsd::sweeplsd).
include(CMakePackageConfigHelpers)
install(TARGETS sweeplsd sweeplsd_io sweeplsd_opencv EXPORT sweeplsdTargets
        ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
        RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
install(DIRECTORY include/sweeplsd DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
install(EXPORT sweeplsdTargets
        NAMESPACE sweeplsd::
        FILE sweeplsdTargets.cmake
        DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/sweeplsd)
write_basic_package_version_file(
        "${CMAKE_CURRENT_BINARY_DIR}/sweeplsdConfigVersion.cmake"
        VERSION ${PROJECT_VERSION} COMPATIBILITY SameMajorVersion)
configure_package_config_file(
        "${CMAKE_CURRENT_SOURCE_DIR}/cmake/sweeplsdConfig.cmake.in"
        "${CMAKE_CURRENT_BINARY_DIR}/sweeplsdConfig.cmake"
        INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/sweeplsd)
install(FILES
        "${CMAKE_CURRENT_BINARY_DIR}/sweeplsdConfig.cmake"
        "${CMAKE_CURRENT_BINARY_DIR}/sweeplsdConfigVersion.cmake"
        DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/sweeplsd)
