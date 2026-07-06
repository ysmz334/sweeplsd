# Benchmark harness. Fetches the baseline detectors at configure time —
# nothing here is vendored into the repository:
#   * LSD (R. Grompone von Gioi et al.)  — AGPL-3.0. Linked ONLY into the
#     benchmark executables below; the sweeplsd library itself stays MIT.
#   * ED_Lib (C. Akinlar & C. Topal)     — MIT. Needs OpenCV, so the EDLines
#     runner is built only when OpenCV is found.
include(FetchContent)

message(STATUS "SweepLSD bench: fetching LSD (AGPL-3.0; benchmark executables only)")
FetchContent_Declare(lsd_upstream
  GIT_REPOSITORY https://github.com/centreborelli/lsd.git
  GIT_TAG 1d21182f5837bd8783d9fd81f5f5842a5bbcdfb8)
FetchContent_MakeAvailable(lsd_upstream)

add_library(lsd_baseline ${lsd_upstream_SOURCE_DIR}/lsd.c)
# Compile the C source as C++ so it links without extern "C" shims.
set_source_files_properties(${lsd_upstream_SOURCE_DIR}/lsd.c PROPERTIES LANGUAGE CXX)
target_include_directories(lsd_baseline PUBLIC ${lsd_upstream_SOURCE_DIR})
if(MSVC)
  target_compile_options(lsd_baseline PRIVATE /w)
else()
  target_compile_options(lsd_baseline PRIVATE -w)
endif()

# Common pieces: the EDLines-style fallback (our own, dependency-free; it
# UNDERESTIMATES genuine EDLines and is kept only so the tools run without
# OpenCV — prefer feeding genuine ED_Lib results via --edreal-dir).
add_library(bench_common OBJECT edlines.cpp)
target_link_libraries(bench_common PUBLIC sweeplsd sweeplsd_io)

foreach(tool evaluate yud_eval time_methods gen_isotropy)
  add_executable(sweeplsd_${tool} ${tool}.cpp $<TARGET_OBJECTS:bench_common>)
  target_link_libraries(sweeplsd_${tool} PRIVATE sweeplsd sweeplsd_io lsd_baseline)
  target_include_directories(sweeplsd_${tool} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
endforeach()

# Genuine EDLines runner (writes per-image "count median_ms" + segment lines
# that the tools ingest via --edreal-dir). Requires OpenCV.
find_package(OpenCV QUIET)
if(OpenCV_FOUND)
  message(STATUS "SweepLSD bench: OpenCV ${OpenCV_VERSION} found — building genuine EDLines runner")
  FetchContent_Declare(edlib_upstream
    GIT_REPOSITORY https://github.com/CihanTopal/ED_Lib.git
    GIT_TAG 69b8d081bd6d28192d816ec0ed02aff9186d73c1)
  FetchContent_MakeAvailable(edlib_upstream)
  add_executable(sweeplsd_edlines_runner edlines_runner.cpp
    ${edlib_upstream_SOURCE_DIR}/ED.cpp
    ${edlib_upstream_SOURCE_DIR}/EDColor.cpp
    ${edlib_upstream_SOURCE_DIR}/EDLines.cpp
    ${edlib_upstream_SOURCE_DIR}/NFA.cpp)
  target_include_directories(sweeplsd_edlines_runner PRIVATE
    ${edlib_upstream_SOURCE_DIR} ${OpenCV_INCLUDE_DIRS})
  target_link_libraries(sweeplsd_edlines_runner PRIVATE ${OpenCV_LIBS})
  if(NOT MSVC)
    target_compile_options(sweeplsd_edlines_runner PRIVATE -w)
  endif()
else()
  message(STATUS "SweepLSD bench: OpenCV not found — genuine EDLines runner skipped "
                 "(tools still accept precomputed results via --edreal-dir)")
endif()
