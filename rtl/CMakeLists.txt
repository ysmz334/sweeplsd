# Host tool for the RTL verification loop: dumps golden vectors consumed by
# the Icarus Verilog testbenches in tb/. Reuses the phase-1 HLS C model (with
# its compat shim) as the source of truth for events and segment records.

add_executable(sweeplsd_dump_vectors
  tools/dump_vectors.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/../hls/src/frontend.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/../hls/src/backend.cpp
)
target_include_directories(sweeplsd_dump_vectors PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/../hls/src
  ${CMAKE_CURRENT_SOURCE_DIR}/../hls/compat
  ${CMAKE_CURRENT_SOURCE_DIR}/../src
)
target_link_libraries(sweeplsd_dump_vectors PRIVATE sweeplsd sweeplsd_io)
if(NOT MSVC)
  target_compile_options(sweeplsd_dump_vectors PRIVATE -Wno-unknown-pragmas)
endif()

# Software twin of the Atlys demo scene (photo-comparison tool).
add_executable(sweeplsd_demo_scene tools/demo_scene.cpp)
target_link_libraries(sweeplsd_demo_scene PRIVATE sweeplsd sweeplsd_io)
