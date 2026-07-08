# Host build of the HLS core + parity testbench (the tool-free inner loop).
# The same sources are consumed by Vitis HLS via scripts/run_hls.tcl; here they
# compile with the compat shim (compat/) against any C++17 compiler so the
# bit-exactness check runs in seconds, no FPGA tools required.

add_executable(sweeplsd_hls_tb
  src/frontend.cpp
  src/backend.cpp
  tb/tb_frontend.cpp
)
target_include_directories(sweeplsd_hls_tb PRIVATE
  src
  compat
  host
  ${CMAKE_CURRENT_SOURCE_DIR}/../src   # golden-model internals (stages.hpp)
)
target_link_libraries(sweeplsd_hls_tb PRIVATE sweeplsd sweeplsd_io)
if(NOT MSVC)
  # #pragma HLS lines are meaningful only to Vitis; keep host builds quiet.
  target_compile_options(sweeplsd_hls_tb PRIVATE -Wno-unknown-pragmas)
endif()

add_test(NAME sweeplsd_hls_tb COMMAND sweeplsd_hls_tb)
