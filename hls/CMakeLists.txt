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

# FIFO-drop study tooling (host-only; not synthesised). density_probe = Step 0
# (per-image event-density tabulation from the front-end event stream).
add_executable(sweeplsd_density
  src/frontend.cpp
  src/backend.cpp
  tools/density_probe.cpp
)
target_include_directories(sweeplsd_density PRIVATE
  src
  compat
  host
  ${CMAKE_CURRENT_SOURCE_DIR}/../src
)
target_link_libraries(sweeplsd_density PRIVATE sweeplsd sweeplsd_io)
if(NOT MSVC)
  target_compile_options(sweeplsd_density PRIVATE -Wno-unknown-pragmas)
endif()

# Step 1: producer -> FIFO -> back-end co-sim; per-image dropped events + lost
# segments under the live event-FIFO drop policy.
add_executable(sweeplsd_dropsim
  src/frontend.cpp
  src/backend.cpp
  tools/fifo_dropsim.cpp
)
target_include_directories(sweeplsd_dropsim PRIVATE
  src
  compat
  host
  ${CMAKE_CURRENT_SOURCE_DIR}/../src
)
target_link_libraries(sweeplsd_dropsim PRIVATE sweeplsd sweeplsd_io)
if(NOT MSVC)
  target_compile_options(sweeplsd_dropsim PRIVATE -Wno-unknown-pragmas)
endif()

# Judge datapath sizing probe: empirical max moment inputs/products the judge
# sees over a corpus (to narrow the 128-bit judge datapath).
add_executable(sweeplsd_moment
  src/frontend.cpp
  src/backend.cpp
  tools/moment_probe.cpp
)
target_include_directories(sweeplsd_moment PRIVATE
  src
  compat
  host
  ${CMAKE_CURRENT_SOURCE_DIR}/../src
)
target_link_libraries(sweeplsd_moment PRIVATE sweeplsd sweeplsd_io)
if(NOT MSVC)
  target_compile_options(sweeplsd_moment PRIVATE -Wno-unknown-pragmas)
endif()

# (B) vertical-stripe parallelism design probe: single-engine golden vs the
# independent two-engine split; boundary-crossing fidelity cost + load balance.
add_executable(sweeplsd_stripe
  src/frontend.cpp
  src/backend.cpp
  tools/stripe_probe.cpp
)
target_include_directories(sweeplsd_stripe PRIVATE
  src
  compat
  host
  ${CMAKE_CURRENT_SOURCE_DIR}/../src
)
target_link_libraries(sweeplsd_stripe PRIVATE sweeplsd sweeplsd_io)
if(NOT MSVC)
  target_compile_options(sweeplsd_stripe PRIVATE -Wno-unknown-pragmas)
endif()

# Per-image PNG visualiser: buffer-occupancy + dropped-pixel map, and
# survived/lost segment overlay.
add_executable(sweeplsd_dropviz
  src/frontend.cpp
  src/backend.cpp
  tools/fifo_dropviz.cpp
)
target_include_directories(sweeplsd_dropviz PRIVATE
  src
  compat
  host
  ${CMAKE_CURRENT_SOURCE_DIR}/../src
)
target_link_libraries(sweeplsd_dropviz PRIVATE sweeplsd sweeplsd_io)
if(NOT MSVC)
  target_compile_options(sweeplsd_dropviz PRIVATE -Wno-unknown-pragmas)
endif()
