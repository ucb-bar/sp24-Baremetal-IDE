add_executable(shmoo_bench__1100
  dsp_conv_bench/src/main.c
)
target_include_directories(shmoo_bench__1100 PUBLIC dsp_conv_bench/include)

target_link_libraries(shmoo_bench__1100 PRIVATE 
  -L${CMAKE_BINARY_DIR}/glossy -Wl,--whole-archive glossy -Wl,--no-whole-archive)

