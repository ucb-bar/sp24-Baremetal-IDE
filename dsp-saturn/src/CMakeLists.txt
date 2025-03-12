
#################################
# Build
#################################

add_executable(dsp-saturn
  fft2_main.c
)

target_link_libraries(dsp-saturn PRIVATE 
  -L${CMAKE_BINARY_DIR}/glossy -Wl,--whole-archive glossy -Wl,--no-whole-archive
)

