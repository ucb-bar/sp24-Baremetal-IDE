
#################################
# Build
#################################

add_executable(app_fft_bmark
  src/main.c
)

target_include_directories(app_fft_bmark PUBLIC include)

target_link_libraries(app_fft_bmarks PRIVATE 
  -L${CMAKE_BINARY_DIR}/glossy -Wl,--whole-archive glossy -Wl,--no-whole-archive
)

target_link_libraries(app_fft_bmark PRIVATE libbmark)