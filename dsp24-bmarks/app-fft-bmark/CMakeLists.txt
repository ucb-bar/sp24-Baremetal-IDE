
#################################
# Build
#################################

add_executable(app-fft-bmark
  src/main.c
)

target_include_directories(app-fft-bmark PUBLIC include)

target_link_libraries(app-fft-bmark PRIVATE 
  -L${CMAKE_BINARY_DIR}/glossy -Wl,--whole-archive glossy -Wl,--no-whole-archive
)

target_link_libraries(app-fft-bmark PRIVATE libbmark)