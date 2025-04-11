
#################################
# Build
#################################

add_executable(app-fft-bmark
  src/main.c
  src/kiss_fft.c
)

target_include_directories(app-fft-bmark PUBLIC include src src/tools)

target_link_libraries(app-fft-bmark PRIVATE 
  -L${CMAKE_BINARY_DIR}/glossy -Wl,--whole-archive glossy -Wl,--no-whole-archive
)

target_link_libraries(app-fft-bmark PRIVATE libbmark)

target_link_libraries(app-fft-bmark PRIVATE m)
