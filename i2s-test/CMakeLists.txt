
#################################
# Build
#################################

add_executable(i2s-test
  src/main.c
)

target_include_directories(i2s-test PUBLIC include)

target_link_libraries(i2s-test PRIVATE 
  -L${CMAKE_BINARY_DIR}/glossy -Wl,--whole-archive glossy -Wl,--no-whole-archive
)