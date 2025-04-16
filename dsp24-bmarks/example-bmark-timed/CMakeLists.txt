
#################################
# Build
#################################

add_executable(example-bmark-timed
  src/main.c
)

target_include_directories(example-bmark-timed PUBLIC include)

target_link_libraries(example-bmark-timed PRIVATE 
  -L${CMAKE_BINARY_DIR}/glossy -Wl,--whole-archive glossy -Wl,--no-whole-archive
)

target_link_libraries(example-bmark-timed PRIVATE libbmark)