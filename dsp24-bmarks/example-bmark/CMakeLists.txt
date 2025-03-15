
#################################
# Build
#################################

add_executable(example-bmark
  src/main.c
)

target_include_directories(example-bmark PUBLIC include)

target_link_libraries(example-bmark PRIVATE 
  -L${CMAKE_BINARY_DIR}/glossy -Wl,--whole-archive glossy -Wl,--no-whole-archive
)

target_link_libraries(example-bmark PRIVATE libbmark)