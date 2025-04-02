
#################################
# Build
#################################

add_executable(conv-bmark
  src/main.c
  src/float16.c
)

target_include_directories(conv-bmark PUBLIC include)

target_link_libraries(conv-bmark PRIVATE 
  -L${CMAKE_BINARY_DIR}/glossy -Wl,--whole-archive glossy -Wl,--no-whole-archive
)

target_link_libraries(conv-bmark PRIVATE libbmark)