
#################################
# Build
#################################

add_executable(conv-pvirus
  src/main.c
)

target_include_directories(conv-pvirus PUBLIC include)

target_link_libraries(conv-pvirus PRIVATE 
  -L${CMAKE_BINARY_DIR}/glossy -Wl,--whole-archive glossy -Wl,--no-whole-archive
)

target_link_libraries(conv-pvirus PRIVATE libbmark)