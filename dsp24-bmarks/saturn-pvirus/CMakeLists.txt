
#################################
# Build
#################################

add_executable(saturn-pvirus
  src/main.c
)

target_include_directories(saturn-pvirus PUBLIC include)

target_link_libraries(saturn-pvirus PRIVATE 
  -L${CMAKE_BINARY_DIR}/glossy -Wl,--whole-archive glossy -Wl,--no-whole-archive
)

target_link_libraries(saturn-pvirus PRIVATE libbmark)