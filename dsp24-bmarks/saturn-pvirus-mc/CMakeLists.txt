
#################################
# Build
#################################

add_executable(saturn-pvirus-mc
  src/main.c
)

target_include_directories(saturn-pvirus-mc PUBLIC include)

target_link_libraries(saturn-pvirus-mc PRIVATE 
  -L${CMAKE_BINARY_DIR}/glossy -Wl,--whole-archive glossy -Wl,--no-whole-archive
)

target_link_libraries(saturn-pvirus-mc PRIVATE libbmark)