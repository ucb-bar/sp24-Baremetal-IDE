#################################
# Build
#################################

add_executable(saturn-bmarks-shmoo
  src/imatmul.c
  src/main.c
  src/massive_matmul_data.S
)

target_include_directories(saturn-bmarks-shmoo PUBLIC include)

target_link_libraries(saturn-bmarks-shmoo PRIVATE 
  -L${CMAKE_BINARY_DIR}/glossy -Wl,--whole-archive glossy -Wl,--no-whole-archive
)

target_link_libraries(saturn-bmarks-shmoo PRIVATE libbmark)
