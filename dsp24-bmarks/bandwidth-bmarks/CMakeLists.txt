
#################################
# Build
#################################

add_executable(membw-bmark
  src/main.c
  src/mtwister.c
)

target_include_directories(membw-bmark PUBLIC include)

target_link_libraries(membw-bmark PRIVATE 
  -L${CMAKE_BINARY_DIR}/glossy -Wl,--whole-archive glossy -Wl,--no-whole-archive
)

target_link_libraries(membw-bmark PRIVATE libbmark)
target_link_libraries(membw-bmark PRIVATE chip-config)
target_link_libraries(membw-bmark PRIVATE rocketcore)