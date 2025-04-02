
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

add_executable(membw-bmark-uc
  src/membw-bmark-uc.c
  src/mtwister.c
)

target_include_directories(membw-bmark-uc PUBLIC include)

target_link_libraries(membw-bmark-uc PRIVATE 
  -L${CMAKE_BINARY_DIR}/glossy -Wl,--whole-archive glossy -Wl,--no-whole-archive
)

target_link_libraries(membw-bmark-uc PRIVATE libbmark)
target_link_libraries(membw-bmark-uc PRIVATE chip-config)
target_link_libraries(membw-bmark-uc PRIVATE rocketcore)