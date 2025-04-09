
#################################
# Build
#################################

add_executable(audio-bmark
  src/main.c
)

target_include_directories(audio-bmark PUBLIC include)

target_link_libraries(audio-bmark PRIVATE 
  -L${CMAKE_BINARY_DIR}/glossy -Wl,--whole-archive glossy -Wl,--no-whole-archive
)

target_link_libraries(audio-bmark PRIVATE libbmark)
target_link_libraries(audio-bmark PRIVATE chip-config)
target_link_libraries(audio-bmark PRIVATE rocketcore)
