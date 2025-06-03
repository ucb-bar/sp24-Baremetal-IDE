
#################################
# Build
#################################

add_executable(audio-app
  src/main.c
)

target_include_directories(audio-app PUBLIC include)
target_include_directories(audio-app PUBLIC audio)

target_link_libraries(audio-app PRIVATE 
  -L${CMAKE_BINARY_DIR}/glossy -Wl,--whole-archive glossy -Wl,--no-whole-archive
)