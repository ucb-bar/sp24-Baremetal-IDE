add_library(libbmark STATIC libbmark.c time.c)

target_include_directories(libbmark PUBLIC .)

target_link_libraries(libbmark PUBLIC metal)
target_link_libraries(libbmark PUBLIC uart)
target_link_libraries(libbmark PUBLIC chip-config)

