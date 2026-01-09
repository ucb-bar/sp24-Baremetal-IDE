
add_library(pmw3901 STATIC pmw3901.c)

target_include_directories(pmw3901 PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

target_link_libraries(pmw3901 PUBLIC metal)
target_link_libraries(pmw3901 PUBLIC spi)