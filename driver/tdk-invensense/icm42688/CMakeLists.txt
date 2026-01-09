
add_library(icm42688 STATIC icm42688.c)

target_include_directories(icm42688 PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

target_link_libraries(icm42688 PUBLIC metal)
target_link_libraries(icm42688 PUBLIC clint)
target_link_libraries(icm42688 PUBLIC i2c)