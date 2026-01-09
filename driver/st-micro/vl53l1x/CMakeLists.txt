
add_library(vl53l1x STATIC vl53l1x.c)

target_include_directories(vl53l1x PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

target_link_libraries(vl53l1x PUBLIC metal)
target_link_libraries(vl53l1x PUBLIC clint)
target_link_libraries(vl53l1x PUBLIC i2c)