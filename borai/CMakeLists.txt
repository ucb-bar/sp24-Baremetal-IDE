### Executables ###
add_compile_options(-g)
add_executable(borai
  fp32/src/main.c
  fp32/src/syscalls.c
)

target_include_directories(borai PUBLIC fp32/include)

add_executable(boraiq
  int8/src/main.c
)

target_include_directories(boraiq PUBLIC int8/include)

### Dependencies ###

target_link_libraries(borai PRIVATE 
  -L${CMAKE_BINARY_DIR}/glossy -Wl,--whole-archive glossy -Wl,--no-whole-archive
)
## Include Math.h library
target_link_libraries(borai PRIVATE m)

target_link_libraries(boraiq PRIVATE 
  -L${CMAKE_BINARY_DIR}/glossy -Wl,--whole-archive glossy -Wl,--no-whole-archive -Wl,--defsym=__boot_hart=$2 -Wl,--defsym=__main=main
)
## Include Math.h library
target_link_libraries(boraiq PRIVATE m)
