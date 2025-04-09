##### BorAIq Benchmark (Lightweight with libbmark) #####

add_executable(boraiq_bmark
  src/main.c
)

target_include_directories(boraiq_bmark PUBLIC include)

target_link_libraries(boraiq_bmark PRIVATE 
  -L${CMAKE_BINARY_DIR}/glossy -Wl,--whole-archive glossy -Wl,--no-whole-archive m libbmark
)

target_link_options(boraiq_bmark BEFORE PRIVATE -T${CMAKE_CURRENT_SOURCE_DIR}/exclude.ld)


##### Preloader (Full benchmark with weights+tokenizer) #####

add_executable(boraiq_preload
  src/main.c
)

target_include_directories(boraiq_preload PUBLIC include)

target_link_libraries(boraiq_preload PRIVATE 
  -L${CMAKE_BINARY_DIR}/glossy -Wl,--whole-archive glossy -Wl,--no-whole-archive m libbmark
)

target_link_options(boraiq_preload BEFORE PRIVATE -T${CMAKE_CURRENT_SOURCE_DIR}/preload.ld)
