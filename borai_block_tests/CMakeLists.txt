### Executables ###

# QTDP
add_compile_options(-g)
add_executable(borai_qtdp
  qtdp/src/main.c
)

target_include_directories(borai_qtdp PUBLIC qtdp/include)

# DMA MAC
add_executable(borai_dma_mac
  dma_mac/src/main.c
)

target_include_directories(borai_dma_mac PUBLIC dma_mac/include)

### Dependencies ###

target_link_libraries(borai_qtdp PRIVATE 
  -L${CMAKE_BINARY_DIR}/glossy -Wl,--whole-archive glossy -Wl,--no-whole-archive -Wl,--defsym=__boot_hart=$2 -Wl,--defsym=__main=main
)

target_link_libraries(borai_dma_mac PRIVATE 
  -L${CMAKE_BINARY_DIR}/glossy -Wl,--whole-archive glossy -Wl,--no-whole-archive -Wl,--defsym=__boot_hart=$2 -Wl,--defsym=__main=main
)