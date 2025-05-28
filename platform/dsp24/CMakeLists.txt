########################################################################################################################
# Baremetal Platform Configuration for default chip
########################################################################################################################

file(GLOB_RECURSE PLATFORM_FILES 
  ${CMAKE_CURRENT_SOURCE_DIR}/src/*.c )
add_library(chip-config STATIC chip.c ${PLATFORM_FILES})

# make compatible with riscv64-unknown-elf-gcc (g04696df09) 14.2.0 -- ignore the previous warnings that now have become errors - https://gcc.gnu.org/gcc-14/porting_to.html
target_compile_options(chip-config PRIVATE -fpermissive) # PRIVATE = applies to chip-config source files only

# set terminal device (printf, scanf, etc.) to HTIF
set(TERMINAL_DEVICE_UART0   ON    PARENT_SCOPE)
target_compile_definitions(chip-config PUBLIC -D TERMINAL_DEVICE_UART0)

set(PLATFORM_INCLUDE
  ${CMAKE_CURRENT_SOURCE_DIR}/inc)
target_include_directories(chip-config PUBLIC ${CMAKE_CURRENT_SOURCE_DIR} ${PLATFORM_INCLUDE})


target_link_libraries(chip-config PUBLIC rocketcore)
target_link_libraries(chip-config PUBLIC clint)
target_link_libraries(chip-config PUBLIC uart)
target_link_libraries(chip-config PUBLIC spi)
target_link_libraries(chip-config PUBLIC i2c)
target_link_libraries(chip-config PUBLIC gpio)

