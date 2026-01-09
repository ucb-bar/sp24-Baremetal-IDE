########################################################################################################################
# file:  CMakeLists.txt
#
# usage: 
#   Edit "VARIABLES"-section to suit project requirements.
#   Build instructions:
#     cmake -S ./ -B ./build/ -D CMAKE_BUILD_TYPE=Debug -D CMAKE_TOOLCHAIN_FILE=./riscv-gcc.cmake
#     cmake --build ./build/ --target all
#
#   Building on Windows:
#     cmake -S ./ -B ./build/ -G "Unix Makefiles" -D CMAKE_BUILD_TYPE=Debug -D CMAKE_TOOLCHAIN_FILE=./riscv-gcc.cmake
#     cmake --build ./build/ --target all
#   Cleaning:
#     cmake --build ./build/ --target clean
#   Install:
#     cmake --build ./build/ --target install
########################################################################################################################
cmake_minimum_required(VERSION 3.10)

project(baremetal-ide LANGUAGES ASM C CXX)


#################################
# Options
#################################

option(BAREMETAL_BUILD_RISCV        "Build for RISC-V platform"               ON  )
option(BAREMETAL_BUILD_X86          "Build for x86 platform"                  OFF )

option(CHIP                         "Build for a specific platform"           OFF )
option(LINKER                "Linker to store program on DRAM or FLASH"       OFF )

#################################
# Toolchain Targets
#################################

set(CMAKE_EXECUTABLE_SUFFIX            ".elf")


#################################
# Flags
#################################

# CPU architecture
set(ARCH                "rv64imafd")
set(ABI                 "lp64d")
set(CMODEL              "medany")
set(ARCH_FLAGS          -march=${ARCH} -mabi=${ABI} -mcmodel=${CMODEL})

# spec
set(SPECS               "nosys.specs")
set(SPEC_FLAGS          -specs=${SPECS})


# convert CHIP to lowercase
string(TOLOWER ${CHIP} CHIP)
string(TOLOWER ${LINKER} LINKER)

# linker script
# HACK: ideally this should be handled by glossy, but currently i couldn't 
# figure out a way to propagate the LINKER_SCRIPT variable to the compile and link commands
if (NOT CHIP)
  message(STATUS "Chip not specified, using default configuration")
  set(CHIP "default")

  set(LINKER_SCRIPT       "${CMAKE_SOURCE_DIR}/glossy/glossy.ld")
else()
  set(LINKER_SCRIPT       "${CMAKE_SOURCE_DIR}/platform/${CHIP}/${CHIP}-${LINKER}.ld")
endif()

add_subdirectory(${CMAKE_SOURCE_DIR}/platform/${CHIP})

add_compile_options(-O0)
add_compile_options(-Wall -Wextra)
add_compile_options(-s)

# add_compile_options(-ffunction-sections -fdata-sections -fno-common -fno-builtin-printf -fno-pie)
# add_compile_options(-Wall -Wextra -Warray-bounds -Wno-unused-parameter -Wcast-qual)
add_compile_options(${ARCH_FLAGS})
add_compile_options(${SPEC_FLAGS})

add_link_options(-static)
# add_link_options(-ffunction-sections -fdata-sections -fno-common -fno-builtin-printf -fno-pie)
# add_link_options(-Wall -Wextra -Warray-bounds -Wno-unused-parameter -Wcast-qual)
add_link_options(-nostartfiles)
add_link_options(-nostartfiles)
add_link_options(${ARCH_FLAGS})
add_link_options(${SPEC_FLAGS})
add_link_options(-T ${LINKER_SCRIPT})



#################################
# Build
#################################

add_executable(app
  app/src/main.c
)

add_executable(quad
  app/src/quad.c
)

add_executable(quad_simple_test
  app/src/quad_simple_test.c
)

add_executable(test_sensor_fusion
  app/src/test_sensor_fusion.c
)

add_executable(quad_sitl
  app/src/quad_sitl.c
)

add_executable(motor_calibration
  app/src/motor_calibration.c
)

target_include_directories(app PUBLIC app/include)
target_include_directories(quad PUBLIC app/include)
target_include_directories(quad_simple_test PUBLIC app/include)
target_include_directories(test_sensor_fusion PUBLIC app/include)
target_include_directories(quad_sitl PUBLIC app/include)
target_include_directories(motor_calibration PUBLIC app/include)


#################################
# Dependencies
#################################

add_subdirectory(driver)
add_subdirectory(glossy)
add_subdirectory(lib)
add_subdirectory(lab)
add_subdirectory(encoders1)

target_link_libraries(app PRIVATE 
  -L${CMAKE_BINARY_DIR}/glossy -Wl,--whole-archive glossy -Wl,--no-whole-archive m
)

target_link_libraries(quad PRIVATE 
  -L${CMAKE_BINARY_DIR}/glossy -Wl,--whole-archive glossy -Wl,--no-whole-archive m
)

target_link_libraries(quad_simple_test PRIVATE 
  -L${CMAKE_BINARY_DIR}/glossy -Wl,--whole-archive glossy -Wl,--no-whole-archive m
)

target_link_libraries(test_sensor_fusion PRIVATE 
  -L${CMAKE_BINARY_DIR}/glossy -Wl,--whole-archive glossy -Wl,--no-whole-archive m
)

target_link_libraries(quad_sitl PRIVATE 
  -L${CMAKE_BINARY_DIR}/glossy -Wl,--whole-archive glossy -Wl,--no-whole-archive m
)

target_link_libraries(motor_calibration PRIVATE 
  -L${CMAKE_BINARY_DIR}/glossy -Wl,--whole-archive glossy -Wl,--no-whole-archive m
)

target_link_libraries(quad PUBLIC icm42688)
target_link_libraries(quad PUBLIC vl53l1x)
target_link_libraries(quad_simple_test PUBLIC icm42688)
target_link_libraries(test_sensor_fusion PUBLIC icm42688)

#################################
# Library Installation
#################################

set(CMAKE_INSTALL_PREFIX "$ENV{RISCV}/riscv64-unknown-elf")

set_property(TARGET glossy PROPERTY PUBLIC_HEADER
  glossy/include/riscv.h
  glossy/include/riscv_encoding.h
  glossy/include/riscv_interrupt.h
)

set_property(TARGET glossy PROPERTY RESOURCE
  glossy/glossy.ld
  glossy/glossy.specs
)

install(TARGETS glossy
  LIBRARY DESTINATION "lib"
  PUBLIC_HEADER DESTINATION "include"
  RESOURCE DESTINATION "lib"
)

install(TARGETS htif
  LIBRARY DESTINATION "lib"
  PUBLIC_HEADER DESTINATION "include"
  RESOURCE DESTINATION "lib"
)
