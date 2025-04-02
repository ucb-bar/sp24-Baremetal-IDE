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

#################################
# Toolchain Targets
#################################

set(CMAKE_EXECUTABLE_SUFFIX            ".elf")

#################################
# Flags
#################################

# CPU architecture
set(ARCH                "rv64gcv_zfh_zvfh")
set(ABI                 "lp64d")
set(CMODEL              "medany")
set(ARCH_FLAGS          -march=${ARCH} -mabi=${ABI} -mcmodel=${CMODEL})

# spec
set(SPECS               "nosys.specs")
set(SPEC_FLAGS          -specs=${SPECS})

# convert CHIP to lowercase
string(TOLOWER ${CHIP} CHIP)

# linker script
# HACK: ideally this should be handled by glossy, but currently i couldn't 
# figure out a way to propagate the LINKER_SCRIPT variable to the compile and link commands
if (NOT CHIP)
  message(STATUS "Chip not specified, using default configuration")
  set(CHIP "default")

  set(LINKER_SCRIPT       "${CMAKE_SOURCE_DIR}/glossy/glossy.ld")
else()
  set(LINKER_SCRIPT       "${CMAKE_SOURCE_DIR}/platform/${CHIP}/${CHIP}.ld")
endif()

add_subdirectory(${CMAKE_SOURCE_DIR}/platform/${CHIP})

add_compile_options(-O1)
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

add_executable(app_interface
  app_interface/src/main.c
)

add_executable(app_naichen_at_3am
  app_naichen_at_3am/src/main.c
)

add_executable(app_SGEMM_vectorized
  app_SGEMM_vectorized/src/main.c
)

add_executable(app_shazam_demo
  app_shazam_demo/src/main.c
  app_shazam_demo/src/kiss_fft.c
)

add_executable(dsp-saturn
  dsp-saturn/src/fft2_main.c
  dsp-saturn/common/syscalls.c
)

add_executable(dsp-sgemm
  benchmarks/vec-sgemm/vec-sgemm_main.c
  benchmarks/vec-sgemm/vec-sgemm.S
)

add_executable(dsp-igemm
  benchmarks/vec-igemm/vec-igemm_main.c
  benchmarks/vec-igemm/vec-igemm.S
)

add_executable(dsp-strcmp
  benchmarks/vec-strcmp/vec-strcmp_main.c
  benchmarks/vec-strcmp/vec-strcmp.S
)

target_include_directories(app_interface PUBLIC app_interface/include)

target_include_directories(app_naichen_at_3am PUBLIC app_naichen_at_3am/include)

target_include_directories(app_SGEMM_vectorized PUBLIC app_SGEMM_vectorized/include)

target_include_directories(app_shazam_demo PUBLIC app_shazam_demo/include)

target_include_directories(dsp-saturn PUBLIC dsp-saturn/common)

target_include_directories(dsp-sgemm PUBLIC benchmarks/common)

target_include_directories(dsp-igemm PUBLIC benchmarks/common)

target_include_directories(dsp-strcmp PUBLIC benchmarks/common)

#################################
# Dependencies
#################################

add_subdirectory(driver)
add_subdirectory(glossy)
add_subdirectory(lib)
add_subdirectory(lab)

target_link_libraries(shazam_demo PRIVATE m)

target_link_libraries(app_rushed PRIVATE 
  -L${CMAKE_BINARY_DIR}/glossy -Wl,--whole-archive glossy -Wl,--no-whole-archive
)

target_link_libraries(app_naich_comparison PRIVATE 
  -L${CMAKE_BINARY_DIR}/glossy -Wl,--whole-archive glossy -Wl,--no-whole-archive
)

target_link_libraries(app_interface PRIVATE 
  -L${CMAKE_BINARY_DIR}/glossy -Wl,--whole-archive glossy -Wl,--no-whole-archive
)

target_link_libraries(shazam_demo PRIVATE 
  -L${CMAKE_BINARY_DIR}/glossy -Wl,--whole-archive glossy -Wl,--no-whole-archive
)

target_link_libraries(dsp-saturn PRIVATE 
  -L${CMAKE_BINARY_DIR}/glossy -Wl,--whole-archive glossy -Wl,--no-whole-archive
)

target_link_libraries(dsp-sgemm PRIVATE
  -L${CMAKE_BINARY_DIR}/glossy -Wl,--whole-archive glossy -Wl,--no-whole-archive
)

target_link_libraries(dsp-igemm PRIVATE
  -L${CMAKE_BINARY_DIR}/glossy -Wl,--whole-archive glossy -Wl,--no-whole-archive
)

target_link_libraries(dsp-strcmp PRIVATE
  -L${CMAKE_BINARY_DIR}/glossy -Wl,--whole-archive glossy -Wl,--no-whole-archive
)


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
