cmake_minimum_required(VERSION 3.16)
# initialize the SDK based on PICO_SDK_PATH
# note: this must happen before project()

option(PICOPOCKET_SIM "use simulation" FALSE)

if(NOT PICOPOCKET_SIM)
include(${CMAKE_CURRENT_LIST_DIR}/pico-sdk/pico_sdk_init.cmake)

set(CMAKE_BUILD_TYPE "RelWithDebInfo")

project(hw_lib C CXX ASM)
set(PICO_BOARD_HEADER_DIRS ${CMAKE_CURRENT_LIST_DIR}/boards ${PICO_BOARD_HEADER_DIRS})
if("${PICO_BOARD}" STREQUAL "")
 set(PICO_BOARD board)
endif()

set(PICO_CYW43_SUPPORTED "1")
pico_register_common_scope_var(PICO_CYW43_SUPPORTED)

pico_sdk_init()

add_subdirectory(hw_lib)
else()
project(hw_lib C CXX ASM)
add_compile_definitions(PICOPOCKET_SIM)
add_subdirectory(hw_lib_sim)	
endif()

add_subdirectory(jmpcoro_lib)

if(PROJECT_IS_TOP_LEVEL)
  add_subdirectory(test_fw)
  add_subdirectory(tracer)
endif()

