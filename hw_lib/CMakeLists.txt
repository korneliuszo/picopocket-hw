add_library(isa_worker STATIC
	isa_worker.cpp
)

pico_generate_pio_header(isa_worker ${CMAKE_CURRENT_LIST_DIR}/isa_dma.pio)

target_include_directories(isa_worker PUBLIC ${CMAKE_CURRENT_LIST_DIR}/include)

target_link_libraries(isa_worker 
	PUBLIC 
		hardware_pio
		hardware_clocks
		hardware_dma
		pico_multicore
		pico_stdio_semihosting
		pico_stdlib
)

add_library(psram_pio INTERFACE)
pico_generate_pio_header(psram_pio ${CMAKE_CURRENT_LIST_DIR}/psram.pio)
target_include_directories(psram_pio INTERFACE ${CMAKE_CURRENT_LIST_DIR}/include)
target_link_libraries(psram_pio INTERFACE hardware_pio hardware_clocks hardware_dma)

add_library(ar1021_pio INTERFACE)
pico_generate_pio_header(ar1021_pio ${CMAKE_CURRENT_LIST_DIR}/ar1021.pio)
target_include_directories(ar1021_pio INTERFACE ${CMAKE_CURRENT_LIST_DIR}/include)
target_link_libraries(ar1021_pio INTERFACE hardware_pio hardware_clocks)
