add_library(isa_worker STATIC
	isa_worker.cpp
)

pico_generate_pio_header(isa_worker ${CMAKE_CURRENT_LIST_DIR}/isa_dma.pio)

target_include_directories(isa_worker PUBLIC ${CMAKE_CURRENT_LIST_DIR}/include)

target_link_libraries(isa_worker 
	PUBLIC 
		hardware_pio
		hardware_clocks
		pico_multicore
		pico_stdio_semihosting
		pico_stdlib
)
