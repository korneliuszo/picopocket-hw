


add_executable(tracer
	main.cpp 
	usb_descriptors.c
)

target_include_directories(tracer PUBLIC ${CMAKE_CURRENT_LIST_DIR})


pico_set_program_name(tracer "picopocket-tracer")
pico_set_program_version(tracer "0.1")
pico_set_binary_type(tracer copy_to_ram)
pico_set_linker_script(tracer ${CMAKE_CURRENT_LIST_DIR}/ram_scratchy.ld)
pico_add_extra_outputs(tracer)

target_link_libraries(tracer PRIVATE
	pico_stdlib
	isa_worker
	tinyusb_device
	tinyusb_board
)

