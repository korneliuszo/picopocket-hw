add_library(test_tusb INTERFACE)
target_include_directories(test_tusb INTERFACE tusb)

add_executable(test
	main.cpp 
	usb_descriptors.c
	testd.cpp
	usb.cpp
)

pico_set_program_name(test "picomem-ng-test")
pico_set_program_version(test "0.1")
pico_set_binary_type(test copy_to_ram)
pico_set_linker_script(test ${CMAKE_CURRENT_LIST_DIR}/ram_scratchy.ld)
pico_add_extra_outputs(test)

target_link_libraries(test PRIVATE
	test_tusb
	pico_stdlib
	isa_worker
	tinyusb_device
	tinyusb_board
	optionrom
	jmpcoro
)
