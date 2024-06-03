add_library(test_tusb INTERFACE)

add_custom_command(
	OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/banner.o
	WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
	COMMAND ${CMAKE_LINKER} -r -b binary -o ${CMAKE_CURRENT_BINARY_DIR}/banner.o banner.bin
	#places in data
	DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/banner.bin
)

add_custom_target(banner_obj DEPENDS ${CMAKE_CURRENT_BINARY_DIR}/banner.o)


add_library(banner OBJECT IMPORTED GLOBAL)
add_dependencies(banner banner_obj)
set_target_properties(banner PROPERTIES
  IMPORTED_OBJECTS "${CMAKE_CURRENT_BINARY_DIR}/banner.o")

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
	banner
	jmpcoro
	psram_pio
	ar1021_pio
)
