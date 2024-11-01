add_library(isa_worker STATIC
	isa_worker.cpp
)

target_include_directories(isa_worker PUBLIC ${CMAKE_CURRENT_LIST_DIR}/include)

target_link_libraries(isa_worker 
	PUBLIC 
		pthread
)
