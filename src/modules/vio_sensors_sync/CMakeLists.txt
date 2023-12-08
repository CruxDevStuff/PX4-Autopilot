px4_add_module(
	MODULE modules__vio_sensors_sync
	MAIN vio_sensors_sync
	STACK_MAIN 2000
	SRCS
		vio_sensors_sync.cpp
		vio_sensors_sync.hpp
	DEPENDS
	 	px4_work_queue
)
