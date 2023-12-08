#include <px4_platform_common/log.h>
#include "vio_sensors_sync.hpp"

extern "C" __EXPORT int vio_sensors_sync_main(int argc, char *argv[]);

VIOSensorsSync::VIOSensorsSync() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::nav_and_controllers)
{
}

VIOSensorsSync::~VIOSensorsSync()
{
	perf_free(_loop_perf);
	perf_free(_loop_interval_perf);
}

bool VIOSensorsSync::init()
{
	// 2000 us interval,  500 Hz rate
	ScheduleOnInterval(2000_us);

	return true;
}

void VIOSensorsSync::Run()
{
	if (should_exit()) {
		ScheduleClear();
		exit_and_cleanup();
		return;
	}

	perf_begin(_loop_perf);
	perf_count(_loop_interval_perf);


	/// DO SOME STUFF


	perf_end(_loop_perf);
}

int VIOSensorsSync::task_spawn(int argc, char *argv[])
{
	VIOSensorsSync *instance = new VIOSensorsSync();

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		}

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;

	return PX4_ERROR;
}

int VIOSensorsSync::print_status()
{
	perf_print_counter(_loop_perf);
	perf_print_counter(_loop_interval_perf);
	return 0;
}

int VIOSensorsSync::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int VIOSensorsSync::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description


)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("vio_sensors_sync", "template");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

int vio_sensors_sync_main(int argc, char *argv[])
{
	PX4_INFO("starting sync!");
	return VIOSensorsSync::main(argc, argv);
}
