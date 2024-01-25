#include <px4_platform_common/log.h>
#include "vio_sensors_sync.hpp"

extern "C" __EXPORT int vio_sensors_sync_main(int argc, char *argv[]);

VIOSensorsSync::VIOSensorsSync() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::hp_default)
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
	px4_arch_configgpio(AUX_OUT_1);
	update_intervalometer();
	ScheduleOnInterval(2000_us);

	return true;
}

void VIOSensorsSync::engage(void *arg)
{
	px4_arch_gpiowrite(AUX_OUT_1, 1);

	// use atomic operation here to read data and integratefrom the ring buffer. atomic so new data is not added when trying to read the data.
}


void VIOSensorsSync::disengage(void *arg)
{
	px4_arch_gpiowrite(AUX_OUT_1, 0);
	// use atomic operation here to read data and integrate from the ring buffer. atomic so new data is not added when trying to read the data.
}

void VIOSensorsSync::update_intervalometer()
{
	// change this to affect trigger rate.
	int interval_time = 10; // 100hz

	hrt_call_every(&_engagecall, 0, (interval_time * 1000), &VIOSensorsSync::engage, this);
	hrt_call_every(&_disengagecall, 100, (interval_time * 1000), &VIOSensorsSync::disengage, this);
}

int VIOSensorsSync::update_imu_data()
{
	sensor_accel_s sensor_accel;

	if (_accel_sensor_sub.updated()) {
		if (_accel_sensor_sub.copy(&sensor_accel)) {
			test_sample.accel_data = matrix::Vector3f(sensor_accel.x, sensor_accel.y, sensor_accel.z);
		}
	}

	return 0;
}

int VIOSensorsSync::publish_vio_trigger_data()
{
	vio_trigger_data_s vio_trigger_data;

	uint64_t current_timestamp = hrt_absolute_time();

	vio_trigger_data.timestamp = current_timestamp;
	vio_trigger_data.accelerometer_m_s2[0] = 12;
	vio_trigger_data.accelerometer_m_s2[1] = 13;
	vio_trigger_data.accelerometer_m_s2[2] = 14;
	vio_trigger_data.accelerometer_integral_dt = (uint32_t)current_timestamp;

	_vio_trigger_pub.publish(vio_trigger_data);

	return 0;
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

	update_imu_data();

	publish_vio_trigger_data();

	/*
	TODO :
	* Deep Dive into v4l2, USB, UVC
	* Publish IMU data + trigger signal timestamp
	* Find a way to sync imu data with the respective frame using the published timestamp
	* write a tool to log first 100 samples of IMU and Image Frames with UTC in a stack to
		understand/analyze delay / frame capture
		https://medium.com/@athul929/capture-an-image-using-v4l2-api-5b6022d79e1d
	*/


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

	matrix::Vector3f vec(1.0f, 2.0f, 3.0f);

	printf("ACCEL DATA - X: %f, Y: %f, Z: %f \n", double(test_sample.accel_data(0)), double(test_sample.accel_data(1)),
	       double(test_sample.accel_data(2)));
	return 0;
}

void VIOSensorsSync::stop()
{
	ScheduleClear();
	hrt_cancel(&_engagecall);
	hrt_cancel(&_disengagecall);

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

	return VIOSensorsSync::main(argc, argv);

	// TODO : FIX START STOP STATUS
}
