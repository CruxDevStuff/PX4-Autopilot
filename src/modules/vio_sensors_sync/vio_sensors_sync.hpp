#pragma once

#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

#include <drivers/drv_hrt.h>
#include <lib/perf/perf_counter.h>

#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/orb_test.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/sensor_accel.h>
#include <uORB/topics/vehicle_status.h>

#include <sensors/Integrator.hpp>

using namespace time_literals;


class VIOSensorsSync : public ModuleBase<VIOSensorsSync>, public ModuleParams, public px4::ScheduledWorkItem
{
public:
	VIOSensorsSync();
	~VIOSensorsSync() override;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	void stop();

	bool init();

	int print_status() override;

private:
	void Run() override;

	void update_intervalometer();


	// performance counters
	perf_counter_t	_loop_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": cycle")};
	perf_counter_t	_loop_interval_perf{perf_alloc(PC_INTERVAL, MODULE_NAME": interval")};

	struct hrt_call _engagecall {};
	struct hrt_call _disengagecall {};

	static void engage(void *arg);

	static void disengage(void *arg);
};
