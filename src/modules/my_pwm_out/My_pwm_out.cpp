#include "My_pwm_out.hpp"
My_pwm_out::My_pwm_out() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::test1)
{
}

My_pwm_out::~My_pwm_out()
{
	perf_free(_loop_perf);
	perf_free(_loop_interval_perf);
}

bool My_pwm_out::init()
{
ScheduleOnInterval(100000_us);
	return true;
}

void My_pwm_out::Run()
{
	if (should_exit()) {
		ScheduleClear();
		exit_and_cleanup();
		return;
	}

	perf_begin(_loop_perf);
	perf_count(_loop_interval_perf);

	// Check if parameters have changed
	// if (_parameter_update_sub.updated()) {
	// 	// clear update
	// 	parameter_update_s param_update;
	// 	_parameter_update_sub.copy(&param_update);
	// 	updateParams(); // update module parameters (in DEFINE_PARAMETERS)
	// }//需要参数更新触发时使用
	const hrt_abstime time_stamp_now = hrt_absolute_time();
	const float dt = (time_stamp_now - _time_stamp_last_loop) * 1e-6f;
	_time_stamp_last_loop = time_stamp_now;
	printf("dt = %f\r\n", (double)dt);

	printf("test workitem");
	PX4_INFO("test workitem");
	perf_end(_loop_perf);
}

int My_pwm_out::task_spawn(int argc, char *argv[])
{
	My_pwm_out *instance = new My_pwm_out();

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

int My_pwm_out::print_status()
{
	perf_print_counter(_loop_perf);
	perf_print_counter(_loop_interval_perf);
	return 0;
}

int My_pwm_out::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int My_pwm_out::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Example of a simple module running out of a work queue.

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("my_pwm_out", "template");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int my_pwm_out_main(int argc, char *argv[])
{
	return My_pwm_out::main(argc, argv);
}
