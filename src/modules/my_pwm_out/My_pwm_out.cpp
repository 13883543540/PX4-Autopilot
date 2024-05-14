#include "My_pwm_out.hpp"
#include <cmath>
#include <drivers/drv_pwm_output.h>
#include <px4_arch/io_timer.h>
#define LIMIT(x, min, max) (((x) <= (min)) ? (min) : (((x) > (max)) ? (max) : (x)))

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
float err;
float err_last;
float Kp,Ki,Kd;
float integral;
float umax;
float umin;
void PID_init()
{
	err = 0.0;
	err_last = 0.0;
	integral = 0.0;
	Kp = 0.75;//0.2 *20 150 相差20 提升100/4 升2.5% 调整范围为300 比老版参数除以4
	Ki = 0.05;//0.015*20
	Kd = 0.4;//0.2*20 待调整
}

bool My_pwm_out::init()
{
	ScheduleOnInterval(5000_us);//20ms为一周期运行 考虑增大频率 5ms试试
	return true;
}
float PID_realize(float ActualSpeed,float speed)//位置式
{
	float Actualout;
	err = speed-ActualSpeed;
	integral+=err;
	Actualout = Kp*err + Ki*integral + Kd*(err - err_last);
	err_last = err;
	return Actualout;
}
void Press_PID(my_task_s &my_task,sensor_baro_s &sensor_baro)
{
	if(my_task.task_num >= 2 && my_task.task_num <= 4 )//限值执行条件
	{
		my_task.output = LIMIT(1400-PID_realize(sensor_baro.pressure,my_task.barometer_ab-90),1325,1600);//原150的压差 35-50之间 42.5起始 //120 150
	}
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
	Press_PID(my_task,sensor_baro);
	// Check if parameters have changed
	// if (_parameter_update_sub.updated()) {
	// 	// clear update
	// 	parameter_update_s param_update;
	// 	_parameter_update_sub.copy(&param_update);
	// 	updateParams(); // update module parameters (in DEFINE_PARAMETERS)
	// }//需要参数更新触发时使用
	const hrt_abstime time_stamp_now = hrt_absolute_time();
	static uint8_t err_num,adsorb_num,stop_switch_num;
	static uint16_t last_distance,pwm_out;//
	if (_input_rc_sub.updated())
	{
		_input_rc_sub.copy(&input_rc);
	}
	if (_distance_sensor_sub.updated())//订阅激光测距
	{
		_distance_sensor_sub.copy(&distance_sensor);
	}
	my_task.distance = distance_sensor.current_distance *100;//精度为cm
	if (_sensor_baro_sub.updated())//订阅气压传感器测距
	{
		_sensor_baro_sub.copy(&sensor_baro);
	}
	if (_actuator_armed_sub.updated())//订阅状态信息
	{
		_actuator_armed_sub.copy(&actuator_armed);
	}

	if(input_rc.values[6]>1800 && actuator_armed.armed)//7通道吸附脱落标志置位
	{
		my_task.fall_f = 0;
		my_task.adsorb_f = 1;
	}
	else if(input_rc.values[6]<1800)//加入解锁判定 防止误触
	{
		my_task.fall_f = 1;
		my_task.adsorb_f = 0;
	}
	//13通 1095-1933+2499
	pwm_out = input_rc.values[12]+566;
	if (input_rc.values[7] < 1200 && input_rc.values[8] > 1200 && input_rc.values[8] < 1700)//advance 8 9通道控制
	{
		up_pwm_servo_set(4, pwm_out);up_pwm_servo_set(5, 0);
		up_pwm_servo_set(6, pwm_out);up_pwm_servo_set(7, 0);
	}
	else if(input_rc.values[7] > 1700 && input_rc.values[8] > 1200&&input_rc.values[8] < 1700 )//retreat
	{
		up_pwm_servo_set(4, 0);up_pwm_servo_set(5, pwm_out);
		up_pwm_servo_set(6, 0);up_pwm_servo_set(7, pwm_out);
	}
	else if(input_rc.values[7] > 1200 &&input_rc.values[7] < 1700 && input_rc.values[8] < 1200)//left
	{
		up_pwm_servo_set(4, pwm_out);up_pwm_servo_set(5, 0);
		up_pwm_servo_set(6, 0);up_pwm_servo_set(7, pwm_out);
	}
	else if(input_rc.values[7] > 1200 &&input_rc.values[7] < 1700 && input_rc.values[8] > 1700)//right
	{
		up_pwm_servo_set(4, 0);up_pwm_servo_set(5, pwm_out);
		up_pwm_servo_set(6, pwm_out);up_pwm_servo_set(7, 0);
	}
	else//stop
	{
		up_pwm_servo_set(4, 0);up_pwm_servo_set(5, 0);
		up_pwm_servo_set(6, 0);up_pwm_servo_set(7, 0);
	}
	if(input_rc.values[11]>1800 && my_task.stop_switch == 0)//12通道急停
	{
		stop_switch_num++;
		if(stop_switch_num>=60)//300ms 开启急停打开警报
		{
			my_task.stop_switch = 1;
			my_task.beep_f = 1;
		}
	}
	else if(input_rc.values[11]>1800 && my_task.stop_switch == 1)
	{
		stop_switch_num++;
		if(stop_switch_num>=200)//2s关闭警报
		{
			my_task.beep_f = 0;
		}
	}
	else
	{
		stop_switch_num = 0;
	}
	switch (my_task.task_num)
	{
	case 0://杆位拨动判断
		if(my_task.adsorb_f)//拨杆
		{
			my_task.distance_ab = my_task.distance;
			//记录腔内气压计气压
			my_task.barometer_ab = sensor_baro.pressure;//后续加入取中值 以及传感器失效报错
			my_task.task_num = 1;
		}
		break;
	case 1://加速上升中
		// if (_vehicle_local_position_sub.updated())//订阅z轴速度
		// {
		// 	_vehicle_local_position_sub.copy(&vehicle_local_position);
		// }
		if( my_task.distance - my_task.distance_ab < 100)
		{
			if((my_task.distance - last_distance <= 1)  && ((my_task.barometer_ab -80) > sensor_baro.pressure))//&& (abs(vehicle_local_position.vz) < 0.05)
			{
				adsorb_num++;
				if(adsorb_num >= 20)// 原200ms
				{
					my_task.task_num = 2;
					PID_init();
					adsorb_num = 0;
				}
			}
			else
			{
				adsorb_num =0;
			}
			last_distance = my_task.distance;
			err_num =0;
		}
		else
		{
			err_num++;
			if(err_num >= 40)//200ms
			{
				my_task.err_f =1;
			}
		}
		break;
	case 2://吸附稳定进行切换 需要气压PID
		if(my_task.adsorb_f == 0)//拨杆
		{
			my_task.task_num = 3;//完成解锁
		}
		break;
	case 3://开始对PWM输出进行加速 需要气压PID
		if (_actuator_outputs_sub.updated())
		{
			_actuator_outputs_sub.copy(&actuator_outputs);
		}
		my_task.outputs_sum = actuator_outputs.output[0]+actuator_outputs.output[1]+actuator_outputs.output[2]+actuator_outputs.output[3];
		if(my_task.outputs_sum >=5200 )//加速
		{
			my_task.task_num = 4;
		}
		break;
	case 4://减速 需要气压PID
		if (_actuator_outputs_sub.updated())
		{
			_actuator_outputs_sub.copy(&actuator_outputs);
		}
		my_task.outputs_sum = actuator_outputs.output[0]+actuator_outputs.output[1]+actuator_outputs.output[2]+actuator_outputs.output[3];
		if(my_task.outputs_sum <=5400 )//减速 最低1250 1250 1406 1406
		{
			my_task.distance_ab = my_task.distance;
			my_task.task_num = 5;
		}
		break;
	case 5://下降0-0.05油门为1350
		if(abs(my_task.distance_ab - my_task.distance)>= 5)
		{
			my_task.task_num = 6;
		}
		break;
	case 6://下降0.05-0.15油门为1400
		if(abs(my_task.distance_ab - my_task.distance) >= 35)//后续加上轮子依然直接掉落加入0.7大于一定值自动加大油门
		{
			my_task.task_num = 0;
		}
		break;
	default:
		break;
	}
	my_task.timestamp = time_stamp_now;
	_my_task_pub.publish(my_task);
	// const float dt = (time_stamp_now - _time_stamp_last_loop) * 1e-6f;
	// _time_stamp_last_loop = time_stamp_now;
	// printf("my_task.task_num  = %d\r\n", my_task.task_num );
	// printf("my_task.distance  = %d\r\n", my_task.distance );

	// printf("test workitem");
	// PX4_INFO("test workitem");
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
