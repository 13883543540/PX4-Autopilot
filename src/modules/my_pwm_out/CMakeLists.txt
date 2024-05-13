px4_add_module(
	MODULE modules__my_pwm_out
	MAIN my_pwm_out  #与cpp中extern "C" __EXPORT int对应
	COMPILE_FLAGS
		#-DDEBUG_BUILD   # uncomment for PX4_DEBUG output
		#-O0             # uncomment when debugging
	SRCS
		My_pwm_out.hpp
		My_pwm_out.cpp
	DEPENDS
		px4_work_queue
	)
