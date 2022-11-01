//###########################################################################
//
// filename		:Struct.h
//
// TwinCopter structure header file.
//
// by Choi gi baek (MAZE 16TH)
//
//###########################################################################
// $Release Date: 2012.01.18 $
//###########################################################################

#ifndef __STRUCT_H__
#define __STRUCT_H__

#ifdef __STRUCT__

#undef __STRUCT__
#define __STRUCT_EXT__

#else

#define __STRUCT_EXT__	extern

#endif

//////////////////DC motor/////////////////////
typedef volatile struct motor_struct
{
	Uint16 u16qep_sample;
	Uint16 u16decel_flag:1;
	int16 int16qep_value;
	int32 int32accel;
	_iq17 iq17tick_distance;
	_iq17 iq17distance_sum;
	_iq17 iq17distance_sum_mm;
	_iq17 iq17user_distance;
	_iq17 iq17err_distance;
	_iq17 iq17decel_distance;
	_iq17 iq17decel_vel;
	_iq17 iq17current_vel[ 4 ];
	_iq17 iq17cur_vel_avr;
	_iq17 iq17user_vel;
	_iq17 iq17next_vel;
	_iq17 iq17err_vel_sum;
	_iq17 iq17err_vel[ 4 ];
	_iq17 iq17proportion_val;
	_iq17 iq17integral_val;
	_iq17 iq17differential_val;
	_iq17 iq17pid_output;
	_iq17 iq17kp;
	_iq17 iq17ki;
	_iq17 iq17kd;
	_iq7 iq7gone_distatnce;
	
}motor_str;
__STRUCT_EXT__ motor_str DC_motor;

//////////////////flag/////////////////////
typedef volatile struct bitfield_flag
{
	Uint16 motor_on:1;
	Uint16 move_state:1;
	Uint16 stop_check:1;
	Uint16 err:1;
}flag_str;

__STRUCT_EXT__ flag_str g_flag;



#endif

