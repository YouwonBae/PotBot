//###########################################################################
//
// FILE		: Variable.h
//
// TITLE	: Variable.h file.
//
// Author	: Yuk Keun Ho
//
// Company	: Maze & Hz
//
//###########################################################################
// $Release Date: 2011.10.6 $
//###########################################################################



#ifdef _MAIN_
	#ifndef __VARIABLE_EXT__
		#define __VARIABLE_EXT__
	#endif
#else
	#ifndef __VARIABLE_EXT__
		#define __VARIABLE_EXT__	extern
	#endif
#endif	

__VARIABLE_EXT__ int32 g_int32drive_vel;	
__VARIABLE_EXT__ float32 distance_sum;	


///////////////////////sci////////////////////
__VARIABLE_EXT__ 	unsigned char	CONTROL_buf[25];

__VARIABLE_EXT__	float32	CONTROL_steer;
__VARIABLE_EXT__	Uint16  CONTROL_steer_uint16;

__VARIABLE_EXT__	float32 CONTROL_speed;
__VARIABLE_EXT__	_iq17	CONTROL_speed_iq17;

__VARIABLE_EXT__	float32	CONTROL_speed_next;
__VARIABLE_EXT__	float32	CONTROL_steer_next;
/////////////////////angle///////////////////
__VARIABLE_EXT__	_iq17	angle_adjust_iq17;
__VARIABLE_EXT__	Uint16	angle_adjust_uint16;










