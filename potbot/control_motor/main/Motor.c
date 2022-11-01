//###########################################################################
//
// FILE		: Motor.c
//
// TITLE	: Motor c file.
//
// Author	: Yuk Keun Ho
//
// Company	: Maze & Hz
//
//###########################################################################
// $Release Date: 2011.10.16 $
//###########################################################################

#define   _MOTOR_


#include "DSP280x_Device.h"
#include "DSP280x_Examples.h"   // DSP280x Examples Include File
#include "Main.h"
#include "Motor.h"


///////////////////////////////////////////////    motor information   ///////////////////////////////////////////////////

//#define WHEEL_RADIUS			36
//#define Gear_Ratio 				3.35
#define M_PI					3.141592653589

#define SAMPLE_FRQ				_IQ( 0.5 )			//500us

//PULSE_TO_D = (WHEEL_diameter * M_PI) / (encoder_pulse * 2) / geer_ratio 
//(44 * M_PI) / 26 / 1
#define PULSE_TO_D				_IQ( 5.316541413767)

////////////////////////////////////////////       PID information       ///////////////////////////////////////////////////

// 10000으로 임의로 잡고 계산했을 경우 공회전으로 7800mm/s 정도까지 올라가는것 확인-> 정속 주행일때를 위해 좀 더 올려야 할 듯...
#define MAX_PID_OUT				_IQ( 9950.0 )
#define MIN_PID_OUT				-_IQ( 9950.0 )

//PULSE_TO_V = (WHEEL_RADIUS * M_PI) / (encoder_pulse * 4) / geer_ratio / SAMPLE_FRQ
//(44 * M_PI) /26 / 1 / 0.0005

#define PULSE_TO_V  			_IQ( 106.3308282753 )
#define PWM_CONVERT				_IQ( 0.2 )		//PWM주파수 최대값(EPWM.c) / MAX_PID_OUT => 2000 / 9000(수정 요망)


void motor_init( motor_str *pm )
{
	memset( ( void * )pm, 0x00, sizeof( motor_str ) );

	pm->int32accel = 5;
	pm->iq17kp = _IQ( 0.8 );		//0.8
	pm->iq17ki = _IQ( 0.00002 );
	pm->iq17kd = _IQ( 0.85 );		//0.85
	pm->iq17user_vel = _IQ( 0.0 );

}

void move_to_move( volatile _iq17 dist, volatile _iq17 dec_dist, volatile _iq17 tar_vel, volatile _iq17 dec_vel, volatile int32 acc )
{
	StopCpuTimer2();

	DC_motor.int32accel = acc;

	DC_motor.iq17decel_distance = dec_dist;

	DC_motor.iq17user_distance = dist;
	
	DC_motor.iq17user_vel = tar_vel;

	DC_motor.iq17err_distance = DC_motor.iq17user_distance - DC_motor.iq17distance_sum;

	DC_motor.iq17decel_vel = dec_vel;

	DC_motor.u16decel_flag = ON;

	g_flag.move_state = ON;

	StartCpuTimer2();
	
}

void move_to_end( volatile _iq17 dist, volatile _iq17 vel, volatile int32 acc )
{
	_iq7 decel_distance = _IQ7( 0 );

	StopCpuTimer2();

	DC_motor.int32accel = acc;
	
	DC_motor.iq17decel_distance = ( decel_distance << 10 );

	DC_motor.iq17user_distance = dist;

	DC_motor.iq17user_vel = vel;

	DC_motor.iq17err_distance = DC_motor.iq17user_distance - DC_motor.iq17distance_sum;

	DC_motor.iq17decel_vel = _IQ( 0 );

	DC_motor.u16decel_flag =	ON;

	g_flag.move_state = OFF;

	StartCpuTimer2();

}


interrupt void motor_pid_ISR( void )
{
	//TxPrintf("motor interrupt\n");

	//Qep값 받기
	DC_motor.u16qep_sample = ( Uint16 )RightQepRegs.QPOSCNT;
	//Qep값 초기화
	RightQepRegs.QEPCTL.bit.SWI = 1;
	//Qep 부호 compute
	DC_motor.int16qep_value = ( DC_motor.u16qep_sample > 13 ) ? ( int16 )(27 - DC_motor.u16qep_sample ) : -( int16 )DC_motor.u16qep_sample;
	//거리 compute
	DC_motor.iq17tick_distance = _IQmpy( ( ( int32 )DC_motor.int16qep_value << 17 ), PULSE_TO_D );

	DC_motor.iq17distance_sum += DC_motor.iq17tick_distance;
	DC_motor.iq17err_distance = DC_motor.iq17user_distance - DC_motor.iq17distance_sum;

	DC_motor.iq7gone_distatnce += ( DC_motor.iq17tick_distance >> 10 );
	DC_motor.iq17distance_sum_mm = _IQmpy(DC_motor.iq17distance_sum, _IQ(0.1));
	//속도 compute
	DC_motor.iq17current_vel[ 3 ] = DC_motor.iq17current_vel[ 2 ];
	DC_motor.iq17current_vel[ 2 ] = DC_motor.iq17current_vel[ 1 ];
	DC_motor.iq17current_vel[ 1 ] = DC_motor.iq17current_vel[ 0 ];
	DC_motor.iq17current_vel[ 0 ] = _IQmpy( ( ( int32 )DC_motor.int16qep_value << 17 ), PULSE_TO_V ) >> 2;
	DC_motor.iq17cur_vel_avr = ( DC_motor.iq17current_vel[ 0 ] + DC_motor.iq17current_vel[ 1 ] + DC_motor.iq17current_vel[ 2 ] + DC_motor.iq17current_vel[ 3 ] );
	//move_to_move & move_to_end call
	if( DC_motor.u16decel_flag )
	{
		if( DC_motor.iq17decel_distance >= _IQabs( DC_motor.iq17err_distance ) )
		{
			DC_motor.iq17user_vel = DC_motor.iq17decel_vel;

			DC_motor.u16decel_flag = OFF;
		}
	}
	//가감속도 compute
	if( DC_motor.iq17user_vel > DC_motor.iq17next_vel )
	{
		DC_motor.iq17next_vel += _IQmpy( ( DC_motor.int32accel << 17 ) , SAMPLE_FRQ );
		if( DC_motor.iq17user_vel < DC_motor.iq17next_vel )
			DC_motor.iq17next_vel = DC_motor.iq17user_vel;
	}
	else if( DC_motor.iq17user_vel < DC_motor.iq17next_vel )
	{
		DC_motor.iq17next_vel -= _IQmpy( ( DC_motor.int32accel << 17 ) , SAMPLE_FRQ );
		if( DC_motor.iq17user_vel > DC_motor.iq17next_vel )
			DC_motor.iq17next_vel = DC_motor.iq17user_vel;
	}
	else;
	//Pid compute
	DC_motor.iq17err_vel_sum -= DC_motor.iq17err_vel[ 3 ];
	DC_motor.iq17err_vel[ 3 ] = DC_motor.iq17err_vel[ 2 ];
	DC_motor.iq17err_vel[ 2 ] = DC_motor.iq17err_vel[ 1 ];
	DC_motor.iq17err_vel[ 1 ] = DC_motor.iq17err_vel[ 0 ];
	DC_motor.iq17err_vel[ 0 ] = DC_motor.iq17next_vel - DC_motor.iq17cur_vel_avr;
	DC_motor.iq17err_vel_sum += DC_motor.iq17err_vel[ 0 ];

	DC_motor.iq17proportion_val = _IQmpy( DC_motor.iq17kp , DC_motor.iq17err_vel[ 0 ] );
	DC_motor.iq17integral_val = _IQmpy( DC_motor.iq17ki, DC_motor.iq17err_vel_sum );
	DC_motor.iq17differential_val = _IQmpy( DC_motor.iq17kd, ( ( DC_motor.iq17err_vel[ 0 ] - DC_motor.iq17err_vel[ 3 ] ) + _IQmpy( _IQ( 3 ), ( DC_motor.iq17err_vel[ 1 ] - DC_motor.iq17err_vel[ 2 ] ) ) ) );
	DC_motor.iq17pid_output += DC_motor.iq17proportion_val + DC_motor.iq17integral_val + DC_motor.iq17differential_val;
	//Pid -> Pwm
	if( g_flag.motor_on == ON )
	{
		if( DC_motor.iq17pid_output >= _IQ( 0 ) )
		{
			if( DC_motor.iq17pid_output > MAX_PID_OUT )
				DC_motor.iq17pid_output = MAX_PID_OUT;

			GpioDataRegs.GPASET.bit.GPIO23 = 1;

			RightPwmRegs.CMPA.half.CMPA = ( Uint16 )( _IQmpy( DC_motor.iq17pid_output, PWM_CONVERT ) >> 17 );
		}
		else
		{
			if( DC_motor.iq17pid_output < MIN_PID_OUT )
				
				DC_motor.iq17pid_output = MIN_PID_OUT;

			GpioDataRegs.GPACLEAR.bit.GPIO23 = 1;

			RightPwmRegs.CMPA.half.CMPA = ( Uint16 )( _IQmpy( -_IQ( 1.0 ), _IQmpy( DC_motor.iq17pid_output, PWM_CONVERT ) ) >> 17 );
		}		
			
	}
	
}
