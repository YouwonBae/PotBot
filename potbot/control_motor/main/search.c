//###########################################################################
//
// FILE		: search.c
//
// TITLE		: _666_ Tracer search source file.
//
// Author	: Yuk Keun Ho
//
// Company	: Maze & Hertz
//
//###########################################################################
// $Release Date: 2009.11.15 $
//###########################################################################


#include "DSP280x_Device.h"     // DSP280x Headerfile Include File
#include "DSP280x_Examples.h"   // DSP280x Examples Include File

void run_init( void )
{
	g_flag.motor_on = OFF;
	g_flag.stop_check = OFF;
	g_flag.err = OFF;

	DC_motor.iq17decel_distance =_IQ( 0 );

	DC_motor.iq17next_vel = _IQ( 0 );

	DC_motor.iq7gone_distatnce = _IQ7( 0 );
}

void angle_adjust(volatile float32 ang)
{
	if(ang >= -45 && ang <= 45 )
	{
		
		angle_adjust_iq17 = _IQ(ang);
		angle_adjust_iq17 = _IQmpy(_IQ(15.3956), angle_adjust_iq17) + _IQ(950);
		angle_adjust_uint16 = (Uint16)(angle_adjust_iq17 >>17);
	}
	else
	{
		if(ang < 0) angle_adjust_uint16 = 250;
		else angle_adjust_uint16 = 1550;
	}

}


void search_run( void )
{
	char RcvData;
	unsigned char scib_buf;
	
	TxPrintf("search \n");

	g_int32drive_vel = 0;
	
	move_to_move( _IQ( 1000 ), _IQ(0), _IQ( g_int32drive_vel ), _IQ( g_int32drive_vel ), ( int32 )5);
	run_init();
	
	g_flag.motor_on = ON;

	while(1)
	{
		//////////////////////////////SCI READ////////////////////////////////////
		scib_buf = SCIx_RxChar_B();
		
		if(scib_buf == '\n')
		{
			sscanf(CONTROL_buf, "*%lf,%lf\n", &CONTROL_steer,&CONTROL_speed);

			//TxPrintf("%lf\n", _IQ17toF(CONTROL_speed_iq17));

			memset((void *)CONTROL_buf,0x00,sizeof(unsigned char)*25);
		}
		else
			strncat((char*)CONTROL_buf, (char*)&scib_buf, 1);
		///////////////////////////////Trash Error deny//////////////////////////////////
		
		if(CONTROL_steer > CONTROL_steer_next)
			CONTROL_steer_next += 1;
		else if(CONTROL_steer < CONTROL_steer_next)
			CONTROL_steer_next -= 1;
		else
			CONTROL_steer_next = CONTROL_steer;//deny error
			
		
		//if(CONTROL_speed > CONTROL_speed_next)
		//	CONTROL_speed_next += 1;
		//else if(CONTROL_speed < CONTROL_speed_next)
		//	CONTROL_speed_next -= 1;
		//else
		//	CONTROL_speed_next = CONTROL_speed;

		//CONTROL_speed_iq17 = _IQ(CONTROL_speed_next); //deny error
		
		angle_adjust(CONTROL_steer_next); //angle unit transform
		CONTROL_steer_uint16 = angle_adjust_uint16;
		CONTROL_speed_iq17 = _IQ(CONTROL_speed);
		//////////////////////////////////////////////////////////////////
			
		DC_motor.iq17user_vel = CONTROL_speed_iq17; //input speed
			
		if(CONTROL_steer_uint16 >= 1650) // control maxmin error
			{
				LeftPwmRegs.CMPA.half.CMPA = 1650;
			}
		else if(CONTROL_steer_uint16 <= 250)
			{
				LeftPwmRegs.CMPA.half.CMPA = 250;
			}
		else
			LeftPwmRegs.CMPA.half.CMPA = CONTROL_steer_uint16; //input steer(250~1650)

		//distance_sum = _IQtoF(DC_motor.iq17distance_sum_mm);
		//TxPrintf_B("d:%lf\n",distance_sum);

		//TxPrintf("%d ,", LeftPwmRegs.CMPA.half.CMPA);
		//TxPrintf("%lf\n", _IQ17toF(DC_motor.iq17user_vel));
		//TxPrintf_B("%d",gone_distance);

		//TxPrintf_B("hello\n");
		
		//////////////////////////////Control with keyboard///////////////////////////////////
		/*
		RcvData = SCIx_RxChar();

		switch(RcvData)
		{
			case 'W':
			case 'w':
				if(DC_motor.iq17user_vel >= _IQ(0))
				{
					DC_motor.iq17user_vel += _IQ(1);
				}
				else
				{
					DC_motor.iq17user_vel =_IQ(0);
				}
				break;
				
			case 'S':
			case 's':
				if(DC_motor.iq17user_vel <=_IQ(0))
				{
					DC_motor.iq17user_vel -= _IQ(1);
				}
				else
				{
					DC_motor.iq17user_vel =_IQ(0);
				}
				break;
				
			case 'D':
			case 'd':
					if(LeftPwmRegs.CMPA.half.CMPA >= 1680)
					{
						LeftPwmRegs.CMPA.half.CMPA = 1680;
					}
					else
					LeftPwmRegs.CMPA.half.CMPA +=  (Uint16 )10;
					
				break;
				
			case 'A':
			case 'a':
					if(LeftPwmRegs.CMPA.half.CMPA <= 210)
					{
						LeftPwmRegs.CMPA.half.CMPA = 210;
					}
					else
					LeftPwmRegs.CMPA.half.CMPA -=  (Uint16 )10;
					
			break;
		
		}
		TxPrintf("%d ,", LeftPwmRegs.CMPA.half.CMPA);
		TxPrintf("%lf\n", _IQ17toF(DC_motor.iq17user_vel));
		*/
		
	}
}

