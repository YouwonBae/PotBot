////////////////////////////ctrl+F9///////////////////////////////////
#define _MAIN_
#define __STRUCT__


#include "DSP280x_Device.h"     // DSP281x Headerfile Include File
#include "DSP280x_Examples.h"  // DSP281x Examples Include File

void System_Init(void)
{
	DINT;				// ��ü ���ͷ�Ʈ ����
	InitSysCtrl();		// ��ġ�� ����, PLL �ʱ�ȭ, �ֺ� ��ġ Ŭ������
	InitGpio();			// ����� ��Ʈ �ʱ�ȭ	
	InitCpuTimers();
	
	MemCopy(&RamfuncsLoadStart, 	&RamfuncsLoadEnd, 		&RamfuncsRunStart); 
	MemCopy(&RamfuncsLoadStart1, 	&RamfuncsLoadEnd1, 	&RamfuncsRunStart1);
	
	InitSci();			
	InitSpi();
	InitPieCtrl();		// PIE ���� �������� �ʱ�ȭ ����
	IER = 0x0000;		// ���ͷ�Ʈ �ο��̺� �������� Ŭ����
	IFR = 0x0000;		// ���ͷ�Ʈ �÷��� �������� Ŭ����
	InitPieVectTable();
	InitAdc();

	Init_ISR();

	InitEPWM( &RightPwmRegs );
	InitEPWM_Servo( &LeftPwmRegs);
	
	InitEQep( &LeftQepRegs );
	InitEQep( &RightQepRegs );
	
}

void Variable_Init( void )
{
	g_int32drive_vel = 0;
	motor_init( &DC_motor );
	LeftPwmRegs.CMPA.half.CMPA = 950;

	memset((void *)CONTROL_buf,0x00,sizeof(unsigned char)*25);

	CONTROL_steer = 0.0;
	CONTROL_speed = 0.0;

	CONTROL_steer_uint16 = 0;
	CONTROL_speed_iq17 = _IQ(0.0);

	CONTROL_steer_next = 0.0;
	CONTROL_speed_next = 0.0;

	angle_adjust_uint16 = 0;

	distance_sum = 0.0;
	
}

void main(void) 
{
	System_Init();
	Variable_Init();

	StartCpuTimer2();

	//TxPrintf("go\n");
	
	while( 1 )
	{
		search_run();
		
		//distance_sum = _IQtoF(DC_motor.iq17distance_sum_mm);
		//TxPrintf("d:%f\n",distance_sum);
		//TxPrintf_B("1234\n");
		//SCIx_TxString("123\n");
		//SCIx_TxString_B("Q12341001\n");
		//DELAY_US(500000);
		
	}
}

void Delay(Uint32 Cnt)
{
	while(Cnt--)
	{
		asm("		nop");
		
		asm("	nop");	
	}
}


