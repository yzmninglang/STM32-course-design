/*
*************************************************************************
*Ӳ��ƽ̨: STC89C52RC
*����ƽ̨��keil 4.7
*��Ȩ    ���ߺ����ܿƼ�
*��ʼʱ�䣺2019.7.11
*����ʱ�䣺2019.7.11
*���ߣ�    ���ĳ�
*��ע��    - δ������������ҵ��;���ؾ�����
*************************************************************************
*/

#ifndef _IR_REMOTE_H_
#define _IR_REMOTE_H_

#include "sysinc.h"

#define IRIN PAin(1)

unsigned char IrValue[6];
unsigned char Time;
unsigned char ir_key_value = 99;
unsigned char ir_key_value_last = 99;

/*******************************************************************************
* �� �� ��         : delay_ir
* ��������		   : ��ʱ������i=1ʱ����Լ��ʱ10us
*******************************************************************************/
void delay_ir(unsigned int i){
	while(i--);	
}


unsigned char get_ir_key(unsigned char ir_v){
	unsigned char ir_key = 0;
	switch( ir_v ){
		case 22 : ir_key = 0; break;
		case 12 : ir_key = 1; break;
		case 24 : ir_key = 2; break;
		case 94 : ir_key = 3; break;
		case  8 : ir_key = 4; break;
		case 28 : ir_key = 5; break;
		case 90 : ir_key = 6; break;
		case 66 : ir_key = 7; break;
		case 82 : ir_key = 8; break;
		case 74 : ir_key = 9; break;
		case 13 : ir_key = 10; break;
		case 21 : ir_key = 11; break;
		case 25 : ir_key = 12; break;
		case 64 : ir_key = 13; break;
		case  7 : ir_key = 14; break;
		case  9 : ir_key = 15; break;
	}
	return ir_key;
}


void ir_decode(){
	unsigned char j,k;
	unsigned int err;
	int IRDAT = IRIN;
	
	Time=0;					 
	SYSTICK_DelayUs(700);	//7ms
	
	if(IRDAT==0)		//ȷ���Ƿ���Ľ��յ���ȷ���ź�
	{

		IRDAT = IRIN;
		ir_key_value = 99;  ///////////////���ĳ�
		err=1000;				//1000*10us=10ms,����˵�����յ�������ź�
		/*������������Ϊ����ѭ���������һ������Ϊ�ٵ�ʱ������ѭ������ó�������ʱ
		�������������*/	
		while((IRDAT==0)&&(err>0))	//�ȴ�ǰ��9ms�ĵ͵�ƽ��ȥ  		
		{			
			SYSTICK_DelayUs(10);
			err--;
			IRDAT = IRIN;
		}
				
		IRDAT = IRIN;
		if(IRDAT==1)			//�����ȷ�ȵ�9ms�͵�ƽ
		{
			err=500;
			IRDAT = IRIN;
			while((IRDAT==1)&&(err>0))		 //�ȴ�4.5ms����ʼ�ߵ�ƽ��ȥ
			{
				SYSTICK_DelayUs(10);
				err--;
				IRDAT = IRIN;
			}
			for(k=0;k<4;k++)		//����4������
			{				
				for(j=0;j<8;j++)	//����һ������
				{

					err=60;		
					IRDAT = IRIN;
					while((IRDAT==0)&&(err>0))//�ȴ��ź�ǰ���560us�͵�ƽ��ȥ
					{
						SYSTICK_DelayUs(10);
						err--;
						IRDAT = IRIN;
					}
					err=500;
					IRDAT = IRIN;
					while((IRDAT==1)&&(err>0))	 //����ߵ�ƽ��ʱ�䳤�ȡ�
					{
						IRDAT = IRIN;
						SYSTICK_DelayUs(100);
						Time++;
						err--;
						if(Time>30)
						{
							ir_key_value = ir_key_value_last;
							return;
						}
					}
					IrValue[k]>>=1;	 //k��ʾ�ڼ�������
					if(Time>=8)			//����ߵ�ƽ���ִ���565us����ô��1
					{
						IrValue[k]|=0x80;
					}
					Time=0;		//����ʱ��Ҫ���¸�ֵ							
				}
			}
		}
		ir_key_value  = get_ir_key(IrValue[2]);
		ir_key_value_last = ir_key_value;
	}
} 

/*******************************************************************************
* ������         : ReadIr()
* ��������		   : ��ȡ������ֵ���жϺ���
* ����           : ��
* ���         	 : ��
*******************************************************************************/

void ir_remote_int() 
{
	//IrValue[2] = 99;
	ir_decode();
}

/*******************************************************************************
* ������         : IrInit()
* ��������		   : ��ʼ�������߽���
* ����           : ��
* ���         	 : ��
*******************************************************************************/
void IrInit()
{
	EXTI_QuickInit(HW_EXTIA, EXTI_Pin_1, 0, 0);
	EXTI_CallbackInstall( EXTI_Pin_1, ir_remote_int);
	GPIO_QuickInit(HW_GPIOA, GPIO_Pin_1, GPIO_Mode_IPU);//��ʼ������������ӵ�Ӳ���ӿ�
}



#endif
