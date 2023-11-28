/*
*************************************************************************
*硬件平台: STC89C52RC
*开发平台：keil 4.7
*版权    ：沁和智能科技
*开始时间：2019.7.11
*结束时间：2019.7.11
*作者：    张文超
*备注：    - 未经允许，用于商业用途，必究责任
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
* 函 数 名         : delay_ir
* 函数功能		   : 延时函数，i=1时，大约延时10us
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
	
	if(IRDAT==0)		//确认是否真的接收到正确的信号
	{

		IRDAT = IRIN;
		ir_key_value = 99;  ///////////////张文超
		err=1000;				//1000*10us=10ms,超过说明接收到错误的信号
		/*当两个条件都为真是循环，如果有一个条件为假的时候跳出循环，免得程序出错的时
		侯，程序死在这里*/	
		while((IRDAT==0)&&(err>0))	//等待前面9ms的低电平过去  		
		{			
			SYSTICK_DelayUs(10);
			err--;
			IRDAT = IRIN;
		}
				
		IRDAT = IRIN;
		if(IRDAT==1)			//如果正确等到9ms低电平
		{
			err=500;
			IRDAT = IRIN;
			while((IRDAT==1)&&(err>0))		 //等待4.5ms的起始高电平过去
			{
				SYSTICK_DelayUs(10);
				err--;
				IRDAT = IRIN;
			}
			for(k=0;k<4;k++)		//共有4组数据
			{				
				for(j=0;j<8;j++)	//接收一组数据
				{

					err=60;		
					IRDAT = IRIN;
					while((IRDAT==0)&&(err>0))//等待信号前面的560us低电平过去
					{
						SYSTICK_DelayUs(10);
						err--;
						IRDAT = IRIN;
					}
					err=500;
					IRDAT = IRIN;
					while((IRDAT==1)&&(err>0))	 //计算高电平的时间长度。
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
					IrValue[k]>>=1;	 //k表示第几组数据
					if(Time>=8)			//如果高电平出现大于565us，那么是1
					{
						IrValue[k]|=0x80;
					}
					Time=0;		//用完时间要重新赋值							
				}
			}
		}
		ir_key_value  = get_ir_key(IrValue[2]);
		ir_key_value_last = ir_key_value;
	}
} 

/*******************************************************************************
* 函数名         : ReadIr()
* 函数功能		   : 读取红外数值的中断函数
* 输入           : 无
* 输出         	 : 无
*******************************************************************************/

void ir_remote_int() 
{
	//IrValue[2] = 99;
	ir_decode();
}

/*******************************************************************************
* 函数名         : IrInit()
* 函数功能		   : 初始化红外线接收
* 输入           : 无
* 输出         	 : 无
*******************************************************************************/
void IrInit()
{
	EXTI_QuickInit(HW_EXTIA, EXTI_Pin_1, 0, 0);
	EXTI_CallbackInstall( EXTI_Pin_1, ir_remote_int);
	GPIO_QuickInit(HW_GPIOA, GPIO_Pin_1, GPIO_Mode_IPU);//初始化与蜂鸣器连接的硬件接口
}



#endif
