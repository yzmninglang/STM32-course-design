#include "sysinc.h"
#include "car_ctrl.h"

#define KEY_CTRL PAin(8)

#define BEEP PBout(11)
#define LED_RED PBout(0)

#define MIN_POWER 6.8

u8 txbuf[33]={1,7,3,4};
u8 rxbuf[33]={1,9,3,4};

u8 flag_run = 0;
u8 flag_power_alarm = 0;

void power_check(void);
void key_int(void);
void power_alarm(void);

//串口2接收中断函数
void uart2(uint16_t Res)
{		
		UART_WriteByte(HW_UART2, Res);
		if(flag_run != 0 ){	

		if(Res == '1' || Res == 1 ){
			car_forward();  
		}else if(Res == '2' || Res == 2 ){
			car_back();  
		}else if(Res == '3' || Res == 3 ){
			car_turn_left_place();  
		}else if(Res == '4' || Res == 4 ){
			car_turn_right_place();   
		}else if(Res == '5' || Res == 5 ){
			car_turn_left();   
		}else if(Res == '6' || Res == 6 ){
			car_turn_right();   
		}else{
			car_brake();  
		}
	}
}

//定时器2中断函数
void timer2(){
	static int i = 0;
	if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET){  //检查TIM3更新中断发生与否
		  TIM_ClearITPendingBit(TIM2, TIM_IT_Update);   //清除TIMx更新中断标志 
		
		  if(++i >= 10) { PCout(13)= !PCout(13); 	i= 0;	 power_check(); }
			power_alarm();
			LED_RED =  !flag_run;

	}
}

//主函数
int main(void){

	NVIC_init();
	SYSTICK_DelayInit();	    	 //延时函数初始化	  	
	
	TIM_us_Init(TIM2, 50000, TIM_IT_Update,2, ENABLE);	//初始化定时器2，定是时间500ms
 	TIMER_CallbackInstall(HW_TIMER2, timer2);	 
	
	UART_QuickInit(HW_UART1, 9600, 2, 2, ENABLE);  //初始化串口1，波特率9600
	
	UART_QuickInit(HW_UART2, 9600, 2, 2, ENABLE);  //初始化串口2，波特率9600
	UART_CallbackInstall(HW_UART2, uart2);
	 
	GPIO_QuickInit(HW_GPIOC, GPIO_Pin_13, GPIO_Mode_Out_PP);//初始化与LED连接的硬件接口

	GPIO_QuickInit(HW_GPIOB, GPIO_Pin_11, GPIO_Mode_Out_PP);//初始化与蜂鸣器连接的硬件接口
	GPIO_QuickInit(HW_GPIOB, GPIO_Pin_0,  GPIO_Mode_Out_PP);//初始化与RGB的红灯连接的硬件接口

	ADC_init(ADC_CH0);
	car_init();
	
	EXTI_QuickInit(HW_EXTIA, EXTI_Pin_8, 3, 3);
	EXTI_CallbackInstall(EXTI_Pin_8, key_int);	

	BEEP = 1;
	LED_RED = 1;
	
	while(1);
	
}

//电量检测函数
void power_check(){
	static u8 cont = 0;
	  u16 adcx;
	  float power = 0;
		adcx=Get_Adc_Average(ADC_CH0,10);
		power=(float)3*adcx*(3.3/4096);
		//printf("adc val: %f\n",power);
		
		if( power < MIN_POWER && cont > 6){
			flag_power_alarm = 1;
			cont++;
		}else{
			cont = 0;
			flag_power_alarm = 0;
		}
}

//按键中断函数
void key_int(){
		SYSTICK_DelayMs(5);
		if(KEY_CTRL == 0){
			flag_run = !flag_run;
			while(KEY_CTRL == 0);
			LED_RED =  !flag_run;
		}
}

//电量检测报警函数
void power_alarm(){
			if(flag_run && flag_power_alarm){
				BEEP = ~BEEP;
				LED_RED = ~LED_RED;
			}else{
				BEEP = 1;
			}
}
