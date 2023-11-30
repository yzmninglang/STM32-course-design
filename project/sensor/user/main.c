#include "sysinc.h"
#include "car_ctrl.h"
#include "hc_sr04.h"
#include "valuepack.h"
#include "steer_sg90.h"

#define KEY_CTRL PAin(8)

#define BEEP PBout(11)

//RGB灯RED
#define LED_RED PBout(0)

#define MIN_POWER 8.0
int LeftSensorValue1 = 0;
int LeftSensorValue2 = 0;
int RightSensorValue1 = 0;
int RightSensorValue2 = 0;


int flag_black=0;
int flag_beep=0;
struct Obstacle
{
	u8 right_ultrasonic;
	u8 left_ultrasonic;
	u8 forword_ultrasonic;
	u8 left_infrared;
	u8 right_infrared;
};

u8 txbuf[33]={1,7,3,4};
u8 rxbuf[33]={1,9,3,4};
char flag_run_char='s';
u8 flag_run = 0;
u8 flag_power_alarm = 0;
void TraceTail(void);
int mode=0;
void power_check(void);
void key_int(void);
void power_alarm(void);
void avoid_func_hw(void);
void control_by_flag(char Res);
void Obstacle_avoidance(int dis,int time_delay);
void Obstacle_Avoid();
void Trace_No_obstacle_avoidance_NONVIC();


struct Obstacle Avoid_Path_Detecet(int dis);
//串口2接收中断函数
void uart2(uint16_t Res)
{		
		UART_WriteByte(HW_UART2, Res);
		printf("%c",Res);
	  if(Res=='6')
		{
			flag_run=~flag_run;
		}
		// control_by_flag(Res)
		flag_run_char=Res;

}
void control_by_flag(char Res)
{
		// if(flag_run != 0 ){			
		// 	flag_run_char=Res;
		if(Res == 'U'  ){
			car_forward();
		}else if(Res == 'D'){
			car_back();  
		}else if(Res == 'L'){
			car_turn_left_place();  
		}else if(Res == 'R'){
			car_turn_right_place();   
		}else if(Res == '1'){
			if(flag_beep)
			{TIM_Cmd(TIM4,DISABLE);}else{
				TIM_Cmd(TIM4,ENABLE);
			}flag_beep=1-flag_beep;  
		}else if(Res == '3'){
			car_turn_right();   
		}else if(Res == 'O'){
			car_brake();  
//			flag_run=~flag_run;
		}else if(Res == '2'){
			//寻迹
			Trace_No_obstacle_avoidance_NONVIC();
		}else if(Res == '4')
		{
			Obstacle_avoidance(2500,500);
		}
	// }
}

//定时器2中断函数
void timer2(){
	static int i = 0;
	if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET){  //检查TIM3更新中断发生与否
		  TIM_ClearITPendingBit(TIM2, TIM_IT_Update);   //清除TIMx更新中断标志 
			if(++i >= 10) {  	i= 0;	 power_check(); }
			power_alarm();
			LED_RED =  !LED_RED;

	}
}

struct Obstacle Avoid_Path_Detecet(int dis)
{
	// 左右两边前方红外避障

	// 先规定前后左右都没有障碍物
		struct Obstacle Obstacle_detection;
		Obstacle_detection.forword_ultrasonic=0;
		Obstacle_detection.right_ultrasonic=0;
		Obstacle_detection.left_ultrasonic=0;
		Obstacle_detection.left_infrared=0;
		Obstacle_detection.right_infrared=0;


		int temp;
		int LeftSensorValue = 0;
		int RightSensorValue = 0;
		// 超声波测距
		int distance = 0;
		distance = get_hc_sr04_value_by_median_filter();
		LeftSensorValue  = PAin(4);
		RightSensorValue = PAin(6);
		// 红外寻迹
		printf("right:%d left:%d dis:%d\r\n",RightSensorValue,LeftSensorValue,distance);
		steer_sg90_run_to_angle( TIM1, PWM_CH4, 90);
		SYSTICK_DelayMs(2000);
		distance = get_hc_sr04_value_by_median_filter();
		SYSTICK_DelayMs(100);
		if(1)
		{
			// 如何超声波的测量距离小于规定的距离和，或者左右两边的传感器有反应
			// 则表示前方有障碍物
			if(distance<3000 || LeftSensorValue==0||RightSensorValue==0)
			{
					car_brake();
					Obstacle_detection.left_infrared=1-LeftSensorValue;
					Obstacle_detection.right_infrared=1-RightSensorValue;
					//右边一定有障碍
					steer_sg90_run_to_angle( TIM1, PWM_CH4, 20);
					SYSTICK_DelayMs(2000);
					temp=get_hc_sr04_value_by_median_filter();
					if(temp<dis)
					{Obstacle_detection.right_ultrasonic=1;
					printf("r:%d\r\n",temp);}
					SYSTICK_DelayMs(100);

					//前方一定有障碍
					steer_sg90_run_to_angle( TIM1, PWM_CH4, 90);
					SYSTICK_DelayMs(2000);
					temp=get_hc_sr04_value_by_median_filter();
					if(temp<dis)
					{Obstacle_detection.forword_ultrasonic=1;
					printf("f:%d\r\n",temp);}
					SYSTICK_DelayMs(100);

					// 左边一定有障碍
					steer_sg90_run_to_angle( TIM1, PWM_CH4, 160);
					SYSTICK_DelayMs(2000);
					temp=get_hc_sr04_value_by_median_filter();
					if(temp<dis)
					{Obstacle_detection.left_ultrasonic=1;
					printf("l:%d\r\n",temp);
					}
					SYSTICK_DelayMs(100);
					steer_sg90_run_to_angle( TIM1, PWM_CH4, 90);
			}
			else
			{
				car_forward();
			}

		}
		printf("f:%d,r:%d,l%d\r\n",Obstacle_detection.forword_ultrasonic,Obstacle_detection.right_ultrasonic,Obstacle_detection.left_ultrasonic);
	return Obstacle_detection;
	
}

void Left_change()
{
	EXTI_ClearITPendingBit(EXTI_Line14);
	if (flag_run)
	{
			car_set_motor_speed(2800, 2800);
			car_turn_right_place();
			printf("Left_change\r\n");
			// flag_change=0;
			while (PCin(13) == 0 || PBin(10) == 0)
			{
				printf("Left_change\r\n");
			};
			car_brake();
	}
}

void Right_change()
{
	EXTI_ClearITPendingBit(EXTI_Line15);
	if (flag_run)
	{
			car_set_motor_speed(2800, 2800);
			car_turn_left_place();
			printf("Right_change\r\n");
			// flag_change=0;
			while (PCin(13) == 0 || PBin(10) == 0)
			{
				printf("Right_change\r\n");
			};
			car_brake();
	}
}

// Trace：利用中间两个红外对管进行寻迹，但是实际效果并不好，不采用！！！！
void Trace()
{

		if(flag_run){
			
				LeftSensorValue1  = PCin(14);
				RightSensorValue1 = PCin(15);
				LeftSensorValue2  = PCin(13);
				RightSensorValue2 = PBin(10);
				
				if (LeftSensorValue2 == 1 && RightSensorValue2 == 1 ) { 
					car_forward();        
				} 
				else if (LeftSensorValue2 == 1 && RightSensorValue2 == 0 ) {
					car_turn_right_place(); 
				}
				else if (LeftSensorValue2 == 0 && RightSensorValue2 == 1) {
					car_turn_left_place();  
				}
				else {
					car_forward(); 
				}
		
		}else{
			car_brake();
		}

}


void timer3()
{
	TIM_ClearITPendingBit(TIM4,TIM_IT_Update);
	BEEP = ~BEEP;
	printf("Tim4\r\n");
	
}
	
void change_flag_black()
{

	flag_black=1;
}

void Trace_Enable()
{
	// 开启外部中断
	EXTI_QuickInit(HW_EXTIC, EXTI_Pin_13, 3, 3,0,ENABLE);
	EXTI_QuickInit(HW_EXTIB, EXTI_Pin_10, 3, 3,0,ENABLE);

	EXTI_CallbackInstall(EXTI_Pin_13,  change_flag_black);
	EXTI_CallbackInstall(EXTI_Pin_10, change_flag_black);
}

void Trace_Disable()
{
	// 开启外部中断
	EXTI_QuickInit(HW_EXTIC, EXTI_Pin_14, 3, 3,0,DISABLE);
	EXTI_QuickInit(HW_EXTIC, EXTI_Pin_15, 3, 3,0,DISABLE);

	EXTI_CallbackInstall(EXTI_Pin_14,  Left_change);
	EXTI_CallbackInstall(EXTI_Pin_15, Right_change);
}


// 1.先开始Trace_Enable;
// 2.开启Trace_No_obstacle_avoidance;
// 3.有必要的时候请关闭Trace_Disable;
void Trace_No_obstacle_avoidance()
{
		if (flag_run)
		{
			car_set_motor_speed(3500,3500);
			car_forward();
		}
}


void Trace_No_obstacle_avoidance_NONVIC()
{
	// 先定义各个检测器
	RightSensorValue1  = PCin(14);
	LeftSensorValue1 = PCin(15);
	RightSensorValue2  = PCin(13);
	LeftSensorValue2 = PBin(10);
	if(flag_run)
	{
		if(LeftSensorValue2==1 && RightSensorValue2==1/*||(LeftSensorValue2==0 && RightSensorValue2==0)*/)
		{
			car_set_motor_speed(3000,3000);
			car_forward();		

		}else if(LeftSensorValue2==1 && RightSensorValue2==0)
		{
			// 左转
			car_set_motor_speed(4000,1000);
			car_forward();

		}else if(LeftSensorValue2==0 && RightSensorValue2==1)
		{	// 右转
			car_set_motor_speed(1000,4000);
			car_forward();

		}else if(LeftSensorValue1==1 && LeftSensorValue2==0 && RightSensorValue1==0)
		{	
			car_set_motor_speed(5000,5000);
			car_turn_left_place();
		}else if(LeftSensorValue1==0 && RightSensorValue2==0 && RightSensorValue1==1)
		{	
			car_set_motor_speed(5000,5000);
			car_turn_right_place();
		}else if(LeftSensorValue2==0 && RightSensorValue2==0 &&LeftSensorValue1==0 && RightSensorValue1==0){
			//car_set_motor_speed(3000,3000);
			//car_brake();
		}
	}

}


//void Trace_No_obstacle_avoidance_NONVIC()
//{
//	// 先定义各个检测器
//	RightSensorValue1  = PCin(14);
//	LeftSensorValue1 = PCin(15);
//	RightSensorValue2  = PCin(13);
//	LeftSensorValue2 = PBin(10);
//	if(flag_run)
//	{
//		if(LeftSensorValue2==1 && RightSensorValue2==1/*||(LeftSensorValue2==0 && RightSensorValue2==0)*/)
//		{
//			car_set_motor_speed(3000,3000);
//			car_forward();		

//		}else if(LeftSensorValue2==1 && RightSensorValue2==0)
//		{
//			// 左转
//			car_set_motor_speed(4000,1000);
//			car_forward();

//		}else if(LeftSensorValue2==0 && RightSensorValue2==1)
//		{	// 右转
//			car_set_motor_speed(1000,4000);
//			car_forward();

//		}else if(LeftSensorValue1==1 && LeftSensorValue2==0 && RightSensorValue1==0)
//		{	
//			car_set_motor_speed(5000,5000);
//			car_turn_left_place();
//		}else if(LeftSensorValue1==0 && RightSensorValue2==0 && RightSensorValue1==1)
//		{	
//			car_set_motor_speed(5000,5000);
//			car_turn_right_place();
//		}else if(LeftSensorValue2==0 && RightSensorValue2==0 &&LeftSensorValue1==0 && RightSensorValue1==0){
//			//car_set_motor_speed(3000,3000);
//			//car_brake();
//		}
//	}

//}


//Trace_obstacle_avoidance_NONVIC()
//{

//	// 先定义各个检测器
//	LeftSensorValue  = PAin(4);
//	RightSensorValue = PAin(6);
//	RightSensorValue1  = PCin(14);
//	LeftSensorValue1 = PCin(15);
//	RightSensorValue2  = PCin(13);
//	LeftSensorValue2 = PBin(10);
//	if(flag_run)
//	{
//		
//		if(LeftSensorValue2==1 && RightSensorValue2==1/*||(LeftSensorValue2==0 && RightSensorValue2==0)*/)
//		{
//			car_set_motor_speed(3000,3000);
//			car_forward();		

//		}else if(LeftSensorValue2==1 && RightSensorValue2==0)
//		{
//			// 左转
//			car_set_motor_speed(4000,1000);
//			car_forward();

//		}else if(LeftSensorValue2==0 && RightSensorValue2==1)
//		{	// 右转
//			car_set_motor_speed(1000,4000);
//			car_forward();

//		}else if(LeftSensorValue1==1 && LeftSensorValue2==0 && RightSensorValue1==0)
//		{	
//			car_set_motor_speed(5000,5000);
//			car_turn_left_place();
//		}else if(LeftSensorValue1==0 && RightSensorValue2==0 && RightSensorValue1==1)
//		{	
//			car_set_motor_speed(5000,5000);
//			car_turn_right_place();
//		}else if(LeftSensorValue2==0 && RightSensorValue2==0 &&LeftSensorValue1==0 && RightSensorValue1==0){
//			//car_set_motor_speed(3000,3000);
//			//car_brake();
//		}
//	}


//}
//主函数
// 资源分配：TIM1作为舵机的PWM波，TIM2作为
// 
int main(void){

	NVIC_init();
	SYSTICK_DelayInit();	    	 //延时函数初始化	  	
	
	TIM_us_Init(TIM2, 500000, TIM_IT_Update,2, ENABLE);	//初始化定时器2，定是时间500ms
 	TIMER_CallbackInstall(HW_TIMER2, timer2);	
//	TIM_us_Init(TIM4, 50, TIM_IT_Update,2, ENABLE);
//	TIM_Cmd(TIM4, DISABLE); 
	
//	TIMER_CallbackInstall(HW_TIMER4, timer3);
//	TIM_Cmd(TIM1,ENABLE);	
	//串口1：下载串口，调试串口
	UART_QuickInit(HW_UART1, 9600, 2, 2, ENABLE);  //初始化串口1，波特率9600
	//串口2:蓝牙串口
	UART_QuickInit(HW_UART2, 9600, 2, 2, ENABLE);  //初始化串口2，波特率9600
	UART_CallbackInstall(HW_UART2, uart2);
	
	GPIO_QuickInit(HW_GPIOA, GPIO_Pin_8, GPIO_Mode_IPU);//初始化与按键相连的引脚
	//初始化红外寻迹模块
	GPIO_QuickInit(HW_GPIOC, GPIO_Pin_13, GPIO_Mode_IPU);//初始化与模块相连的引脚
	GPIO_QuickInit(HW_GPIOC, GPIO_Pin_14, GPIO_Mode_IPU);//初始化与模块相连的引脚
	GPIO_QuickInit(HW_GPIOC, GPIO_Pin_15, GPIO_Mode_IPU);//初始化与模块相连的引脚
	GPIO_QuickInit(HW_GPIOB, GPIO_Pin_10, GPIO_Mode_IPU);//初始化与模块相连的引脚

	
//	GPIO_QuickInit(HW_GPIOC, GPIO_Pin_13, GPIO_Mode_Out_PP);//初始化与LED连接的硬件接口
	GPIO_QuickInit(HW_GPIOB, GPIO_Pin_11, GPIO_Mode_Out_PP);//初始化与蜂鸣器连接的硬件接口
	GPIO_QuickInit(HW_GPIOB, GPIO_Pin_0,  GPIO_Mode_Out_PP);//初始化与RGB的红灯连接的硬件接口
	hc_sr04_init();       //超声波初始化
	ADC_init(ADC_CH0);
	car_init();


	// 初始化红外
	GPIO_QuickInit(HW_GPIOA, GPIO_Pin_4, GPIO_Mode_IPU);
	GPIO_QuickInit(HW_GPIOA, GPIO_Pin_6, GPIO_Mode_IPU);

	//初始化PA8为外部触发中断：按键
	EXTI_QuickInit(HW_EXTIA, EXTI_Pin_8, 3, 3,1,ENABLE);

	EXTI_CallbackInstall(EXTI_Pin_8, key_int);	
	//TIM1设置为成功
	steer_sg90_init(TIM1, PWM_CH4);
	Trace_Enable();
	// steer_sg90_run_to_angle( TIM1, PWM_CH4, 90);
	// BEEP = 1;
	LED_RED = 1;
	car_set_motor_speed(3500, 3500);



	// int trun_time=200;
	while(1)
	{
//		if(flag_run)
//		{
		Trace_No_obstacle_avoidance_NONVIC();
		// avoid_func_hw();

//			printf("main\r\n");
//			SYSTICK_DelayUs(100);
//			TIM_cmd(TIM4,ENABLE);
			// BEEP=~BEEP;
			// TIM_Cmd(TIM1,); 



			// TIM_Cmd(TIM3, ENABLE); 

			//无避障的自动寻迹
			// Trace_No_obstacle_avoidance_NONVIC();
			
			
			// 有避障的自动寻迹
			// Obstacle_avoidance(2500,500);
//		}
	// (2000,50);
	// Trace_No_obstacle_avoidance_NONVIC();
	// 	// Trace();
	// 	printf("hell0");

	// 	RightSensorValue1 = PCin(15);
	// 	printf("%d",RightSensorValue1);
	// 	SYSTICK_DelayMs(100);
	// 	car_set_motor_speed(3500,3500);
	// // car_turn_left();
	// // car_turn_left_place();
	// if(flag_run){
			
	// 		LeftSensorValue1  = PCin(14);
	// 		RightSensorValue1 = PCin(15);

	// 		LeftSensorValue2  = PCin(13);
	// 		RightSensorValue2 = PBin(10);
			
	// 		if (LeftSensorValue1 == 1 && RightSensorValue1 == 1  ) { 
	// 			// car_back();
	// 			car_set_motor_speed(3500,3500);
	// 			car_forward();        
	// 		} 
	// 		else if (LeftSensorValue1 == 1 && RightSensorValue1 == 0 ) {
	// 			// car_back();
	// 			// SYSTICK_DelayMs(100);
	// 			car_set_motor_speed(6000,2000);
	// 			car_turn_right_place(); 
	// 			SYSTICK_DelayMs(trun_time);
	// 		}
	// 		else if (LeftSensorValue1 == 0 && RightSensorValue1 == 1) {
	// 			// car_back();
	// 			// SYSTICK_DelayMs(100);
	// 			car_set_motor_speed(2000,6000);
	// 			car_turn_left_place(); 
	// 			SYSTICK_DelayMs(trun_time);
	// 		}
	// 		else {
	// 			car_set_motor_speed(3500,3500);
	// 			car_forward();
	// 			// car_set_motor_speed(4000,4000);
	// 			// car_turn_left_place(); 
	// 		}
	// 		printf("ir is %d, %d, %d, %d \n",LeftSensorValue1, LeftSensorValue2, RightSensorValue1, RightSensorValue2 );

	
	// }else{
	// 	car_brake();
	// }

	// }
	
}
}



//电量检测函数
void power_check(){
	static u8 cont = 0;
	  u16 adcx;
	  float power = 0;
		adcx=Get_Adc_Average(ADC_CH0,10);
		power=(float)3*adcx*(3.3/4096);
		//printf("adc val: %f\n",power);
		
		if( power < MIN_POWER && cont >6){
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
			TIM_Cmd(TIM3, ENABLE); 
		}
}

//电量检测报警函数
void power_alarm(){
			if(flag_power_alarm){			
					LED_RED = ~LED_RED;
				// TIM_Cmd(TIM3,ENABLE);
			}else{
				// BEEP = 1;
				TIM_Cmd(TIM3,ENABLE);
			}
}


int flag_Obstacle_left=0;
int flag_Obstacle_right=0;
void TraceTail()
{
			car_set_motor_speed(3000, 3000);
			int LeftSensorValue1 = 0;
			int LeftSensorValue2 = 0;
			int RightSensorValue1 = 0;
			int RightSensorValue2 = 0;
			int distance = 0;
			int LeftSensorValue = 0;
			int RightSensorValue = 0;

			if (flag_run)
			{

				RightSensorValue1 = PCin(14);
				LeftSensorValue1 = PCin(15);
				LeftSensorValue = PAin(4);
				RightSensorValue = PAin(6);
				RightSensorValue2 = PCin(13);
				LeftSensorValue2 = PBin(10);
				distance = get_hc_sr04_value_by_median_filter();
				printf("ir is %d, %d, %d, %d \r\n", LeftSensorValue1, LeftSensorValue2, RightSensorValue1, RightSensorValue2);
				if (RightSensorValue==0||RightSensorValue==0||distance<=2000)
				{
					flag_black=0;
					if (LeftSensorValue == 1 && RightSensorValue == 0)
					{
							// 右边有障碍物
							flag_Obstacle_right=1;
							car_set_motor_speed(5000, 5000);
							car_turn_left_place();
					}
					else if (RightSensorValue == 1 && LeftSensorValue == 0)
					{
							// 左边有障碍物
							flag_Obstacle_left=1;
							car_set_motor_speed(5000, 5000);
							car_turn_right_place();
							// SYSTICK_DelayMs(10);
							
					}
					else
					{
						car_set_motor_speed(5000, 5000);
						car_turn_right_place();

					}
				}
				else if (RightSensorValue == 1 && LeftSensorValue == 1 && (flag_Obstacle_right == 1 || flag_Obstacle_left == 1) && flag_black!=1)
				{
					if (flag_Obstacle_right == 1)
					{
						car_set_motor_speed(4000, 1000);
						car_forward();
					}
					if (flag_Obstacle_left == 1)
					{
						car_set_motor_speed(1000, 4000);
						car_forward();
					}
				}
				else
				{
					flag_Obstacle_left=flag_Obstacle_right=0;
					if (LeftSensorValue2 == 1 && RightSensorValue2 == 1 /*||(LeftSensorValue2==0 && RightSensorValue2==0)*/)
					{
							car_set_motor_speed(3000, 3000);
							car_forward();
					}
					else if (LeftSensorValue2 == 1 && RightSensorValue2 == 0)
					{
							// 左转
							car_set_motor_speed(4000, 1000);
							car_forward();
					}
					else if (LeftSensorValue2 == 0 && RightSensorValue2 == 1)
					{ // 右转
							car_set_motor_speed(1000, 4000);
							car_forward();
					}
					else if (LeftSensorValue1 == 1 && LeftSensorValue2 == 0 && RightSensorValue1 == 0)
					{
							car_set_motor_speed(5000, 5000);
							car_turn_left_place();
					}
					else if (LeftSensorValue1 == 0 && RightSensorValue2 == 0 && RightSensorValue1 == 1)
					{
							car_set_motor_speed(5000, 5000);
							car_turn_right_place();
					}
					else if (LeftSensorValue2 == 0 && RightSensorValue2 == 0 && LeftSensorValue1 == 0 && RightSensorValue1 == 0)
					{
							// car_set_motor_speed(3000,3000);
							// car_brake();
					}
				}
			}
			else
				{
			car_brake();
				}
			
}

//避障程序
void avoid_func_hw()
{
		int LeftSensorValue = 0;
		int RightSensorValue = 0;
		int distance = 0;
//		SYSTICK_DelayMs(20);
		distance = get_hc_sr04_value_by_median_filter();
		printf("distance:%d\r\n",distance);
		if(flag_run){
			
				LeftSensorValue  = PAin(4);
				RightSensorValue = PAin(6);
				printf("%d:%d:%d:%c\r\n",LeftSensorValue,RightSensorValue,distance,flag_run_char);
				if ((LeftSensorValue == 1 && RightSensorValue == 1 && distance>2000)||flag_run_char=='D'||flag_run_char=='O'||flag_run_char=='2') { 
					//左边有障碍物
					car_set_motor_speed(3500, 3500);
//					car_forward();
							//printf(" continue\r\n");
							control_by_flag(flag_run_char);
				}else
				{
				Obstacle_avoidance(2000,200);
				}
}else{

		car_brake();
}
		}

void Obstacle_avoidance(int dis,int delay_time)
{
		int LeftSensorValue = PAin(4);
		int RightSensorValue = PAin(6);
		int distance = 0;
		distance = get_hc_sr04_value_by_median_filter();
		printf("%d",distance);
		if (distance >= dis || (LeftSensorValue==1&& RightSensorValue==1))
		{
		Trace_No_obstacle_avoidance_NONVIC();
		}
		else
		{
			// car_brake();
			LED_RED=~LED_RED;
			if (RightSensorValue == 1 && LeftSensorValue == 0)
			{
					// 左边有障碍物
					car_set_motor_speed(5000, 5000);
					car_back();
					SYSTICK_DelayMs(delay_time/4);
					car_set_motor_speed(5000, 5000);
					car_turn_right_place();
					SYSTICK_DelayMs(delay_time);
			}
			else if (LeftSensorValue == 1 && RightSensorValue == 0)
			{
					// 右边有障碍物
					car_set_motor_speed(5000, 5000);
					car_back();
					SYSTICK_DelayMs(delay_time/4);
					car_set_motor_speed(5000, 5000);
					car_turn_left_place();
					SYSTICK_DelayMs(delay_time);

			}
			else
			{
					// 全部有
					car_set_motor_speed(5000, 5000);
					car_back();
					SYSTICK_DelayMs(delay_time/4);
					car_set_motor_speed(5000, 5000);
					car_turn_left_place();
					SYSTICK_DelayMs(delay_time);
			}
		}
}



void Obstacle_Avoid()
{
			// int LeftSensorValue = PAin(4);
			// int RightSensorValue = PAin(6);
			// int distance = 0;
			// distance = get_hc_sr04_value_by_median_filter();
			// printf("%d", distance);
			// if (distance >= dis && LeftSensorValue == 0)
			// {
			// 	Trace_No_obstacle_avoidance_NONVIC();
			// }
			// else if (LeftSensorValue == 1 && RightSensorValue == 0)
			// {
			// 	// 右边有障碍物
			// 	car_set_motor_speed(3000, 3000);
			// 	car_back();
			// 	SYSTICK_DelayMs(delay_time);
			// 	car_set_motor_speed(3500, 2500);
			// 	car_turn_left_place();
			// 	SYSTICK_DelayMs(delay_time);
			// }
			// else if (RightSensorValue == 1 && LeftSensorValue == 0)
			// {
			// 	// 左边有障碍物
			// 	car_set_motor_speed(3000, 3000);
			// 	car_back();
			// 	SYSTICK_DelayMs(delay_time);
			// 	car_set_motor_speed(2500, 3500);
			// 	car_turn_right_place();
			// 	SYSTICK_DelayMs(delay_time);
			// }
			// else
			// {
			// 	// 全部有
			// 	car_set_motor_speed(3000, 3000);
			// 	car_back();
			// 	SYSTICK_DelayMs(delay_time);
			// 	car_set_motor_speed(2500, 3500);
			// 	car_turn_left_place();
			// 	SYSTICK_DelayMs(delay_time);
			// }
}











// 测试扭头的代码 while循环中
// struct Obstacle Obstacle_decide=Avoid_Path_Detecet(2500);
// 		if(Obstacle_decide.right_ultrasonic==1 && Obstacle_decide.left_ultrasonic==1)
// 		{
// 			printf("R and L can't go\r\n");
// 			car_back();
// 			SYSTICK_DelayMs(100);
// 			car_turn_left();
// 			SYSTICK_DelayMs(100);
// 			// 左右都不行
// 			// 返回一段距离，再判断

// 		}else if(Obstacle_decide.left_ultrasonic==1)
// 		{
// 			car_back();
// 			SYSTICK_DelayMs(100);
// 			printf("L can't go\r\n");
// 			car_turn_right_place();
// 			SYSTICK_DelayMs(100);
// 			// 不能左转，右转

// 		}else if (Obstacle_decide.right_ultrasonic==1)
// 		{
// 			car_back();
// 			SYSTICK_DelayMs(100);
// 			printf("R can't go\r\n");
// 			car_turn_left_place();
// 			SYSTICK_DelayMs(100);
// 			// 不能右转，左转
// 		}else if(Obstacle_decide.right_infrared==1)
// 		{
// 			car_back();
// 			SYSTICK_DelayMs(100);
// 			printf("R can't go\r\n");
// 			car_turn_left_place();
// 			SYSTICK_DelayMs(100);
// 			// 不能右转，左转

// 		}else if(Obstacle_decide.left_infrared==1)
// 		{
// 			car_back();
// 			SYSTICK_DelayMs(100);
// 			printf("L can't go \r\n");
// 			//不能左转，右转
// 			car_turn_right_place();
// 			SYSTICK_DelayMs(100);

// 		}else
// 		{
// 			car_forward();
// 		}
