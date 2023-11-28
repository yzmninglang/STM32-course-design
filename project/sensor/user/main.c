#include "sysinc.h"
#include "car_ctrl.h"
#include "hc_sr04.h"
#include "valuepack.h"
#include "steer_sg90.h"

#define KEY_CTRL PAin(8)

#define BEEP PBout(11)

//RGB��RED
#define LED_RED PBout(0)

#define MIN_POWER 6.8



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

struct Obstacle Avoid_Path_Detecet(int dis);
//����2�����жϺ���
void uart2(uint16_t Res)
{		
		UART_WriteByte(HW_UART2, Res);
		printf("%c",Res);
	  if(Res=='6')
		{
			flag_run=~flag_run;
		}
		control_by_flag(Res);
}
void control_by_flag(char Res)
{
		if(flag_run != 0 ){	
			flag_run_char=Res;
		if(Res == 'U'  ){
			car_forward();
		}else if(Res == 'D'){
			car_back();  
		}else if(Res == 'L'){
			car_turn_left_place();  
		}else if(Res == 'R'){
			car_turn_right_place();   
		}else if(Res == '1'){
			car_turn_left();   
		}else if(Res == '3'){
			car_turn_right();   
		}else if(Res == 'O'){
			car_brake();  
//			flag_run=~flag_run;
		}else if(Res == '2'|| mode==1){
			//Ѱ��
			TraceTail();
		}
	}
}
//��ʱ��2�жϺ���
void timer2(){
	static int i = 0;
	if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET){  //���TIM3�����жϷ������
		  TIM_ClearITPendingBit(TIM2, TIM_IT_Update);   //���TIMx�����жϱ�־ 
			if(++i >= 10) { PCout(13)= !PCout(13); 	i= 0;	 power_check(); }
			power_alarm();
			LED_RED =  !flag_run;

	}
}

struct Obstacle Avoid_Path_Detecet(int dis)
{
	// ��������ǰ���������

	// �ȹ涨ǰ�����Ҷ�û���ϰ���
		struct Obstacle Obstacle_detection;
		Obstacle_detection.forword_ultrasonic=0;
		Obstacle_detection.right_ultrasonic=0;
		Obstacle_detection.left_ultrasonic=0;
		Obstacle_detection.left_infrared=0;
		Obstacle_detection.right_infrared=0;


		int temp;
		int LeftSensorValue = 0;
		int RightSensorValue = 0;
		// ���������
		int distance = 0;
		distance = get_hc_sr04_value_by_median_filter();
		LeftSensorValue  = PAin(4);
		RightSensorValue = PAin(6);
		// ����Ѱ��
		printf("right:%d left:%d dis:%d\r\n",RightSensorValue,LeftSensorValue,distance);
		steer_sg90_run_to_angle( TIM1, PWM_CH4, 90);
		SYSTICK_DelayMs(2000);
		distance = get_hc_sr04_value_by_median_filter();
		SYSTICK_DelayMs(100);
		if(1)
		{
			// ��γ������Ĳ�������С�ڹ涨�ľ���ͣ������������ߵĴ������з�Ӧ
			// ���ʾǰ�����ϰ���
			if(distance<3000 || LeftSensorValue==0||RightSensorValue==0)
			{
					car_brake();
					Obstacle_detection.left_infrared=1-LeftSensorValue;
					Obstacle_detection.right_infrared=1-RightSensorValue;
					//�ұ�һ�����ϰ�
					steer_sg90_run_to_angle( TIM1, PWM_CH4, 20);
					SYSTICK_DelayMs(2000);
					temp=get_hc_sr04_value_by_median_filter();
					if(temp<dis)
					{Obstacle_detection.right_ultrasonic=1;
					printf("r:%d\r\n",temp);}
					SYSTICK_DelayMs(100);

					//ǰ��һ�����ϰ�
					steer_sg90_run_to_angle( TIM1, PWM_CH4, 90);
					SYSTICK_DelayMs(2000);
					temp=get_hc_sr04_value_by_median_filter();
					if(temp<dis)
					{Obstacle_detection.forword_ultrasonic=1;
					printf("f:%d\r\n",temp);}
					SYSTICK_DelayMs(100);

					// ���һ�����ϰ�
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



void timer1()
{
	BEEP = ~BEEP;
	TIM_ClearITPendingBit(TIM3,TIM_IT_Update);
}
	
//������
int main(void){

	NVIC_init();
	SYSTICK_DelayInit();	    	 //��ʱ������ʼ��	  	
	
	TIM_us_Init(TIM2, 500000, TIM_IT_Update,2, ENABLE);	//��ʼ����ʱ��2������ʱ��500ms
 	TIMER_CallbackInstall(HW_TIMER2, timer2);	
	TIM_us_Init(TIM3, 100, TIM_IT_Update,1, ENABLE);
	TIMER_CallbackInstall(HW_TIMER3, timer1);
//	TIM_Cmd(TIM1,ENABLE);	
	//����1�����ش��ڣ����Դ���
	UART_QuickInit(HW_UART1, 9600, 2, 2, ENABLE);  //��ʼ������1��������9600
	//����2:��������
	UART_QuickInit(HW_UART2, 9600, 2, 2, ENABLE);  //��ʼ������2��������9600
	UART_CallbackInstall(HW_UART2, uart2);
	
	GPIO_QuickInit(HW_GPIOA, GPIO_Pin_8, GPIO_Mode_IPU);//��ʼ���밴������������
	//��ʼ������Ѱ��ģ��
	GPIO_QuickInit(HW_GPIOC, GPIO_Pin_13, GPIO_Mode_IPU);//��ʼ����ģ������������
	GPIO_QuickInit(HW_GPIOC, GPIO_Pin_14, GPIO_Mode_IPU);//��ʼ����ģ������������
	GPIO_QuickInit(HW_GPIOC, GPIO_Pin_15, GPIO_Mode_IPU);//��ʼ����ģ������������
	GPIO_QuickInit(HW_GPIOB, GPIO_Pin_10, GPIO_Mode_IPU);//��ʼ����ģ������������

	
//	GPIO_QuickInit(HW_GPIOC, GPIO_Pin_13, GPIO_Mode_Out_PP);//��ʼ����LED���ӵ�Ӳ���ӿ�
	GPIO_QuickInit(HW_GPIOB, GPIO_Pin_11, GPIO_Mode_Out_PP);//��ʼ������������ӵ�Ӳ���ӿ�
	GPIO_QuickInit(HW_GPIOB, GPIO_Pin_0,  GPIO_Mode_Out_PP);//��ʼ����RGB�ĺ�����ӵ�Ӳ���ӿ�
	hc_sr04_init();       //��������ʼ��
	ADC_init(ADC_CH0);
	car_init();
	//��ʼ��PA8Ϊ�ⲿ�����жϣ�����
	EXTI_QuickInit(HW_EXTIA, EXTI_Pin_8, 3, 3);
	EXTI_CallbackInstall(EXTI_Pin_8, key_int);	
	//TIM1����Ϊ�ɹ�
	steer_sg90_init(TIM1, PWM_CH4);
	// steer_sg90_run_to_angle( TIM1, PWM_CH4, 90);
	BEEP = 1;
	LED_RED = 1;
	car_set_motor_speed(3500, 3500);
	
		int LeftSensorValue1 = 0;
	int LeftSensorValue2 = 0;
	int RightSensorValue1 = 0;
	int RightSensorValue2 = 0;
	int trun_time=200;
	while(1)
	{
		RightSensorValue1 = PCin(15);
		printf("%d",RightSensorValue1);
		SYSTICK_DelayMs(100);
	// car_set_motor_speed(3500,3500);
	// // car_turn_left();
	// // car_turn_left_place();
	// if(flag_run){
			
	// 		LeftSensorValue1  = PCin(14);
	// 		RightSensorValue1 = PCin(15);

	// 		LeftSensorValue2  = PCin(13);
	// 		RightSensorValue2 = PBin(10);
			
	// 		if (LeftSensorValue1 == 1 && RightSensorValue1 == 1 && (LeftSensorValue2==0||RightSensorValue2==0) ) { 
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

	}
	
}



//������⺯��
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

//�����жϺ���
void key_int(){
		SYSTICK_DelayMs(5);
		if(KEY_CTRL == 0){
			flag_run = !flag_run;
			while(KEY_CTRL == 0);
			LED_RED =  !flag_run;
		}
}

//������ⱨ������
void power_alarm(){
			if(flag_power_alarm){			
					LED_RED = ~LED_RED;
				//TIM_Cmd(TIM1,ENABLE);
			}else{
				BEEP = 1;
			}
}


void TraceTail()
{
	car_set_motor_speed(3000, 3000);
	int LeftSensorValue1 = 0;
	int LeftSensorValue2 = 0;
	int RightSensorValue1 = 0;
	int RightSensorValue2 = 0;
	int distance=0;
	int LeftSensorValue=0;
	int RightSensorValue=0;
		
	if(flag_run){
			
				LeftSensorValue1  = PCin(14);
				RightSensorValue1 = PCin(15);
				distance = get_hc_sr04_value_by_median_filter();
				LeftSensorValue  = PAin(4);
				RightSensorValue = PAin(6);
				LeftSensorValue2  = PCin(13);
				RightSensorValue2 = PBin(10);
				printf("ir is %d, %d, %d, %d \r\n",LeftSensorValue1, LeftSensorValue2, RightSensorValue1, RightSensorValue2 );
				if (LeftSensorValue1 == 1 && RightSensorValue1 == 1 ) { 
					car_set_motor_speed(3000, 3000);
					car_forward();
					// SYSTICK_DelayMs(100);        
				} 
				else if (LeftSensorValue1 == 1 && RightSensorValue1 == 0 ) {
					car_set_motor_speed(2500, 2500);
					car_turn_right_place();
					while (RightSensorValue2!=0){
						printf("trun right");
					}
					car_set_motor_speed(3000, 3000);
					car_forward();
					printf("go out");
					
					// if(LeftSensorValue2==1)
					// {
					// 	car_forward();
					// }
//					
					// SYSTICK_DelayMs(50);
				}
				else if (LeftSensorValue1 == 0 && RightSensorValue1==1) {
					car_set_motor_speed(2500, 2500);
					car_turn_left_place(); 
					// �������ڲ�������Ϊ0��˵����ǰ����
					while (LeftSensorValue2!=0){
						printf("trun left");
					}
					car_set_motor_speed(3000, 3000);
					car_forward();
					printf("go out");
					// SYSTICK_DelayMs(50); 
				}
				else {
					car_set_motor_speed(3000, 3000);
					car_forward(); 
				}
		
		}else{
			car_brake();
		}
	}


	
//���ϳ���
void avoid_func_hw()
{
		int LeftSensorValue = 0;
		int RightSensorValue = 0;
		int distance = 0;
		SYSTICK_DelayMs(20);
		distance = get_hc_sr04_value_by_median_filter();
		printf("distance:%d\r\n",distance);
		if(flag_run){
			
				LeftSensorValue  = PAin(4);
				RightSensorValue = PAin(6);
				printf("%d:%d:%d:%c\r\n",LeftSensorValue,RightSensorValue,distance,flag_run_char);
				if ((LeftSensorValue == 1 && RightSensorValue == 1 && distance>2000)||flag_run_char=='D'||flag_run_char=='O'||flag_run_char=='2') { 
					//������ϰ���
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
		if(distance >= dis)
			{
				car_set_motor_speed(3500, 3500);
				car_forward();
			}
			else if (LeftSensorValue == 1 && RightSensorValue == 0) {
				//�ұ����ϰ���
				car_set_motor_speed(3000, 3000);
				car_back();
				SYSTICK_DelayMs(delay_time);
				car_set_motor_speed(3500, 2500);
				car_turn_left_place();  
				SYSTICK_DelayMs(delay_time);
			}
			else if (RightSensorValue == 1 && LeftSensorValue == 0) {
				//������ϰ���
				car_set_motor_speed(3000, 3000);
				car_back();
				SYSTICK_DelayMs(delay_time);
				car_set_motor_speed(2500, 3500);
				car_turn_right_place();  
				SYSTICK_DelayMs(delay_time);  
			}
			else {
				//ȫ����
				car_set_motor_speed(3000, 3000);
				car_back();
				SYSTICK_DelayMs(delay_time);
				car_set_motor_speed(2500, 3500);
				car_turn_left_place();  
				SYSTICK_DelayMs(delay_time);
			}
}





// ����Ťͷ�Ĵ��� whileѭ����
// struct Obstacle Obstacle_decide=Avoid_Path_Detecet(2500);
// 		if(Obstacle_decide.right_ultrasonic==1 && Obstacle_decide.left_ultrasonic==1)
// 		{
// 			printf("R and L can't go\r\n");
// 			car_back();
// 			SYSTICK_DelayMs(100);
// 			car_turn_left();
// 			SYSTICK_DelayMs(100);
// 			// ���Ҷ�����
// 			// ����һ�ξ��룬���ж�

// 		}else if(Obstacle_decide.left_ultrasonic==1)
// 		{
// 			car_back();
// 			SYSTICK_DelayMs(100);
// 			printf("L can't go\r\n");
// 			car_turn_right_place();
// 			SYSTICK_DelayMs(100);
// 			// ������ת����ת

// 		}else if (Obstacle_decide.right_ultrasonic==1)
// 		{
// 			car_back();
// 			SYSTICK_DelayMs(100);
// 			printf("R can't go\r\n");
// 			car_turn_left_place();
// 			SYSTICK_DelayMs(100);
// 			// ������ת����ת
// 		}else if(Obstacle_decide.right_infrared==1)
// 		{
// 			car_back();
// 			SYSTICK_DelayMs(100);
// 			printf("R can't go\r\n");
// 			car_turn_left_place();
// 			SYSTICK_DelayMs(100);
// 			// ������ת����ת

// 		}else if(Obstacle_decide.left_infrared==1)
// 		{
// 			car_back();
// 			SYSTICK_DelayMs(100);
// 			printf("L can't go \r\n");
// 			//������ת����ת
// 			car_turn_right_place();
// 			SYSTICK_DelayMs(100);

// 		}else
// 		{
// 			car_forward();
// 		}
