#include "hc_sr04.h"

u8 flag_hc_sr04_outtime = 0;            //��ʱ��־λ

void HC_SR04_timer(){
	if (TIM_GetITStatus(HC_SR04_TIMERX, TIM_IT_Update) != RESET){  //���TIM3�����жϷ������
		  
		  flag_hc_sr04_outtime  = 1;        //��ʱ
		
	  	TIM_ClearITPendingBit(HC_SR04_TIMERX, TIM_IT_Update);  //���TIMx�����жϱ�־ 
	}
}

/**
 * @return[out]  ��������ʼ������---������ĳ������˿ڣ�����������GPIO����
 */
void hc_sr04_init(void){
	GPIO_QuickInit(HW_GPIOA, GPIO_Pin_15, GPIO_Mode_Out_PP);//tric
	 
	GPIO_QuickInit(HW_GPIOA, GPIO_Pin_12, GPIO_Mode_IPU);//echo
	
	TRIG = 0;
	
	TIM_us_Init_org(HC_SR04_TIMERX, 10000, 72);	       //��ʱ40ms��������680cm
	TIM_Cmd(HC_SR04_TIMERX, DISABLE);  
	
	if(HC_SR04_TIMERX == TIM2) 	TIMER_CallbackInstall(HW_TIMER2, HC_SR04_timer);	 
	else if(HC_SR04_TIMERX == TIM3) 	TIMER_CallbackInstall(HW_TIMER3, HC_SR04_timer);	 
	else if(HC_SR04_TIMERX == TIM4) 	TIMER_CallbackInstall(HW_TIMER4, HC_SR04_timer);	 
	
}

/**
 * @return[out]  ���ز�õĳ������ľ���  ��λmm
 * @node  ������ֵΪ65535������ʱ
 */
u16 get_hc_sr04_value(void){
	flag_hc_sr04_outtime = 0;
	TRIG = 1;
	SYSTICK_DelayUs(15);
	TRIG = 0;
	
	TIM_Cmd(HC_SR04_TIMERX, ENABLE);  
	while(!ECHO && flag_hc_sr04_outtime == 0);
	TIM_SetCounter(HC_SR04_TIMERX,0);
	TIM_Cmd(HC_SR04_TIMERX, ENABLE); 
	
	while(ECHO && flag_hc_sr04_outtime == 0); 
	TIM_Cmd(HC_SR04_TIMERX,DISABLE);
	
	if(flag_hc_sr04_outtime == 1){
		flag_hc_sr04_outtime = 0;
		return 65535;
	}
	
	return TIM_GetCounter(HC_SR04_TIMERX)*340/200;   //���ز�ľ���
}

u16 get_hc_sr04_value_by_median_filter(void){  //��ֵ�˲���õ���������þ���
	
	u16 dis[MEDIAN_FILTER_LEN] = {0};
	u16 tmp = 0;
	for(u8 i=0; i<MEDIAN_FILTER_LEN; i++){
		dis[i] = get_hc_sr04_value();
	}
	
	for(u8 i=0; i<MEDIAN_FILTER_LEN-1; i++){
		for(u8 j=0; j<MEDIAN_FILTER_LEN-1-i; j++){
			if(dis[j] > dis[j+1]){
				tmp = dis[j];
				dis[j] = dis[j+1];
				dis[j+1] = tmp;
			}
		}
	}
	return dis[MEDIAN_FILTER_LEN/2];
}


