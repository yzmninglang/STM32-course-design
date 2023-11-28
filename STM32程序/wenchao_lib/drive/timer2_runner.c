#include "sysinc.h"			


static TIMER2_RUNNER_CallBackType TIMER2_RUNNER_CallBackTable[4][10] = {NULL};
timer2_sys_sta Status0 ;

void timer2_runner(){
		if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET)  //检查TIM3更新中断发生与否
		{ 
			System_Flag_5Ms = 1;
			TIM_ClearITPendingBit(TIM2, TIM_IT_Update);  //清除TIMx更新中断标志 
		}
	
}

void time2_500ms_runner_deal(){

		if( System_Flag_500Ms == 1 ){
			System_Flag_500Ms = 0;
			
			//处理函数
			for(int i = 0; i < 10; i++){
				if(TIMER2_RUNNER_CallBackTable[3][i] != NULL){
					TIMER2_RUNNER_CallBackTable[3][i]();
				}else{
					break;
				}
			}			
			
		} 
}

void time2_100ms_runner_deal(){
	static u8 counter = 0;

		if( System_Flag_100Ms == 1 ){
			System_Flag_100Ms = 0;
			
			//处理函数
			for(int i = 0; i < 10; i++){
				if(TIMER2_RUNNER_CallBackTable[2][i] != NULL){
					TIMER2_RUNNER_CallBackTable[2][i]();
				}else{
					break;
				}
			}
						
			if(++counter >= 5){
				counter = 0;
				System_Flag_500Ms = 1;
			}			
		}
}

void time2_10ms_runner_deal(){
	static u8 counter = 0;

		if( System_Flag_10Ms == 1 ){
			System_Flag_10Ms = 0;
			
			//处理函数
			for(int i = 0; i < 10; i++){
				if(TIMER2_RUNNER_CallBackTable[1][i] != NULL){
					TIMER2_RUNNER_CallBackTable[1][i]();
				}else{
					break;
				}
			}
						
			if(++counter >= 10){
				counter = 0;
				System_Flag_100Ms = 1;
			}			
		}
}

void time2_5ms_runner_deal(){
	static u8 counter = 0;
	if( System_Flag_5Ms==1 ){
			System_Flag_5Ms = 0;
			
			//处理函数
			for(int i = 0; i < 10; i++){
				if(TIMER2_RUNNER_CallBackTable[0][i] != NULL){
					TIMER2_RUNNER_CallBackTable[0][i]();
				}else{
					break;
				}
			}
			
			if(++counter >= 2){
				counter = 0;
				System_Flag_10Ms = 1;
			}
	}
}


void timer2_runner_init(){
	TIM_us_Init(TIM2, 5000, TIM_IT_Update,2, ENABLE);	

	TIMER_CallbackInstall(HW_TIMER2, timer2_runner);
	
	for(int i = 0; i <4; i++){
		for(int k=0; k<10; k++){
			TIMER2_RUNNER_CallBackTable[i][k] = NULL;
		}
	}
}

void timer2_runner_deal(void){
	time2_5ms_runner_deal();
	time2_10ms_runner_deal();
	time2_100ms_runner_deal();
	time2_500ms_runner_deal();
}

void TIMER2_5ms_CallbackInstall(TIMER2_RUNNER_CallBackType AppCBFun){
	static u8 counter = 0;
	if(counter < 10 && TIMER2_RUNNER_CallBackTable[0][counter] == NULL){
		TIMER2_RUNNER_CallBackTable[0][counter] = AppCBFun;
		counter++;
	}else{
		return;  
	}
}

void TIMER2_10ms_CallbackInstall(TIMER2_RUNNER_CallBackType AppCBFun){
	static u8 counter = 0;
	if(counter < 10 && TIMER2_RUNNER_CallBackTable[1][counter] == NULL){
		TIMER2_RUNNER_CallBackTable[1][counter] = AppCBFun;
		counter++;
	}else{
		return;  
	}
}

void TIMER2_100ms_CallbackInstall(TIMER2_RUNNER_CallBackType AppCBFun){
	static u8 counter = 0;
	if(counter < 10 && TIMER2_RUNNER_CallBackTable[2][counter] == NULL){
		TIMER2_RUNNER_CallBackTable[2][counter] = AppCBFun;
		counter++;
	}else{
		return;  
	}
}


void TIMER2_500ms_CallbackInstall(TIMER2_RUNNER_CallBackType AppCBFun){
	static u8 counter = 0;
	if(counter < 10 && TIMER2_RUNNER_CallBackTable[3][counter] == NULL){
		TIMER2_RUNNER_CallBackTable[3][counter] = AppCBFun;
		counter++;
	}else{
		return;  
	}
}


