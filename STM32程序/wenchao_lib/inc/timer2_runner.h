/**
  ******************************************************************************
  * @file    timer2_runner.h
  * @author  张文超 
	* @qq      269426626
  * @version V1.0
  * @date    2019.8.05
  * @note    timer2给出5ms的时钟基准,控制系统稳步向前(时间转轮)
  ******************************************************************************
  */

#ifndef _TIMER2_RUNNER_H_
#define _TIMER2_RUNNER_H_

#include "sysinc.h"							

typedef void (*TIMER2_RUNNER_CallBackType)(void);

typedef struct timer2_runner_sys_sta
{
    unsigned char a:1 ;
    unsigned char b:1 ;
    unsigned char c:1 ;
    unsigned char d:1 ; 
}timer2_sys_sta ;

extern timer2_sys_sta Status0 ;

#define System_Flag_5Ms     Status0.a
#define System_Flag_10Ms    Status0.b 
#define System_Flag_100Ms   Status0.c
#define System_Flag_500Ms   Status0.d


void timer2_runner_init(void);
void timer2_runner_deal(void);

void TIMER2_5ms_CallbackInstall(TIMER2_RUNNER_CallBackType AppCBFun);
void TIMER2_10ms_CallbackInstall(TIMER2_RUNNER_CallBackType AppCBFun);
void TIMER2_100ms_CallbackInstall(TIMER2_RUNNER_CallBackType AppCBFun);
void TIMER2_500ms_CallbackInstall(TIMER2_RUNNER_CallBackType AppCBFun);


#endif




