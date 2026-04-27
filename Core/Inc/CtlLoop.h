#ifndef __CTLLOOP_H
#define __CTLLOOP_H	
#include "main.h"
#include "function.h" 

void PID_Init(void);
void BuckBoostVILoopCtlPID(void);
void Reset_CurrentIntegral(void); 

//一个开关周期数字量 
#define PERIOD 10240	 

#endif
