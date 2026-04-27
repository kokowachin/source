#ifndef _STATE_H_
#define _STATE_H_

#include "main.h"
#include "function.h"

extern volatile uint16_t ADC1_RESULT[4];
extern struct  _ADI SADC;
extern struct  _Ctr_value  CtrValue;
extern struct  _FLAG    DF;
extern uint16_t OLEDShowCnt;

/*****************************故障类型*****************/
#define     F_NOERR      			0x0000//无故障
#define     F_SW_VIN_UVP  		0x0001//输入欠压
#define     F_SW_VIN_OVP    	0x0002//输入过压
#define     F_SW_VOUT_UVP  		0x0004//输出欠压
#define     F_SW_VOUT_OVP    	0x0008//输出过压
#define     F_SW_IOUT_OCP    	0x0010//输出过流
#define     F_SW_SHORT  			0x0020//输出短路

void StateMInit(void);
void StateMWait(void);
void StateMRise(void);
void StateMRun(void);
void StateMErr(void);
void StateM(void);

//标志位定义
struct  _FLAG
{
	uint16_t	SMFlag;//状态机标志位
	uint16_t	CtrFlag;//控制标志位
	uint16_t  ErrFlag;//故障标志位
	uint8_t	BBFlag;//运行模式标志位，BUCK模式，BOOST模式，MIX混合模式	
	uint8_t PWMENFlag;//启动标志位	
	uint8_t BBModeChange;//工作模式切换标志位
};

//状态机枚举量
typedef enum
{
    Init,//初始化
    Wait,//空闲等待
    Rise,//软启
    Run,//正常运行
    Err//故障
}STATE_M;

//状态机枚举量
typedef enum
{
    NA,//未定义
		Buck,//BUCK模式
    Boost,//BOOST模式
    Mix//MIX混合模式
}BB_M;

//区分首次启动和故障恢复时软起动的区别
typedef enum {
    START_FIRST,      // 首次启动
    START_FAULT_RECOVER  // 故障恢复
} StartType_t;

//软启动枚举变量
typedef enum
{
	SSInit,//软启初始化
	SSWait,//软启等待
	SSRun//开始软启
 } SState_M;


 
 #endif
 