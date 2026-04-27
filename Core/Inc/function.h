#ifndef _FUNCTION_H_
#define _FUNCTION_H_

#include "main.h"

//宏定义
#define ADC_MAX_VALUE 4095.0F				   // ADC最大值
#define REF_3V3 3.3006F						   // VREF参考电压


#define MIN_BUCK_DUTY	80//BUCK最小占空比
#define MAX_BUCK_DUTY 3809//BUCK最大占空比，93%*Q12
#define	MAX_BUCK_DUTY1 3277//MIX模式下 BUCK固定占空比80%
#define MIN_BOOST_DUTY 80//BOOST最小占空比
#define MIN_BOOST_DUTY1 283//BOOST最小占空7%
#define MAX_BOOST_DUTY	2662//最大占空比 65%最大占空比
#define MAX_BOOST_DUTY1	3809//BOOST最大占空比，93%*Q12


// 电压/电流校正系数（Q12格式）
#define CAL_VIN_K   4096    // 输入电压校正斜率
#define CAL_VIN_B   0       // 输入电压校正截距

#define CAL_IIN_K   4096    // 输入电流校正斜率
#define CAL_IIN_B   0       // 输入电流校正截距

#define CAL_VOUT_K  4096    // 输出电压校正斜率
#define CAL_VOUT_B  0       // 输出电压校正截距

#define CAL_IOUT_K  4096    // 输出电流校正斜率
#define CAL_IOUT_B  0       // 输出电流校正截距

// 在 function.h 中声明（
extern volatile int32_t VErr0, VErr1, VErr2;  // 电压误差
extern volatile int32_t IErr0, IErr1;         // 电流误差
extern volatile int32_t u0, u1;               // 电压环输出量
extern volatile int32_t i0, i1;               // 电流环输出量
extern volatile float VOUT;

//结构体
struct _ADI {
    uint16_t Vin;       // 输入电压采样值
    uint16_t Iin;       // 输入电流采样值
    uint16_t IinOffset; // 输入电流偏置（用于校正）
    uint16_t Iout;      // 输出电流采样值
    uint16_t IoutOffset;// 输出电流偏置（用于校正）
    uint16_t Vout;      // 输出电压采样值
    uint16_t Vadj;      // 参考电压调节值（从滑动变阻器）
    uint16_t VadjAvg;   // 参考电压平均值
    uint16_t VinAvg;    // 输入电压平均值
    uint16_t IinAvg;    // 输入电流平均值
    uint16_t VoutAvg;   // 输出电压平均值
    uint16_t IoutAvg;   // 输出电流平均值
		uint16_t VinOffset; // 输入电压偏置（用于矫正）
	  uint16_t VoutOffset;// 输出电压偏置（用于矫正） 
};

extern struct _ADI SADC;  //输入输出参数采样值和平均值

typedef struct {
    float Vout;               // 输出电压设定
    float Iout;               // 输出电流设定
    uint8_t SET_modified_flag;// 数据已修改标记
} SET_Param_t;

struct _Ctr_value
{
	volatile int32_t Vout_ref;	   // 输出参考电压
	volatile int32_t Vout_SSref;   // 软启动时的输出参考电压
	volatile int32_t Vout_SETref;  // 设置的参考电压
	volatile int32_t Iout_ref;	   // 输出参考电流
	volatile int32_t I_Limit;	   // 限流参考电流
	volatile int16_t BUCKMaxDuty;  // Buck最大占空比
	volatile int16_t BoostMaxDuty; // Boost最大占空比
	volatile int16_t BuckDuty;	   // Buck控制占空比
	volatile int16_t BoostDuty;	   // Boost控制占空比
	volatile int32_t Ilimitout;	   // 电流环输出
};

extern struct _FLAG  DF;//控制标志位

extern volatile uint16_t ADC1Read[4];
extern volatile uint16_t ADC1_RESULT[4];//ADC采样外设到内存的DMA数据保存寄存器

extern SET_Param_t SET_Value; // 在 .c 中实例化，此处声明供外部调用

/*
 * 设置寄存器的位
 * 参数：
 *   reg: 要操作的寄存器
 *   mask: 指定要设置的位掩码
 * 返回值：无
 */
#define setRegBits(reg, mask) (reg |= (unsigned int)(mask))

/*
 * 清除寄存器的位
 * 参数：
 *   reg: 要操作的寄存器
 *   mask: 指定要清除的位掩码
 * 返回值：无
 */
#define clrRegBits(reg, mask) (reg &= (unsigned int)(~(unsigned int)(mask)))

/*
 * 获取寄存器中指定位的值
 * 参数：
 *   reg: 要操作的寄存器
 *   mask: 指定要获取的位掩码
 * 返回值：掩码中为1的位的值
 */
#define getRegBits(reg, mask) (reg & (unsigned int)(mask))

/*
 * 获取寄存器的值
 * 参数：
 *   reg: 要获取的寄存器
 * 返回值：寄存器的当前值
 */
#define getReg(reg) (reg)

//函数部分
void ADCSample(void);
void ValInit(void);	//初始化
void PWM_Start(void);	//启动PWM
void Check_NoLoad_And_ClearCC(void);//无负载切换回恒压模式
void OCP(void);//输出过流保护函数
void OVP(void);//输出电压保护函数
void ShortOff(void);//短路保护
void BBMode(void);//模式判断
void PWMStop(void);//关闭pwm
void Test_PWM_Output(void);//测试代码
#endif
