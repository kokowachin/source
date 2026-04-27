#include "CtlLoop.h"
#include "function.h"
#include "main.h"

// 声明外部定义的定时器句柄
extern TIM_HandleTypeDef htim1;

extern volatile uint16_t ADC1_RESULT[4];          // ADC1通道1~4采样结果
volatile int32_t VErr0 = 0, VErr1 = 0, VErr2 = 0; // 电压误差
volatile int32_t IErr0 = 0, IErr1 = 0;            // 电流误差
volatile int32_t u0 = 0, u1 = 0;                  // 电压环输出量
volatile int32_t i0 = 0, i1 = 0;                  // 电流环输出量
volatile _CVCC_Mode CVCC_Mode = CV;               // 恒流恒压模式标志位
volatile int32_t I_Integral = 0;									//电流环路积分

//环路的参数buck输出-恒压-PID型补偿器
// 环路的参数buck输出-恒压-PID型补偿器
#define BUCKPIDb0 5271
#define BUCKPIDb1 -10363
#define BUCKPIDb2 5093
// 环路的参数BOOST输出-恒压-PID型补偿器
#define BOOSTPIDb0 8044
#define BOOSTPIDb1 -15813
#define BOOSTPIDb2 7772

#define ILOOP_KP 6 // 电流环PID补偿器P值
#define ILOOP_KI 3 // 电流环PID补偿器I值
#define ILOOP_KD 1 // 电流环PID补偿器D值

//PID初始化
void PID_Init(void)
{
    // 电压环重置
    VErr0 = 0; VErr1 = 0; VErr2 = 0;
    u0 = 0; u1 = 0;
    
    // 电流环重置
    IErr0 = 0; IErr1 = 0;   
    i0 = 0; i1 = 0;I_Integral = 0; 
}


/**
 * @brief BuckBoost电压电流环路控制PID函数。
 * 该函数用于实现BuckBoost电压电流环路控制的PID算法。
 * 在stm32f4xx_it.c文件中的TIM3_IRQHandler中断函数里调用此函数。
 */
void BuckBoostVILoopCtlPID(void)
{
		uint16_t sample_vin, sample_iin, sample_vout, sample_iout;
		sample_vin  = ADC1_RESULT[0];
    sample_iin  = ADC1_RESULT[1];
    sample_vout = ADC1_RESULT[2];
    sample_iout = ADC1_RESULT[3];
	
    CtrValue.Vout_ref = CtrValue.Vout_SETref; // 输出参考电压设置为设置电压

    int32_t VoutTemp = (sample_vout* CAL_VOUT_K >> 12) + CAL_VOUT_B; // 获取矫正后的输出电压
    int32_t IoutTemp = (sample_iout* CAL_IOUT_K >> 12) + CAL_IOUT_B; // 获取矫正后的输出电流

    // 计算电流误差量，当输出电流小于参考电流，输出量增加
    IErr0 = CtrValue.Iout_ref - IoutTemp;
    // 电流环路输出= 积分量 + KP*误差量 + KD*当前误差减上次误差
    i0 = I_Integral + IErr0 * ILOOP_KP + (IErr0 - IErr1) * ILOOP_KD;
    // 积分量=积分量+KI*误差量
    I_Integral = I_Integral + IErr0 * ILOOP_KI;

    // 积分量限制，积分量最大值限制
    if (I_Integral > ADC_MAX_VALUE)
        I_Integral = ADC_MAX_VALUE;

    if (DF.SMFlag == Rise && (VoutTemp < (CtrValue.Vout_ref / 2))) // 判断是否在软启动状态
    {

        CtrValue.Vout_ref = CtrValue.Vout_ref + i0;  // 输出参考电压加上电流环计算结果
        CVCC_Mode = CC;                              // 恒流模式
        if (CtrValue.Vout_ref > CtrValue.Vout_SSref) // 输出参考电压超过软启动设置电压时限制在软启动设置电压
        {
            CtrValue.Vout_ref = CtrValue.Vout_SSref; // 限制输出参考电压
            CVCC_Mode = CV;                          // 恒压模式
        }
        if (CtrValue.Vout_ref < 0) // 输出参考电压小于0时限制在0
        {
            CtrValue.Vout_ref = 0;
        }
    }
    else
    {
        CtrValue.Vout_ref = CtrValue.Vout_ref + i0;   // 输出参考电压加上电流环计算结果
        CVCC_Mode = CC;                               // 恒流模式
        if (CtrValue.Vout_ref > CtrValue.Vout_SETref) // 输出参考电压超过设置电压时限制在设置电压
        {
            CtrValue.Vout_ref = CtrValue.Vout_SETref; // 限制输出参考电压
            CVCC_Mode = CV;                           // 恒压模式
        }
        if (CtrValue.Vout_ref < 0) // 输出参考电压小于0时限制在0
        {
            CtrValue.Vout_ref = 0;
        }
    }

    VErr0 = CtrValue.Vout_ref - VoutTemp; // 计算电压误差量，当参考电压大于输出电压，占空比增加，输出量增加

    // 当模式切换时，降低占空比，确保模式切换不过冲
    // BBModeChange为模式切换为，不同模式切换时，该位会被置1
    if (DF.BBModeChange)
    {
        u1 = 0;
        I_Integral = 0;
        i0 = 0;
        DF.BBModeChange = 0;
			  VErr0 = VErr1 = VErr2 = 0; 
    }

    // 判断工作模式，BUCK，BOOST，BUCK-BOOST
    switch (DF.BBFlag)
    {
    
			case NA: // 初始阶段
    {
        VErr0 = 0;
        VErr1 = 0;
        VErr2 = 0;
        u0 = 0;
        u1 = 0;
        i0 = 0;
        I_Integral = 0;
        IErr0 = 0;
        IErr1 = 0;
        break;
    }
    
		case Buck: // BUCK模式
    {
        u0 = u1 + VErr0 * BUCKPIDb0 + VErr1 * BUCKPIDb1 + VErr2 * BUCKPIDb2; // 计算电压环输出
        // 历史数据幅值
        VErr2 = VErr1;
        VErr1 = VErr0;
        u1 = u0;

        // 环路输出赋值
        CtrValue.BoostDuty = MIN_BOOST_DUTY1; // BOOST上管固定占空比94%，下管6%
        CtrValue.BuckDuty = (u0 >> 8) * 3;    // 电压环占空比输出

        // 环路输出最大最小占空比限制
        if (CtrValue.BuckDuty > CtrValue.BUCKMaxDuty)
            CtrValue.BuckDuty = CtrValue.BUCKMaxDuty;
        if (CtrValue.BuckDuty < MIN_BUCK_DUTY)
            CtrValue.BuckDuty = MIN_BUCK_DUTY;
        break;
    }
    
		case Boost: // Boost模式
    {
        // 调用PID环路计算公式（参照PID环路计算文档）
        u0 = u1 + VErr0 * BOOSTPIDb0 + VErr1 * BOOSTPIDb1 + VErr2 * BOOSTPIDb2;
        // 历史数据幅值
        VErr2 = VErr1;
        VErr1 = VErr0;
        u1 = u0;

        // 环路输出赋值
        CtrValue.BuckDuty = MAX_BUCK_DUTY;  // BUCK上管固定占空比94%
        CtrValue.BoostDuty = (u0 >> 8) * 3; // 电压环占空比输出

        // 环路输出最大最小占空比限制
        if (CtrValue.BoostDuty > CtrValue.BoostMaxDuty)
            CtrValue.BoostDuty = CtrValue.BoostMaxDuty;
        if (CtrValue.BoostDuty < MIN_BOOST_DUTY)
            CtrValue.BoostDuty = MIN_BOOST_DUTY;
        break;
    }
    
		case Mix: // Mix模式
    {
        // 调用PID环路计算公式
        u0 = u1 + VErr0 * BOOSTPIDb0 + VErr1 * BOOSTPIDb1 + VErr2 * BOOSTPIDb2;
        // 历史数据幅值
        VErr2 = VErr1;
        VErr1 = VErr0;
        u1 = u0;
        IErr1 = IErr0;

        // 环路输出赋值
        CtrValue.BuckDuty = MAX_BUCK_DUTY1; // BUCK上管固定占空比80%
        CtrValue.BoostDuty = (u0 >> 8) * 3; // 电压环占空比输出

        // 环路输出最大最小占空比限制
        if (CtrValue.BoostDuty > CtrValue.BoostMaxDuty)
            CtrValue.BoostDuty = CtrValue.BoostMaxDuty;
        if (CtrValue.BoostDuty < MIN_BOOST_DUTY)
            CtrValue.BoostDuty = MIN_BOOST_DUTY;
        break;
    }
    }

    // PWMENFlag是PWM开启标志位，当该位为0时,buck的占空比为0，无输出;
    // =========================================================
    // PWM 占空比更新与安全保护逻辑
    // TIM1_CH1 (PE9)  与 CH1N (PA7)  为 BUCK  桥臂控制
    // TIM1_CH2 (PE11) 与 CH2N (PB0)  为 BOOST 桥臂控制
    // =========================================================
    
    // PWMENFlag是PWM开启标志位
    if (DF.PWMENFlag) 
    {
        // 1. 恢复高级定时器的主输出使能 (Main Output Enable)
        TIM1->BDTR |= TIM_BDTR_MOE;

        // 2. 更新定时器比较寄存器以调整占空比
        // 硬件会自动根据 CCR 的值生成带死区的互补波形给到 PE9/PA7 和 PE11/PB0
        TIM1->CCR1 = CtrValue.BuckDuty;
        TIM1->CCR2 = CtrValue.BoostDuty;
    }
    else 
    {
        PWMStop();
    }
}

//重置函数
void Reset_CurrentIntegral(void)
{
    I_Integral = 0;
}
