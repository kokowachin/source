#include "PID.h"

// 实例化两个大循环控制器
PID_TypeDef pid_v;
PID_TypeDef pid_i;

// 全局状态标志，给主循环 OLED 显示用
volatile CVCC_Mode_TypeDef CVCC_Mode = CV;

/**
 * @brief PID 参数初始化
 */
void PID_Init(void)
{
    // ============ 电压环参数 (CV) ============
    // 物理意义：如果电压差 1V，Kp=2.0 就会让占空比瞬间增加 2.0%
    pid_v.Kp = 2.0f;     
    pid_v.Ki = 0.05f;    // 积分项负责消除静态误差，不宜过大
    pid_v.Kd = 0.0f;
    pid_v.error_last = 0.0f;
    pid_v.integral = 0.0f;
    pid_v.output_max = 95.0f; // 满占空比 100%
    pid_v.output_min = 0.0f;   // 最低占空比 0%

    // ============ 电流环参数 (CC) ============
    // 物理意义：电感电流上升极快，如果参数给太大，板子会发出刺耳的“滋滋”啸叫声！
    // 所以电流环的 Kp 和 Ki 必须比电压环温柔得多。
    pid_i.Kp = 0.5f;     
    pid_i.Ki = 0.01f;    
    pid_i.Kd = 0.0f;
    pid_i.error_last = 0.0f;
    pid_i.integral = 0.0f;
    pid_i.output_max = 95.0f; 
    pid_i.output_min = 0.0f;
}

/**
 * @brief 核心运算：标准位置式 PID
 * @param pid 对应的PID结构体指针 (&pid_v 或 &pid_i)
 * @param target 用户设定的目标物理量 (如 24.0V 或 5.0A)
 * @param actual 传感器读到的实际物理量 (如 22.5V 或 1.2A)
 * @return 算出的 PWM 占空比百分比 (0.0 ~ 100.0)
 */
float PID_Calc(PID_TypeDef *pid, float target, float actual)
{
    // 1. 计算当前误差 (目标 - 实际)
    float error = target - actual;
    
    // 2. 积分项累加
    pid->integral += (pid->Ki * error);
    
    // 3. 积分自身防饱和限幅
    if (pid->integral > pid->output_max) pid->integral = pid->output_max;
    if (pid->integral < pid->output_min) pid->integral = pid->output_min;
    
    // 4. 计算比例(P)和微分(D)
    float p_term = pid->Kp * error;
    float d_term = pid->Kd * (error - pid->error_last);
    
    // 更新历史误差，给下一次微分用
    pid->error_last = error;
    
    // 5. 计算总输出
    float output = p_term + pid->integral + d_term;
    
    // 6. 总输出安全限幅
    if (output > pid->output_max) output = pid->output_max;
    if (output < pid->output_min) output = pid->output_min;
    
    return output; // 返回需要的占空比力度
}