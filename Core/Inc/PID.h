#include "stdint.h"

// 恒压恒流状态枚举
typedef enum {
    CV = 0, // 恒压模式
    CC = 1  // 恒流模式
} CVCC_Mode_TypeDef;

extern volatile CVCC_Mode_TypeDef CVCC_Mode;

// 位置式 PID 结构体
typedef struct {
    float Kp;           // 比例系数
    float Ki;           // 积分系数
    float Kd;           // 微分系数
    float error_last;   // 上一次的误差
    float integral;     // 积分累加项 (双环无缝钳位的核心！)
    float output_max;   // 输出上限 (限制在 100.0%)
    float output_min;   // 输出下限 (限制在 0.0%)
} PID_TypeDef;

// 声明外部变量，让中断和主循环都能调用
extern PID_TypeDef pid_v; // 电压环控制器
extern PID_TypeDef pid_i; // 电流环控制器

void PID_Init(void);
float PID_Calc(PID_TypeDef *pid, float target, float actual);
