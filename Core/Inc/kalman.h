#include "main.h"
// 定义卡尔曼滤波器的结构体
typedef struct {
    float Estimate;      // 最优估计值（最终输出的滤波结果）
    float EstimateError; // 估计协方差（系统内部状态，不用管）
    float Q;             // 过程噪声协方差（信任模型的程度）
    float R;             // 测量噪声协方差（信任传感器的程度）
    float KalmanGain;    // 卡尔曼增益（系统内部状态，不用管）
} Kalman1D_TypeDef;
extern Kalman1D_TypeDef kal_v;
extern Kalman1D_TypeDef kal_i;

void Kalman_Init(void);
float Kalman_Update(Kalman1D_TypeDef *k, float measurement);