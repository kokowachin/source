#include "kalman.h"

    Kalman1D_TypeDef kal_v;
		Kalman1D_TypeDef kal_i;
		
// 初始化滤波器参数
// 针对你的电压/电流 ADC，建议初始值如下：
void Kalman_Init(void) {
		//电压环
	kal_v.Estimate = 0.0f;
	kal_v.EstimateError = 1.0f;
	kal_v.Q = 0.0001f;
	kal_v.R = 5.0f;
		//电流环
	kal_i.Estimate = 0.0f;
	kal_i.EstimateError = 1.0f;
	kal_i.Q = 0.01f;
	kal_i.R = 10.0f;
}

// 卡尔曼滤波核心迭代函数（每次 ADC 中断或定时器中调用）
float Kalman_Update(Kalman1D_TypeDef *k, float measurement) {
    // 1. 预测协方差方程 (一维模型下，预测值等于上一次的最佳估计值)
    float PriorError = k->EstimateError + k->Q;

    // 2. 计算卡尔曼增益
    k->KalmanGain = PriorError / (PriorError + k->R);

    // 3. 更新最优估计值
    k->Estimate = k->Estimate + k->KalmanGain * (measurement - k->Estimate);

    // 4. 更新估计协方差
    k->EstimateError = (1.0f - k->KalmanGain) * PriorError;

    // 返回滤波后的最优值
    return k->Estimate;
}