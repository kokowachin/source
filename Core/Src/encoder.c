#include "encoder.h"
#include "function.h" // 确保这里面定义了 SET_Value 和 CtrValue 结构体
#include "tim.h"      
#include "gpio.h"

// ==========================================
// [硬件与系统配置] 
// ==========================================
#define EN_SW_GPIO_Port GPIOC       // 编码器按键引脚端口
#define EN_SW_Pin       GPIO_PIN_1  // 编码器按键引脚编号

#define ENCODER_PULSE_PER_STEP 4    // 编码器每格脉冲数

// ==========================================
// [全局交互变量] 
// ==========================================
volatile int VorI = 0;      // 0: 电压调节模式(CV), 1: 电流调节模式(CC)
uint8_t step_mode = 0;      // 0: 细调 (步进 0.01), 1: 粗调 (步进 1.0 / 0.1)

static int16_t Last_Encoder_Count = 0;

/**
 * @brief 初始化硬件编码器
 */
void Encoder_Init(void)
{
    // 开启定时器2编码器模式
    HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);	
    // 记录系统启动时的初始计数值
    Last_Encoder_Count = (int16_t)__HAL_TIM_GET_COUNTER(&htim2);
}

/**
 * @brief 按键状态机扫描（支持短按与长按）
 * @return 0: 无动作完成, 1: 触发短按(已切换VorI), 2: 触发长按(已切换步进)
 */
uint8_t Scan_Encoder_Button(void)
{
    static uint32_t press_time = 0;
    static uint8_t  btn_state = 0;   // 0:空闲, 1:按下计时中, 2:等待松开
    uint8_t event_flag = 0;
    
    // 读取当前按键电平 (假设按下为低电平 RESET)
    GPIO_PinState current_level = HAL_GPIO_ReadPin(EN_SW_GPIO_Port, EN_SW_Pin);
    
    switch (btn_state) 
    {
        case 0: // 【状态 0：空闲等待】
            if (current_level == GPIO_PIN_RESET) { 
                press_time = HAL_GetTick(); // 记录按下的时刻
                btn_state = 1;              
            }
            break;
            
        case 1: // 【状态 1：正在按下，判断时长】
            if (current_level == GPIO_PIN_SET) { 
                // 按键松开，计算总时长
                uint32_t duration = HAL_GetTick() - press_time;
                
                // >20ms防抖， <500ms为短按
                if (duration > 20 && duration < 500) {
                    VorI = !VorI;     // 【短按动作】：翻转恒压/恒流模式
                    event_flag = 1;   // 返回短按标志
                }
                btn_state = 0; // 回到空闲状态
            } 
            else if (HAL_GetTick() - press_time >= 500) { 
                // 按下超过 500ms 且未松手 -> 确认为长按
                step_mode = !step_mode; // 【长按动作】：翻转粗调/细调模式
                event_flag = 2;         // 返回长按标志
                
                btn_state = 2; // 进入等待松手状态，防止连续触发
            }
            break;
            
        case 2: // 【状态 2：等待长按松手】
            if (current_level == GPIO_PIN_SET) {
                btn_state = 0; 
            }
            break;
    }
    
    return event_flag;
}

/**
 * @brief 获取编码器旋转方向
 * @return 1: 正转, -1: 反转, 0: 无动作
 */
int8_t Get_Encoder_Dir(void)
{
    int8_t dir = 0;
    int16_t current_count = (int16_t)__HAL_TIM_GET_COUNTER(&htim2);
    int16_t diff = current_count - Last_Encoder_Count;

    if (diff >= ENCODER_PULSE_PER_STEP) {
        dir = 1;
        Last_Encoder_Count += ENCODER_PULSE_PER_STEP;
    }
    else if (diff <= -ENCODER_PULSE_PER_STEP) {
        dir = -1;
        Last_Encoder_Count -= ENCODER_PULSE_PER_STEP;
    }
    return dir;
}

/**
 * @brief 核心控制逻辑：执行步进调节
 * @param dir 旋转方向
 */
/**
 * @brief 核心控制逻辑：执行步进调节
 * @param dir 旋转方向
 */
void App_Menu_Action(int8_t dir)
{
    if (dir == 0) return;
    
    // 1. 步进值分配逻辑
    // 电压受长按控制：细调 0.01V，粗调 1.0V
    float step_v = (step_mode == 0) ? 0.1f : 1.0f;
    // 电流写死：永远固定为 0.1A 步进，不再受 step_mode 影响
    float step_i = 0.1f; 
    
    // 2. 根据当前是恒压还是恒流，执行调节
    if (VorI == 1) {
        // --- CC 电流模式 ---
        SET_Value.Iout += (dir * step_i);
        
        // 电流软件限幅 (根据你的硬件规格，这里假设最大 3A)
        if (SET_Value.Iout < 0.0f)  SET_Value.Iout = 0.0f;
        if (SET_Value.Iout > 3.5f)  SET_Value.Iout = 3.5f;
    } else {
        // --- CV 电压模式 ---
        SET_Value.Vout += (dir * step_v);
        
        // 电压软件限幅 (根据你的硬件规格，这里假设最大 30V)
        if (SET_Value.Vout < 1.0f)  SET_Value.Vout = 1.0f;
        if (SET_Value.Vout > 44.0f) SET_Value.Vout = 44.0f;
    }
    
    // 标记设定值已修改，通知主循环或 OLED 更新
    SET_Value.SET_modified_flag = 1;
}