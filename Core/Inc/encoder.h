#ifndef __ENCODER_H
#define __ENCODER_H

#include "main.h"

extern volatile int VorI;
// 1. 初始化硬件编码器
void Encoder_Init(void);

// 2. 获取编码器旋转方向 (返回: 1顺时针, -1逆时针, 0无动作)
int8_t Get_Encoder_Dir(void);

// 3. 菜单应用层逻辑 (传入方向，执行加减)
void App_Menu_Action(int8_t dir);

uint8_t Scan_Encoder_Button(void);

#endif
