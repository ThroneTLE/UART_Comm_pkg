#ifndef __SERVO_APP_H
#define __SERVO_APP_H

#include "servo_interp.h"

// 初始化舵机控制（在 main() 里调用一次）
void servo_app_init(void);

// 指定两个舵机目标角度（单位：度，0~270）
// 会平滑移动到目标位置
void servo_point_to(float base_deg, float arm_deg);
void servo_point_norm(float x01, float y01);
// 按角速度控制舵机移动（单位：度/秒）
void update_servo_from_vector(void);
void parse_vector(const char *buf, float *x, float *y);
void uart_debug_print(const char *msg);
#endif // __SERVO_APP_H
