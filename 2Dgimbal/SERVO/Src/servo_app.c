#include <math.h>
#include "servo_interp.h"
#include "main.h"
#include "usart.h"
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

servo_player_t g_player;
// ====== 工作区与校准（按需要微调这些常量） ======
static float BASE_MIN_DEG    = 65.0f;   // 仅允许朝向平面的扇区
static float BASE_MAX_DEG    = 115.0f;  // base≈90° 时正对平面

// 关键：arm=0° 为“垂直入射”，arm=90° 为“平行”
static float ARM_VERTICAL_DEG  = 0.0f;   // 垂直入射
static float ARM_PARALLEL_DEG  = 45.0f;  // 平行于平面

// 机械零位误差 & 方向（装反时设为 -1.0f）
static float BASE_OFFSET_DEG = 0.0f;
static float ARM_OFFSET_DEG  = 0.0f;
static float BASE_DIR        = +1.0f;
static float ARM_DIR         = +1.0f;
// servo_app.c
float g_dx = 0;
float g_dy = 0;
char uart_buf[64];
int uart_idx = 0;
int g_dx_int = 0;  // -100 ~ 100
int g_dy_int = 0;  // -100 ~ 100
void uart_debug_print(const char *msg) {
    HAL_UART_Transmit(&huart1, (uint8_t *)msg, strlen(msg), HAL_MAX_DELAY);
}

void parse_vector(const char *buf, float *dx, float *dy)
{
    int dx_int, dy_int;
    if (sscanf(buf, "%d,%d", &dx_int, &dy_int) == 2) {
        *dx = dx_int / 100.0f;
        *dy = dy_int / 100.0f;
    }
}


extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim4;
typedef struct {
    float kp, ki, kd;
    float prev_err;
    float integral;
    float max_speed; // 最大角速度 (度/周期)
} pid_t;

pid_t pid_base = { .kp = -0.80f, .ki = -0.000f, .kd = -11.0f, .max_speed = 9.0f };
pid_t pid_arm  = { .kp = -0.80f, .ki = -0.000f, .kd = -11.0f, .max_speed = 9.0f };

float base_angle = 135.0f; // 当前目标角
float arm_angle  = 135.0f;
float pid_update(pid_t *pid, float err)
{
    float derivative = err - pid->prev_err;
    pid->integral += err;

    float output = pid->kp * err + pid->ki * pid->integral + pid->kd * derivative;

    // 限制最大速度
    if (output > pid->max_speed) output = pid->max_speed;
    if (output < -pid->max_speed) output = -pid->max_speed;

    pid->prev_err = err;
    return output;
}


static inline float clampf(float v, float lo, float hi) {
    return (v < lo) ? lo : (v > hi) ? hi : v;
}

// 进入算法前做方向/零位/限幅
static inline float apply_calib_and_limit_base(float deg_in) {
    float d = BASE_DIR * deg_in + BASE_OFFSET_DEG;
    return clampf(d, BASE_MIN_DEG, BASE_MAX_DEG);
}

static inline float apply_calib_and_limit_arm(float deg_in) {
    float d = ARM_DIR * deg_in + ARM_OFFSET_DEG;
    // 仅允许在“垂直(0°) ↔ 平行(90°)”这段内活动
    float lo = fminf(ARM_VERTICAL_DEG, ARM_PARALLEL_DEG);
    float hi = fmaxf(ARM_VERTICAL_DEG, ARM_PARALLEL_DEG);
    return clampf(d, lo, hi);
}


void servo_app_init(void)
{
    float tick_hz = 1000.0f; // TIM2 中断频率33

    servo_channel_t base;
    servo_channel_init(&base, 135.0f, 0.0f, 270.0f,
                       tick_hz, EASE_COSINE_S,
                       angle_to_pwm_270, NULL);

    servo_channel_t arm;
    servo_channel_init(&arm, 135.0f, 0.0f, 270.0f,
                       tick_hz, EASE_COSINE_S,
                       angle_to_pwm_270, NULL);


    // 用原来的初始化函数，但路径设置为 NULL
    servo_player_init(&g_player, base, arm, NULL, 0, 0);

    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);
    HAL_TIM_Base_Start_IT(&htim2);
}


void servo_point_to(float base_deg, float arm_deg)
{
    // 先限制到物理舵机范围（0~270），再做校准与工作区裁剪
    base_deg = clampf(base_deg, 0.0f, 270.0f);
    arm_deg  = clampf(arm_deg,  0.0f, 270.0f);

    float base_cmd = apply_calib_and_limit_base(base_deg);
    float arm_cmd  = apply_calib_and_limit_arm(arm_deg);

    // 丝滑：1500ms + EASE_CUBIC_S（你已加好）
    servo_channel_move(&g_player.base, base_cmd, 500, EASE_CUBIC_S);
    servo_channel_move(&g_player.arm,  arm_cmd,  500, EASE_CUBIC_S);
}
// x ∈ [0,1] 映射到 base 的 [BASE_MIN, BASE_MAX]
// y ∈ [0,1] 映射到 arm 的 [垂直(0°), 平行(90°)]
void servo_point_norm(float x01, float y01)
{
    x01 = clampf(x01, 0.0f, 1.0f);
    y01 = clampf(y01, 0.0f, 1.0f);

    float base_deg = BASE_MIN_DEG + x01 * (BASE_MAX_DEG - BASE_MIN_DEG);
    float arm_deg  = ARM_VERTICAL_DEG + y01 * (ARM_PARALLEL_DEG - ARM_VERTICAL_DEG);

    servo_point_to(base_deg, arm_deg);
}
void update_servo_from_vector(void)
{
    // 1) 直接使用归一化误差
    //    约定：数学坐标系，上为正。为了保持你之前的方向一致性：
    //    base 用 g_dx；arm 对 g_dy 取负号（等效于你之前 target_arm 中的那个负号）
    float e_base = g_dx;      // [-1, 1]
    float e_arm  = -g_dy;     // [-1, 1]

    // 2) 可选：输入低通，抑制抖动（如不需要可删除）
    static float e_base_f = 0.0f, e_arm_f = 0.0f;
    const float alpha = 0.2f; // 越小越平滑
    e_base_f += alpha * (e_base - e_base_f);
    e_arm_f  += alpha * (e_arm  - e_arm_f);

    // 3) PID 输出 = 每周期的角度增量（单位：度/周期）
    float delta_base = pid_update(&pid_base, e_base_f);
    float delta_arm  = pid_update(&pid_arm,  e_arm_f);

    // 4) 累加到当前角度
    base_angle += delta_base;
    arm_angle  += delta_arm;

    // 5) 工作区/装配校准限幅（只限角度，不动误差）
    base_angle = apply_calib_and_limit_base(base_angle);
    arm_angle  = apply_calib_and_limit_arm(arm_angle);

    // 6) 下发到舵机（保持你原来的“丝滑”插值时长/曲线）
    servo_channel_move(&g_player.base, base_angle, 1, EASE_LINEAR);
    servo_channel_move(&g_player.arm,  arm_angle,  1, EASE_LINEAR);
}



