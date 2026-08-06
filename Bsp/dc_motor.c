#include "dc_motor.h"
#include "pid.h"
#include "foc_drv.h"
#include "AS5600.h"
#include "foc_task.h"
#include "tim.h"

/* H桥双PWM接线: TIM3_CH1(PA6)=IN1, TIM3_CH2(PA7)=IN2(见tim.c MX_TIM3_Init/MspPostInit) */
#define DC_MOTOR_TIM3_CH_IN1  TIM_CHANNEL_1
#define DC_MOTOR_TIM3_CH_IN2  TIM_CHANNEL_2
/* 对应tim.c里MX_TIM3_Init的htim3.Init.Period,84MHz/(4199+1)=20kHz,避开可听见的电机啸叫 */
#define DC_MOTOR_PWM_PERIOD   4199U

/* 编码器线数(每转脉冲数),按实际编码器规格改;TIM_ENCODERMODE_TI12对AB两相的四个边沿都计数,
   是线数的4倍,这个宏已经包含了4倍频,不要再乘4 */
#define DC_MOTOR_ENCODER_LINES_PER_REV     500.0f
#define DC_MOTOR_ENCODER_COUNTS_PER_REV    (DC_MOTOR_ENCODER_LINES_PER_REV * 4.0f)

extern PIDController PID_DC_Speed;
extern PIDController PID_DC_Position;

volatile DCMotor_Mode_t g_dc_motor_mode = DC_MOTOR_MODE_IDLE;
volatile float DCMotor_Mech_Angle = 0.0f;
volatile float DCMotor_Mech_RPM = 0.0f;
volatile float DCMotor_PWM_Duty = 0.0f;

static float dc_total_angle_rad = 0.0f;
static uint16_t dc_last_cnt = 0U;

static uint16_t DCMotor_DutyToCompare(float duty)
{
    float mag = (duty < 0.0f) ? -duty : duty;
    if (mag > 1.0f)
    {
        mag = 1.0f;
    }
    return (uint16_t)(mag * (float)DC_MOTOR_PWM_PERIOD);
}

/* duty范围[-1,1]: 正=IN1给PWM/IN2拉低(正转), 负=IN1拉低/IN2给PWM(反转), 0=双路都不通(自由滑行) */
static void DCMotor_SetDuty(float duty)
{
    uint16_t compare;

    DCMotor_PWM_Duty = duty;
    compare = DCMotor_DutyToCompare(duty);

    if (duty >= 0.0f)
    {
        __HAL_TIM_SET_COMPARE(&htim3, DC_MOTOR_TIM3_CH_IN1, compare);
        __HAL_TIM_SET_COMPARE(&htim3, DC_MOTOR_TIM3_CH_IN2, 0U);
    }
    else
    {
        __HAL_TIM_SET_COMPARE(&htim3, DC_MOTOR_TIM3_CH_IN1, 0U);
        __HAL_TIM_SET_COMPARE(&htim3, DC_MOTOR_TIM3_CH_IN2, compare);
    }
}

void DCMotor_Init(void)
{
    HAL_TIM_PWM_Start(&htim3, DC_MOTOR_TIM3_CH_IN1);
    HAL_TIM_PWM_Start(&htim3, DC_MOTOR_TIM3_CH_IN2);
    HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);

    dc_last_cnt = (uint16_t)__HAL_TIM_GET_COUNTER(&htim4);
    dc_total_angle_rad = 0.0f;
    DCMotor_Mech_Angle = 0.0f;
    DCMotor_Mech_RPM = 0.0f;
    DCMotor_SetDuty(0.0f);
}

void DCMotor_SetMode(uint8_t mode)
{
    if (mode > (uint8_t)DC_MOTOR_MODE_POSITION)
    {
        return;
    }
    g_dc_motor_mode = (DCMotor_Mode_t)mode;
}

/* TIM4是16位循环计数的编码器计数器,相邻两拍的原始计数差值强转成int16_t即可自动处理
   跨0/65536回绕(标准无符号差值截断成有符号数的技巧),前提是单拍(0.5ms)转过的脉冲数
   不超过±32767——4倍频下相当于±8191个编码器整线,换算成转速上限高达几十万RPM,
   远超直流有刷电机实际能达到的转速,不会产生二义性。 */
static float DCMotor_ReadEncoderDeltaRad(void)
{
    uint16_t raw = (uint16_t)__HAL_TIM_GET_COUNTER(&htim4);
    int16_t delta_counts = (int16_t)(raw - dc_last_cnt);

    dc_last_cnt = raw;
    return ((float)delta_counts / DC_MOTOR_ENCODER_COUNTS_PER_REV) * 6.283185307f;
}

void DCMotor_ControlTick(void)
{
    static DCMotor_Mode_t prev_mode = DC_MOTOR_MODE_IDLE;
    float delta_rad = DCMotor_ReadEncoderDeltaRad();
    float pid_out;

    dc_total_angle_rad += delta_rad;
    DCMotor_Mech_Angle = _normalizeAngle(dc_total_angle_rad);
    DCMotor_Mech_RPM = rad_sec_to_rpm(delta_rad / FOC_LOOP_DT_S);

    if (g_dc_motor_mode != prev_mode)
    {
        /* 模式切换清空积分/历史误差并把占空比归零,避免残留积分或上一模式遗留的占空比
           在切换瞬间造成冲击,和FOC_ModeDispatch(App/foc_task.c)的处理方式一致 */
        PID_Reset(&PID_DC_Speed);
        PID_Reset(&PID_DC_Position);
        DCMotor_SetDuty(0.0f);
        prev_mode = g_dc_motor_mode;
    }

    switch (g_dc_motor_mode)
    {
        case DC_MOTOR_MODE_SPEED:
            pid_out = PID_Position_Calculate(&PID_DC_Speed, PID_DC_Speed.target, DCMotor_Mech_RPM, FOC_LOOP_DT_S);
            DCMotor_SetDuty(pid_out);
            break;

        case DC_MOTOR_MODE_POSITION:
        {
            float wrapped_err = AngleErrorWrap(PID_DC_Position.target - DCMotor_Mech_Angle);

            PID_DC_Speed.target = PID_Position_Calculate(&PID_DC_Position, DCMotor_Mech_Angle + wrapped_err,
                                                           DCMotor_Mech_Angle, FOC_LOOP_DT_S);
            pid_out = PID_Position_Calculate(&PID_DC_Speed, PID_DC_Speed.target, DCMotor_Mech_RPM, FOC_LOOP_DT_S);
            DCMotor_SetDuty(pid_out);
            break;
        }

        case DC_MOTOR_MODE_IDLE:
        default:
            DCMotor_SetDuty(0.0f);
            break;
    }
}
