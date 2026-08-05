#include "cmsis_os.h"
#include "math.h"
#include "main.h"
#include "AS5600.h"
#include "foc_drv.h"
#include "pid.h"
#include "current_sense.h"
#include "uart_task.h"
#include "timer_utils.h"
#include "foc_task.h"

extern Udq_Struct Udq_M0;
extern Ualpbe_Struct Ualpbe_M0;
extern Iabc_Struct Iabc_M0;
extern Ialpbe_Struct Ialpbe_M0;
extern Iqd_Struct Iqd_M0;
extern SVPWM_Struct SVPWM_M0;
extern AS5600 M0;

extern volatile float Mech_Angle;
extern float Elec_Angle;
extern volatile float Mech_RPM;

extern PIDController PID_Current_D;
extern PIDController PID_Current_Q;
extern PIDController PID_Speed;
extern PIDController PID_Position;

extern osMessageQueueId_t PIDQueueHandle;
extern PID_Param_t Speed_pid;

#define OPEN_LOOP_ALIGN_TIME_MS          500U
#define OPEN_LOOP_RAMP_TIME_MS           3000U
#define OPEN_LOOP_ALIGN_UD_V_DEFAULT     1.0f
#define OPEN_LOOP_RUN_UQ_V_DEFAULT       1.2f
#define OPEN_LOOP_TARGET_ELEC_HZ_DEFAULT 5.0f
#define OPEN_LOOP_POLE_PAIRS             7.0f
#define OPEN_LOOP_UART_PERIOD_MS         50U
#define OPEN_LOOP_TWO_PI                 6.283185307f
/* TIM10实测: 168MHz/(84分频)=2MHz计数时钟, ARR=1000 => 更新频率约2kHz(约0.5ms/拍),
   而不是注释原来假设的1kHz/1ms, 这里改成与硬件实际节拍一致的值。
   该值同时喂给下面PID的dt形参和Mech_RPM/开环电角度积分,三处必须与TIM10真实周期保持一致,
   以后如果改了TIM10的Prescaler/Period,要同步改这里。 */
#define FOC_LOOP_DT_S                    0.0005f

volatile FOC_Mode_t g_foc_mode = FOC_MODE_IDLE;

float OpenLoop_AlignUd_V = OPEN_LOOP_ALIGN_UD_V_DEFAULT;
float OpenLoop_RunUq_V = OPEN_LOOP_RUN_UQ_V_DEFAULT;
float OpenLoop_TargetElecHz = OPEN_LOOP_TARGET_ELEC_HZ_DEFAULT;

static float prev_speed_angle = 0.0f;
static uint8_t prev_speed_valid = 0U;

static void MechSpeed_Update(float current_angle)
{
    float angle_delta;

    if (prev_speed_valid == 0U)
    {
        prev_speed_angle = current_angle;
        prev_speed_valid = 1U;
        return;
    }

    angle_delta = current_angle - prev_speed_angle;
    if (angle_delta > 3.14159f)
    {
        angle_delta -= 2.0f * 3.14159f;
    }
    else if (angle_delta < -3.14159f)
    {
        angle_delta += 2.0f * 3.14159f;
    }

    Mech_RPM = rad_sec_to_rpm(angle_delta / FOC_LOOP_DT_S);
    prev_speed_angle = current_angle;
}

void angle_proc(void)
{
    float raw_angle;

    /* 用上一拍已经由I2C2 DMA读回并处理好的角度(AS5600_Init里的一次阻塞读取打底,
       之后全部走DMA),而不是本拍再去阻塞读I2C——100kHz I2C下一次阻塞读约0.4~0.5ms,
       几乎占满1ms控制周期的一半。DMA方式把这部分时间从FOC中断里去掉,代价是角度
       多了约一拍(当前周期下约1ms)的固有延迟,对该级别的电流/速度环带宽可以忽略。
       本拍处理完就立即为下一拍启动新的非阻塞DMA读取,读完后由
       HAL_I2C_MemRxCpltCallback(AS5600.c)异步更新到 M0.total_angle_rad。 */
    raw_angle = AS5600_GetAngle(&M0);
    MechSpeed_Update(raw_angle);
    Mech_Angle = _normalizeAngle(raw_angle);
    Elec_Angle = 7.0f * Mech_Angle;

    AS5600_UpdateAngle_DMA(&M0);
}

/* 电流环: 只跑 D/Q 电流PID,不参与速度/位置级联。Id/Iq target 由上位机通过既有
 * PID写命令(二进制0x01/0x02步进 或 ASCII $WPID,ID/IQ,...#)直接设置。 */
static void FOC_CurrentLoop_Step(void)
{
    angle_proc();
    Current_read();
    Clarke_transform(&Iabc_M0,&Ialpbe_M0);
    Park_transform(&Iqd_M0,&Ialpbe_M0,Elec_Angle);

    Udq_M0.Ud = PID_Position_Calculate(&PID_Current_D,PID_Current_D.target,Iqd_M0.Id,FOC_LOOP_DT_S);
    Udq_M0.Uq = PID_Position_Calculate(&PID_Current_Q,PID_Current_Q.target,Iqd_M0.Iq,FOC_LOOP_DT_S);
    SVPWM(Elec_Angle, &Ualpbe_M0, &SVPWM_M0, &Udq_M0);
    /* 闭环路径的Ud/Uq来自PID输出,可能饱和到过调制区,tcm1/2/3不能保证落在[0,1],
       必须和开环一样过PWM_LimitCompare限幅,否则负值转uint16_t会变成一个极大的
       比较值,导致该相PWM在计数周期内卡死在全通/全断。 */
    PWM_TIM2_Set(PWM_LimitCompare(FOC_PWM_PERIOD*SVPWM_M0.tcm1),
                 PWM_LimitCompare(FOC_PWM_PERIOD*SVPWM_M0.tcm2),
                 PWM_LimitCompare(FOC_PWM_PERIOD*SVPWM_M0.tcm3));
}

/* 速度环: 速度PID输出作为Iq目标,再走电流环。SPD target 由上位机设置(PID_Speed.target)。 */
static void FOC_SpeedLoop_Step(void)
{
    angle_proc();
    Current_read();
    Clarke_transform(&Iabc_M0,&Ialpbe_M0);
    Park_transform(&Iqd_M0,&Ialpbe_M0,Elec_Angle);

    PID_Current_Q.target = -PID_Position_Calculate(&PID_Speed,PID_Speed.target,Mech_RPM,FOC_LOOP_DT_S);

    Udq_M0.Ud = PID_Position_Calculate(&PID_Current_D,PID_Current_D.target,Iqd_M0.Id,FOC_LOOP_DT_S);
    Udq_M0.Uq = PID_Position_Calculate(&PID_Current_Q,PID_Current_Q.target,Iqd_M0.Iq,FOC_LOOP_DT_S);
    SVPWM(Elec_Angle, &Ualpbe_M0, &SVPWM_M0, &Udq_M0);
    /* 闭环路径的Ud/Uq来自PID输出,可能饱和到过调制区,tcm1/2/3不能保证落在[0,1],
       必须和开环一样过PWM_LimitCompare限幅,否则负值转uint16_t会变成一个极大的
       比较值,导致该相PWM在计数周期内卡死在全通/全断。 */
    PWM_TIM2_Set(PWM_LimitCompare(FOC_PWM_PERIOD*SVPWM_M0.tcm1),
                 PWM_LimitCompare(FOC_PWM_PERIOD*SVPWM_M0.tcm2),
                 PWM_LimitCompare(FOC_PWM_PERIOD*SVPWM_M0.tcm3));
}

/* 位置环: 位置PID(角度误差按最短路径归一化)输出作为速度目标,再走速度环->电流环。
 * POS target 由上位机设置(PID_Position.target,单位: 机械角 rad,范围不限,内部按最短路径归一化)。 */
static void FOC_PositionLoop_Step(void)
{
    float wrapped_err;

    angle_proc();
    Current_read();
    Clarke_transform(&Iabc_M0,&Ialpbe_M0);
    Park_transform(&Iqd_M0,&Ialpbe_M0,Elec_Angle);

    wrapped_err = AngleErrorWrap(PID_Position.target - Mech_Angle);
    PID_Speed.target = PID_Position_Calculate(&PID_Position, Mech_Angle + wrapped_err, Mech_Angle, FOC_LOOP_DT_S);

    PID_Current_Q.target = -PID_Position_Calculate(&PID_Speed,PID_Speed.target,Mech_RPM,FOC_LOOP_DT_S);

    Udq_M0.Ud = PID_Position_Calculate(&PID_Current_D,PID_Current_D.target,Iqd_M0.Id,FOC_LOOP_DT_S);
    Udq_M0.Uq = PID_Position_Calculate(&PID_Current_Q,PID_Current_Q.target,Iqd_M0.Iq,FOC_LOOP_DT_S);
    SVPWM(Elec_Angle, &Ualpbe_M0, &SVPWM_M0, &Udq_M0);
    /* 闭环路径的Ud/Uq来自PID输出,可能饱和到过调制区,tcm1/2/3不能保证落在[0,1],
       必须和开环一样过PWM_LimitCompare限幅,否则负值转uint16_t会变成一个极大的
       比较值,导致该相PWM在计数周期内卡死在全通/全断。 */
    PWM_TIM2_Set(PWM_LimitCompare(FOC_PWM_PERIOD*SVPWM_M0.tcm1),
                 PWM_LimitCompare(FOC_PWM_PERIOD*SVPWM_M0.tcm2),
                 PWM_LimitCompare(FOC_PWM_PERIOD*SVPWM_M0.tcm3));
}

/* 曾经是OpenLoop_Control()内部的函数局部static,只在MCU复位后清零一次:第一次切到OPEN_LOOP
   正常从对齐阶段(tick=0)开始,但IDLE/其他模式切走再切回来时,这两个变量因为是static而不会重置,
   open_loop_tick还停在上次退出时的值(通常已经过了对齐+斜坡,ramp=1.0),导致第二次进入OPEN_LOOP
   会跳过500ms对齐直接以满幅Uq和上次遗留的电角度启动,造成明显的电流冲击/顿挫。改成模块级变量,
   配合下面OpenLoop_ResetState()在FOC_ModeDispatch检测到"切入OPEN_LOOP"的那一拍主动清零。 */
static uint32_t open_loop_tick = 0U;
static float open_loop_elec_angle = 0.0f;

static void OpenLoop_ResetState(void)
{
    open_loop_tick = 0U;
    open_loop_elec_angle = 0.0f;
}

static void OpenLoop_Control(void)
{
    uint16_t raw_adc[3] = {0U, 0U, 0U};
    float ramp = 1.0f;
    float elec_hz = 0.0f;

    Current_ReadRaw(raw_adc);
    ad_val_orig[0] = raw_adc[0];
    ad_val_orig[1] = raw_adc[1];
    ad_val_orig[2] = raw_adc[2];

    if ((open_loop_tick % OPEN_LOOP_UART_PERIOD_MS) == 0U)
    {
        UART2_SendRawAdc(open_loop_tick, raw_adc);
    }

    if (open_loop_tick < OPEN_LOOP_ALIGN_TIME_MS)
    {
        open_loop_elec_angle = 0.0f;
        Udq_M0.Ud = OpenLoop_AlignUd_V;
        Udq_M0.Uq = 0.0f;
        Mech_RPM = 0.0f;
    }
    else
    {
        uint32_t run_tick = open_loop_tick - OPEN_LOOP_ALIGN_TIME_MS;

        if (run_tick < OPEN_LOOP_RAMP_TIME_MS)
        {
            ramp = (float)run_tick / (float)OPEN_LOOP_RAMP_TIME_MS;
        }

        elec_hz = OpenLoop_TargetElecHz * ramp;
        Udq_M0.Ud = 0.0f;
        Udq_M0.Uq = OpenLoop_RunUq_V * ramp;
        open_loop_elec_angle += OPEN_LOOP_TWO_PI * elec_hz * FOC_LOOP_DT_S;
        open_loop_elec_angle = _normalizeAngle(open_loop_elec_angle);
        Mech_RPM = (elec_hz / OPEN_LOOP_POLE_PAIRS) * 60.0f;
    }

    Elec_Angle = open_loop_elec_angle;
    SVPWM(Elec_Angle, &Ualpbe_M0, &SVPWM_M0, &Udq_M0);
    PWM_TIM2_Set(PWM_LimitCompare(FOC_PWM_PERIOD * SVPWM_M0.tcm1),
                 PWM_LimitCompare(FOC_PWM_PERIOD * SVPWM_M0.tcm2),
                 PWM_LimitCompare(FOC_PWM_PERIOD * SVPWM_M0.tcm3));

    open_loop_tick++;

    if (open_loop_tick == 0U)
    {
        /* 溢出后从对齐阶段重新开始,避免长时间运行后 tick 溢出导致状态错乱 */
        open_loop_tick = 1U;
    }
}

/* PWM 归零(50%对称占空,三相电压差为0),用于IDLE及模式切换瞬间,避免残留占空比造成冲击 */
static void FOC_OutputZero(void)
{
    PWM_TIM2_Set((uint16_t)(FOC_PWM_PERIOD * 0.5f),
                 (uint16_t)(FOC_PWM_PERIOD * 0.5f),
                 (uint16_t)(FOC_PWM_PERIOD * 0.5f));
}

void FOC_ModeDispatch(void)
{
    static FOC_Mode_t prev_mode = FOC_MODE_IDLE;

    if (g_foc_mode != prev_mode)
    {
        PID_Reset(&PID_Current_D);
        PID_Reset(&PID_Current_Q);
        PID_Reset(&PID_Speed);
        PID_Reset(&PID_Position);
        FOC_OutputZero();
        if (g_foc_mode == FOC_MODE_OPEN_LOOP)
        {
            OpenLoop_ResetState();
        }
        prev_mode = g_foc_mode;
    }

    switch (g_foc_mode)
    {
        case FOC_MODE_OPEN_LOOP:
            OpenLoop_Control();
            break;
        case FOC_MODE_CURRENT:
            FOC_CurrentLoop_Step();
            break;
        case FOC_MODE_SPEED:
            FOC_SpeedLoop_Step();
            break;
        case FOC_MODE_POSITION:
            FOC_PositionLoop_Step();
            break;
        case FOC_MODE_IDLE:
        default:
            /* 空闲态仍刷新角度/电流用于监控,但不输出非零电压 */
            angle_proc();
            Current_read();
            FOC_OutputZero();
            break;
    }
}

void FOC_SetMode(uint8_t mode)
{
    if (mode > (uint8_t)FOC_MODE_POSITION)
    {
        return;
    }
    g_foc_mode = (FOC_Mode_t)mode;
}

uint8_t FOC_RequestRecalibration(void)
{
    if (g_foc_mode != FOC_MODE_IDLE)
    {
        return 0U;
    }
    Current_CalibrateOffset(512U);
    return 1U;
}

void Queue_proc(void)
{
    osMessageQueueGet(PIDQueueHandle, &Speed_pid, NULL, 0);
    PID_param_set(&PID_Speed,Speed_pid.kp,Speed_pid.ki,Speed_pid.kd);
}

void AngleTask_Entry(void *argument)
{
  for(;;)
  {
    osDelay(5);  // 100Hz 更新
  }
}
