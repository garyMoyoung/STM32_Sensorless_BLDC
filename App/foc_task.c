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
#include "SMO.h"

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

/* 速度环目标转速斜坡限幅: 防止上位机突然给一个大的目标转速阶跃时,
 * 速度PID瞬间要求电流环冲很大的Iq去追赶,导致电源电流瞬间超限。
 * 实际喂给速度PID的目标值(speed_target_ramped)每个控制周期只朝着
 * PID_Speed.target靠近有限的一步,相当于给目标转速做了软启动。 */
#define SPEED_TARGET_RAMP_RPM_PER_S      50.0f
static float speed_target_ramped = 0.0f;

#define MOTOR_ELECTRICAL_ANGLE_DIRECTION (-1.0f)
#define ELECTRICAL_ALIGN_UD_V            0.25f
#define ELECTRICAL_ALIGN_TICKS            1000U
#define CURRENT_ADC_SATURATION_MARGIN    8U

typedef enum
{
    ELECTRICAL_ALIGN_IDLE = 0,
    ELECTRICAL_ALIGN_RUNNING,
    ELECTRICAL_ALIGN_DONE,
    ELECTRICAL_ALIGN_FAILED
} ElectricalAlignState_t;

static volatile ElectricalAlignState_t electrical_align_state = ELECTRICAL_ALIGN_IDLE;
static volatile uint32_t electrical_align_tick = 0U;
static volatile uint8_t electrical_align_result_pending = 0U;
static float electrical_angle_offset = 0.0f;
static uint8_t electrical_angle_aligned = 0U;
static volatile uint8_t current_saturation_trip_pending = 0U;
static uint16_t current_saturation_trip_raw[3] = {0U, 0U, 0U};

static void FOC_OutputZero(void);

#define OPEN_LOOP_ALIGN_TIME_MS          500U
#define OPEN_LOOP_RAMP_TIME_MS           3000U
#define OPEN_LOOP_ALIGN_UD_V_DEFAULT     1.0f
#define OPEN_LOOP_RUN_UQ_V_DEFAULT       1.2f
#define OPEN_LOOP_TARGET_ELEC_HZ_DEFAULT 5.0f
#define OPEN_LOOP_POLE_PAIRS             7.0f
#define OPEN_LOOP_UART_PERIOD_MS         50U
#define OPEN_LOOP_TWO_PI                 6.283185307f

volatile FOC_Mode_t g_foc_mode = FOC_MODE_IDLE;

float OpenLoop_AlignUd_V = OPEN_LOOP_ALIGN_UD_V_DEFAULT;
float OpenLoop_RunUq_V = OPEN_LOOP_RUN_UQ_V_DEFAULT;
float OpenLoop_TargetElecHz = OPEN_LOOP_TARGET_ELEC_HZ_DEFAULT;

static float prev_speed_angle = 0.0f;
static uint8_t prev_speed_valid = 0U;
#define MECH_RPM_LPF_ALPHA 0.1f
static float Mech_RPM_Filtered = 0.0f;

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
    /* AS5600原始角度存在量化噪声,0.5ms这么短的差分窗口会把这点抖动放大成
       转速读数上的高频毛刺(哪怕目标斜坡很平缓,PID看到的瞬时误差也会因为
       这些毛刺忽大忽小,Kp一乘就把Iq目标顶向限幅,表现为电源电流反复冲顶)。
       这里对Mech_RPM做一阶低通滤波后再供PID使用,只影响反馈平滑度,
       不引入模式切换突变问题(不满足PID_Reset那类复位场景)。 */
    Mech_RPM_Filtered = MECH_RPM_LPF_ALPHA * Mech_RPM + (1.0f - MECH_RPM_LPF_ALPHA) * Mech_RPM_Filtered;
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
    Elec_Angle = _normalizeAngle(MOTOR_ELECTRICAL_ANGLE_DIRECTION *
                                  MOTOR_POLE_PAIRS * Mech_Angle +
                                  electrical_angle_offset);

    AS5600_UpdateAngle_DMA(&M0);
}

static uint8_t CurrentSample_IsSaturated(void)
{
    uint8_t ch;

    for (ch = 0U; ch < 3U; ch++)
    {
        if ((ad_val_orig[ch] <= CURRENT_ADC_SATURATION_MARGIN) ||
            (ad_val_orig[ch] >= (4095U - CURRENT_ADC_SATURATION_MARGIN)))
        {
            current_saturation_trip_raw[0] = ad_val_orig[0];
            current_saturation_trip_raw[1] = ad_val_orig[1];
            current_saturation_trip_raw[2] = ad_val_orig[2];
            current_saturation_trip_pending = 1U;
            return 1U;
        }
    }
    return 0U;
}

static void FOC_AbortToIdle(void)
{
    PID_Reset(&PID_Current_D);
    PID_Reset(&PID_Current_Q);
    PID_Reset(&PID_Speed);
    PID_Reset(&PID_Position);
    PID_Current_D.target = 0.0f;
    PID_Current_Q.target = 0.0f;
    g_foc_mode = FOC_MODE_IDLE;
    FOC_OutputZero();
}

static void FOC_ElectricalAlignment_Step(void)
{
    angle_proc();
    Current_read();

    if (CurrentSample_IsSaturated() != 0U)
    {
        electrical_align_state = ELECTRICAL_ALIGN_FAILED;
        electrical_align_result_pending = 1U;
        FOC_OutputZero();
        return;
    }

    Udq_M0.Ud = ELECTRICAL_ALIGN_UD_V;
    Udq_M0.Uq = 0.0f;
    Elec_Angle = 0.0f;
    SVPWM(Elec_Angle, &Ualpbe_M0, &SVPWM_M0, &Udq_M0);
    PWM_TIM2_Set(PWM_LimitCompare(FOC_PWM_PERIOD * SVPWM_M0.tcm1),
                 PWM_LimitCompare(FOC_PWM_PERIOD * SVPWM_M0.tcm2),
                 PWM_LimitCompare(FOC_PWM_PERIOD * SVPWM_M0.tcm3));

    electrical_align_tick++;
    if (electrical_align_tick >= ELECTRICAL_ALIGN_TICKS)
    {
        electrical_angle_offset = _normalizeAngle(-MOTOR_ELECTRICAL_ANGLE_DIRECTION *
                                                   MOTOR_POLE_PAIRS * Mech_Angle);
        electrical_angle_aligned = 1U;
        electrical_align_state = ELECTRICAL_ALIGN_DONE;
        electrical_align_result_pending = 1U;
        FOC_OutputZero();
    }
}

/* 电流环: 只跑 D/Q 电流PID,不参与速度/位置级联。Id/Iq target 由上位机通过既有
 * PID写命令(二进制0x01/0x02步进 或 ASCII $WPID,ID/IQ,...#)直接设置。 */
static void FOC_CurrentLoop_Step(void)
{
    angle_proc();
    Current_read();
    if (CurrentSample_IsSaturated() != 0U)
    {
        FOC_AbortToIdle();
        return;
    }
    Clarke_transform(&Iabc_M0,&Ialpbe_M0);
    Park_transform(&Iqd_M0,&Ialpbe_M0,Elec_Angle);

    Udq_M0.Ud = PID_Position_Calculate(&PID_Current_D,PID_Current_D.target,Iqd_M0.Id,FOC_LOOP_DT_S);
    Udq_M0.Uq = PID_Position_Calculate(&PID_Current_Q,PID_Current_Q.target,Iqd_M0.Iq,FOC_LOOP_DT_S);
    SVPWM(Elec_Angle, &Ualpbe_M0, &SVPWM_M0, &Udq_M0);
    /* SMO影子估算: 只喂真实电流/电压/角度进去做估算,不接管上面已经算好的PWM占空比 */
    SMO_ShadowUpdate(Ialpbe_M0.I_alpha, Ialpbe_M0.I_beta,
                      Ualpbe_M0.U_alpha, Ualpbe_M0.U_beta, Elec_Angle);
    /* 闭环路径的Ud/Uq来自PID输出,可能饱和到过调制区,tcm1/2/3不能保证落在[0,1],
       必须和开环一样过PWM_LimitCompare限幅,否则负值转uint16_t会变成一个极大的
       比较值,导致该相PWM在计数周期内卡死在全通/全断。 */
    PWM_TIM2_Set(PWM_LimitCompare(FOC_PWM_PERIOD*SVPWM_M0.tcm1),
                 PWM_LimitCompare(FOC_PWM_PERIOD*SVPWM_M0.tcm2),
                 PWM_LimitCompare(FOC_PWM_PERIOD*SVPWM_M0.tcm3));
}

/* 每周期把speed_target_ramped向真实目标PID_Speed.target靠近有限一步,
 * 步长由SPEED_TARGET_RAMP_RPM_PER_S和控制周期决定,实现目标转速软启动。 */
static float SpeedTargetRamp_Update(float target)
{
    float step = SPEED_TARGET_RAMP_RPM_PER_S * FOC_LOOP_DT_S;
    float diff = target - speed_target_ramped;

    if (diff > step)
    {
        speed_target_ramped += step;
    }
    else if (diff < -step)
    {
        speed_target_ramped -= step;
    }
    else
    {
        speed_target_ramped = target;
    }
    return speed_target_ramped;
}

/* 速度环: 速度PID输出作为Iq目标,再走电流环。SPD target 由上位机设置(PID_Speed.target)。 */
static void FOC_SpeedLoop_Step(void)
{
    float speed_target_now;

    angle_proc();
    Current_read();
    if (CurrentSample_IsSaturated() != 0U)
    {
        FOC_AbortToIdle();
        return;
    }
    Clarke_transform(&Iabc_M0,&Ialpbe_M0);
    Park_transform(&Iqd_M0,&Ialpbe_M0,Elec_Angle);

    speed_target_now = SpeedTargetRamp_Update(PID_Speed.target);
    /* 这里必须保持和机械正转方向/电流 dq 正定向一致：
       正转时希望产生正的Iq扭矩，速度误差为正则 PID_Speed.output 也应为正，
       不能再额外乘一个 '-'，否则在 forward/negative 两个象限中会被强制反向
       施力，表现为电源负载急剧冲顶/机械方向相反、速度环上下振荡。 */
    PID_Current_Q.target = PID_Position_Calculate(&PID_Speed,speed_target_now,Mech_RPM_Filtered,FOC_LOOP_DT_S);

    Udq_M0.Ud = PID_Position_Calculate(&PID_Current_D,PID_Current_D.target,Iqd_M0.Id,FOC_LOOP_DT_S);
    Udq_M0.Uq = PID_Position_Calculate(&PID_Current_Q,PID_Current_Q.target,Iqd_M0.Iq,FOC_LOOP_DT_S);
    SVPWM(Elec_Angle, &Ualpbe_M0, &SVPWM_M0, &Udq_M0);
    /* SMO影子估算: 只喂真实电流/电压/角度进去做估算,不接管上面已经算好的PWM占空比 */
    SMO_ShadowUpdate(Ialpbe_M0.I_alpha, Ialpbe_M0.I_beta,
                      Ualpbe_M0.U_alpha, Ualpbe_M0.U_beta, Elec_Angle);
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
    if (CurrentSample_IsSaturated() != 0U)
    {
        FOC_AbortToIdle();
        return;
    }
    Clarke_transform(&Iabc_M0,&Ialpbe_M0);
    Park_transform(&Iqd_M0,&Ialpbe_M0,Elec_Angle);

    wrapped_err = AngleErrorWrap(PID_Position.target - Mech_Angle);
    PID_Speed.target = PID_Position_Calculate(&PID_Position, Mech_Angle + wrapped_err, Mech_Angle, FOC_LOOP_DT_S);

    PID_Current_Q.target = PID_Position_Calculate(&PID_Speed,SpeedTargetRamp_Update(PID_Speed.target),Mech_RPM_Filtered,FOC_LOOP_DT_S);

    Udq_M0.Ud = PID_Position_Calculate(&PID_Current_D,PID_Current_D.target,Iqd_M0.Id,FOC_LOOP_DT_S);
    Udq_M0.Uq = PID_Position_Calculate(&PID_Current_Q,PID_Current_Q.target,Iqd_M0.Iq,FOC_LOOP_DT_S);
    SVPWM(Elec_Angle, &Ualpbe_M0, &SVPWM_M0, &Udq_M0);
    /* SMO影子估算: 只喂真实电流/电压/角度进去做估算,不接管上面已经算好的PWM占空比 */
    SMO_ShadowUpdate(Ialpbe_M0.I_alpha, Ialpbe_M0.I_beta,
                      Ualpbe_M0.U_alpha, Ualpbe_M0.U_beta, Elec_Angle);
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

    if (electrical_align_state == ELECTRICAL_ALIGN_RUNNING)
    {
        FOC_ElectricalAlignment_Step();
        return;
    }

    if (g_foc_mode != prev_mode)
    {
        PID_Reset(&PID_Current_D);
        PID_Reset(&PID_Current_Q);
        PID_Reset(&PID_Speed);
        PID_Reset(&PID_Position);
        /* PID_Reset只清运行状态(error/integral/output),不清target。之前在current模式
           手动测试时可能给PID_Current_D.target设过非零值(理论上D轴电流任何模式都应为0),
           如果切到speed/position模式后不清零,这个残留的非零Id目标会让电流环持续去追一个
           不该有的励磁电流分量,叠加到Iq上,是"刚切到speed模式转速还很低电流就冲顶"、以及
           "Id和Iq都同时偏到+5A左右"的直接原因。这里统一清零,确保每次进入新模式都是干净的。 */
        PID_Current_D.target = 0.0f;
        if (g_foc_mode != FOC_MODE_CURRENT)
        {
            PID_Current_Q.target = 0.0f;
        }
        speed_target_ramped = Mech_RPM; /* 切换到速度/位置模式时从当前实际转速开始斜坡,避免突变 */
        Mech_RPM_Filtered = Mech_RPM; /* 同步重置滤波器初值,避免残留旧值导致误差瞬间跳变 */
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
    if ((mode >= (uint8_t)FOC_MODE_CURRENT) && (electrical_angle_aligned == 0U))
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

uint8_t FOC_RequestElectricalAlignment(void)
{
    if ((g_foc_mode != FOC_MODE_IDLE) || (electrical_align_state == ELECTRICAL_ALIGN_RUNNING))
    {
        return 0U;
    }

    PID_Reset(&PID_Current_D);
    PID_Reset(&PID_Current_Q);
    PID_Reset(&PID_Speed);
    PID_Reset(&PID_Position);
    PID_Current_D.target = 0.0f;
    PID_Current_Q.target = 0.0f;
    electrical_angle_aligned = 0U;
    electrical_align_tick = 0U;
    electrical_align_state = ELECTRICAL_ALIGN_RUNNING;
    electrical_align_result_pending = 0U;
    return 1U;
}

uint8_t FOC_IsElectricalAlignmentValid(void)
{
    return electrical_angle_aligned;
}

uint8_t FOC_TakeElectricalAlignmentResult(float *offset_rad)
{
    if (electrical_align_result_pending == 0U)
    {
        return 0U;
    }

    electrical_align_result_pending = 0U;
    if (offset_rad != NULL)
    {
        *offset_rad = electrical_angle_offset;
    }
    return (electrical_align_state == ELECTRICAL_ALIGN_DONE) ? 1U : 2U;
}

uint8_t FOC_TakeCurrentSaturationTrip(uint16_t raw_adc[3])
{
    if (current_saturation_trip_pending == 0U)
    {
        return 0U;
    }

    raw_adc[0] = current_saturation_trip_raw[0];
    raw_adc[1] = current_saturation_trip_raw[1];
    raw_adc[2] = current_saturation_trip_raw[2];
    current_saturation_trip_pending = 0U;
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
