# FOC 算法 & 通用算法函数库 —— 可移植模块说明

> 本文件从 `STM32_Sensorless_BLDC` 工程中提炼出 FOC 控制算法本体，以及和电机控制无关、可直接拿去别的项目用的
> 通用算法小模块（PID、滤波器、边沿检测等）。涉及源文件：`Bsp/foc_drv.c/.h`、`Bsp/pid.c/.h`、
> `Bsp/current_sense.c/.h`、`Bsp/Algorithmic.c/.h`、`Bsp/SMO.c/.h`（附录）、`App/foc_task.c`（应用层editor组装范例）。
> 数值型宏（增益、限幅值、分流电阻阻值等）是**本工程的具体标定值**，移植到新硬件必须重新标定，不能直接照抄数值。

---

## 1. 模块依赖关系一览

```
foc_task.c (应用层：模式状态机 + 每拍调用顺序编排)
   ├─ AS5600.c            角度传感器读取(阻塞/DMA两种)
   ├─ current_sense.c     三相电流ADC注入采样 + 零点标定
   ├─ foc_drv.c            Clarke/Park/逆Park + SVPWM + PWM输出 + 角度工具函数
   └─ pid.c                通用PID控制器(位置式/增量式) + 简易一阶低通

Algorithmic.c   通用滤波器/数字逻辑小工具库，与FOC无耦合，可单独移植到任意工程
SMO.c           无感BEMF滑模观测器 + 锁相环，当前工程未启用(死代码)，作为参考实现放在附录
```

四个核心数据结构（`Core/Inc/main.h`），移植时整体拷贝：

```c
typedef struct { float Ia, Ib, Ic; }              Iabc_Struct;    // 三相电流
typedef struct { float I_alpha, I_beta; }         Ialpbe_Struct;  // αβ电流
typedef struct { float Id, Iq; }                  Iqd_Struct;     // dq电流
typedef struct { float Uq, Ud; }                  Udq_Struct;     // dq电压
typedef struct { float U_alpha, U_beta; }         Ualpbe_Struct;  // αβ电压
typedef struct {                                                   // SVPWM中间量
    int sector;
    float U1,U2,U3, Ux,Uy,Uz, ta,tb,tc, Ts,t1,t2,t0,t7, tcm1,tcm2,tcm3;
} SVPWM_Struct;
```

---

## 2. PID 模块（`Bsp/pid.c` / `Bsp/inc/pid.h`）

```c
typedef struct {
    float kp, ki, kd;
    float error, lastError, preError;
    float integral, output;
    float maxOutput, minOutput, maxIntegral;
    float target;
} PIDController;
```

### 2.1 位置式PID（推荐，本工程电流/速度/位置三级环都用它）

```c
float PID_Position_Calculate(PIDController *pid, float target, float current, float dt)
{
    pid->error = target - current;

    pid->integral += pid->error * dt;                 // 积分按真实时间累加，不是"每调用一次+1次"
    if (pid->integral > pid->maxIntegral)  pid->integral = pid->maxIntegral;   // 抗积分饱和
    if (pid->integral < -pid->maxIntegral) pid->integral = -pid->maxIntegral;

    float derivative = (dt > 0.0f) ? ((pid->error - pid->lastError) / dt) : 0.0f;  // dt<=0视为无效调用

    pid->output = pid->kp*pid->error + pid->ki*pid->integral + pid->kd*derivative;
    if (pid->output >= pid->maxOutput) pid->output = pid->maxOutput;              // 输出限幅
    if (pid->output <= pid->minOutput) pid->output = pid->minOutput;

    pid->lastError = pid->error;
    return pid->output;
}
```

**设计要点（移植时务必保留）**：显式传入 `dt`，积分项按 `error*dt` 累加、微分项按 `(error-lastError)/dt` 计算，这样 `Ki`/`Kd` 是标准的"每秒"物理量，**调用频率变化时不需要重新折算增益**——这是本工程从"固定调用频率隐含假设"重构过来的经验教训，任何新工程只要控制环节拍可能变化（调试时降频、不同型号MCU主频不同等），都建议直接照抄这个 `dt` 显式传参的写法，而不是自己再重新发明一遍隐式频率假设的PID。

对应的初始化/复位/写参数三件套：

```c
void PID_Init(PIDController *pid, float maxOutput, float minOutput, float maxIntegral);
// kp/ki/kd清零，limit三件套按调用时传入的值设置，error/integral/output清零

void PID_param_set(PIDController *pid, float kp, float ki, float kd);
// 只改增益，不碰limit/target/历史状态——运行时热调参用这个

void PID_Reset(PIDController *pid);
// 只清 error/lastError/preError/integral/output 历史状态，不改增益/限幅/target
// 用在"模式切换/重新使能"瞬间，避免残留积分导致输出突跳
```

### 2.2 增量式PID（备选，`PID_Increment_Calculate`）

```c
increment = kp*(error - lastError) + ki*error + kd*(error - 2*lastError + preError);
output += increment;   // 输出是"累加"而不是"整体重算"，天然自带抗积分饱和的味道(限幅只夹output本身)
```

适合执行机构本身是"增量式"的场景（比如步进电机/舵机角度增量指令），本工程未使用但保留了实现，移植时按被控对象类型二选一。

### 2.3 简易一阶低通（`First_order_Filtering`）

```c
float First_order_Filtering(float input) {
    static float output_last;
    float output = 0.3f*output_last + 0.7f*input;   // 固定系数版一阶IIR，够用但系数写死不可配置
    output_last = output;
    return output;
}
```
如果需要系数可调，改用第5节 `Algorithmic.c` 里的 `Low_Pass_Filter(last, new, alpha)`。

---

## 3. Clarke / Park / SVPWM（`Bsp/foc_drv.c`）

### 3.1 坐标变换

```c
// Clarke: abc -> αβ (等幅值变换，1/√3 = 0.57735026919f)
I_alpbe->I_alpha = I_abc->Ia;
I_alpbe->I_beta  = (I_abc->Ib - I_abc->Ic) * 0.57735026919f;

// Park: αβ -> dq，用 CMSIS-DSP 的 arm_sin_f32/arm_cos_f32 (查表法，比 sinf/cosf 快)
float32_t s = arm_sin_f32(angle), c = arm_cos_f32(angle);
I_dq->Id =  I_alpbe->I_alpha*c + I_alpbe->I_beta*s;
I_dq->Iq = -I_alpbe->I_alpha*s + I_alpbe->I_beta*c;

// 逆Park: dq -> αβ，用于把PID算出来的电压指令转回定子坐标系
U_alphaBeta->U_alpha = U_dq->Ud*c - U_dq->Uq*s;
U_alphaBeta->U_beta  = U_dq->Ud*s + U_dq->Uq*c;
```
移植提示：`arm_sin_f32`/`arm_cos_f32` 来自 CMSIS-DSP（`arm_math.h`），需要工程里挂上 `arm_cortexM4lf_math.lib`（或对应型号的DSP库）并 `#define ARM_MATH_CM4`（按MCU型号）。如果目标平台没有CMSIS-DSP，退化用标准库 `sinf`/`cosf` 即可，只是运算稍慢。

### 3.2 SVPWM（七段式，扇区判断 + 过调制处理）

核心公式（`Udc` 是母线电压，`Ts` 归一化为1）：

```c
K = 1.73205080756f * Ts / Udc;                 // √3 * Ts / Udc
Ux = U_beta * K;
Uy = (0.8660254f*U_alpha + 0.5f*U_beta) * K;
Uz = (-0.8660254f*U_alpha + 0.5f*U_beta) * K;

// 扇区号 = 4*z + 2*y + x，其中 x/y/z 分别是 U1=U_beta, U2=√3/2·Uα-Uβ/2, U3=-√3/2·Uα-Uβ/2 的符号位
```

扇区判断得到 `sector∈{1..6}` 后查表取 `t1/t2`（六个扇区各自的组合关系见源码 `SVPWM_timer_period_set`），然后：

```c
if (t1+t2 > Ts) {                       // 过调制：矢量作用时间超过一个开关周期
    k = Ts / (t1+t2);
    t1 *= k;
    t2 = Ts - t1;
}
ta = (Ts - t1 - t2) / 4.0f;
tb = ta + t1/2.0f;
tc = tb + t2/2.0f;
// 再按扇区号把 ta/tb/tc 映射到 tcm1/tcm2/tcm3(三相占空比)，最后 ×2 得到中心对齐PWM的实际比较值比例
```

调用顺序封装成一个函数：

```c
void SVPWM(float Ele_angle, Ualpbe_Struct *Uab, SVPWM_Struct *svpwm, Udq_Struct *Udq) {
    inverseParkTransform(Udq, Uab, Ele_angle);
    svpwm_sector_choice(svpwm, Uab);
    SVPWM_timer_period_set(svpwm, Uab);
}
```

### 3.3 PWM 输出与限幅（**易漏的坑**）

```c
uint16_t PWM_LimitCompare(float compare) {
    if (compare < 0.0f) return 0U;
    if (compare > FOC_PWM_PERIOD) return (uint16_t)FOC_PWM_PERIOD;
    return (uint16_t)compare;
}
```

**必须在写定时器比较寄存器之前对 `tcm1/2/3` 过一遍这个限幅**：闭环PID的输出可能饱和到过调制区，导致 `tcm` 不落在 `[0,1]`；如果不限幅直接强转 `uint16_t`，负的浮点数转出来是一个巨大的正整数，会让该相PWM在计数周期内卡死在全通或全断——本工程曾经只有开环路径做了限幅、三条闭环路径忘了做，是一次实际修复过的bug，移植时三种模式（开环/闭环/任何新增模式）都要统一过这一层。

### 3.4 角度工具函数

```c
float _normalizeAngle(float angle);   // fmod 到 [0, 2π)
float AngleErrorWrap(float error);    // 把任意范围的角度差 wrap 到 [-π, π]，位置环跨0/2π边界走最短路径必用
```

---

## 4. FOC 模式状态机设计模式（`App/foc_task.c`）

这是"应用层怎么把上面这些算法零件拼起来"的编排范例，与具体电机无关，模式本身可复用：

```c
typedef enum { MODE_IDLE, MODE_OPEN_LOOP, MODE_CURRENT, MODE_SPEED, MODE_POSITION } Mode_t;

void ModeDispatch(void) {
    static Mode_t prev = MODE_IDLE;
    if (g_mode != prev) {                 // 切模式瞬间：清空所有PID历史状态 + 输出打回安全值
        PID_Reset(&pid_d); PID_Reset(&pid_q); PID_Reset(&pid_speed); PID_Reset(&pid_pos);
        OutputZero();
        prev = g_mode;
    }
    switch (g_mode) {
        case MODE_OPEN_LOOP: OpenLoopStep();  break;
        case MODE_CURRENT:   CurrentLoopStep(); break;   // 只跑Id/Iq环
        case MODE_SPEED:     SpeedLoopStep();  break;    // 速度环输出 -> Iq target -> 电流环
        case MODE_POSITION:  PositionLoopStep(); break;  // 位置环 -> 速度target -> 速度环 -> 电流环
        default: /* IDLE */ AngleAndCurrentRefreshOnly(); OutputZero(); break;
    }
}
```

- **级联环的写法**：外环算出的 `output` 直接赋给内环的 `target`（如 `PID_Current_Q.target = -PID_Position_Calculate(&PID_Speed, ...)`），每一级都是独立的 `PIDController` 实例，级联层数按需增减。
- **模式切换必须清历史状态**：否则残留的积分/lastError会在新模式下造成第一拍输出突跳。
- **每一拍的标准流程**（电流环为最小闭环单元）：`读角度 → 读电流 → Clarke → Park → PID(电流) → 逆Park+SVPWM → PWM限幅输出`；速度环/位置环只是在最前面多插一到两级PID。
- **开环强拖启动**（`OpenLoop_Control`，无编码器时的常见兜底方案）：先固定时长"对齐"（给定一个固定Ud电压把转子拉到已知电角度0点），再固定时长内电角频率从0线性爬升到目标频率、Uq电压同比例爬升，纯开环跑完整个SVPWM链路但不需要角度反馈。
- **dt 一致性**：级联的每一级 `PID_Position_Calculate` 调用、`Mech_RPM` 微分计算、开环电角度积分，必须共用**同一个** `dt` 常量（本工程是 `FOC_LOOP_DT_S`，由控制中断的实际周期决定），移植时如果改了控制中断频率，这一个常量要跟着改，且要检查是否被多处引用。

---

## 5. 电流采样 + 零点标定套路（`Bsp/current_sense.c`）

通用模式，不含本工程具体的运放增益/分流电阻数值：

```c
#define ADC_TO_AMP  (ADC_REF_VOLTAGE / ADC_FULL_SCALE / (AMP_GAIN * SHUNT_RESISTOR_OHM))
// 三相电流放大器输出经ADC采到的count值，换算成安培的通用公式模板

void Current_read(void) {
    Current_ReadRaw(ad_val_orig);                       // 读ADC注入通道原始count
    for (ch) Iabc.x[ch] = (ad_val_orig[ch] - offset[ch]) * ADC_TO_AMP;   // 减零点偏置再换算
}

void Current_CalibrateOffset(uint16_t sample_count) {
    // 多次采样取平均作为新零点；若新零点偏离默认值(理论上应为ADC满量程一半)超过容忍阈值，
    // 判定该相模拟前端异常，不采信标定结果，只置故障标志位，零点仍用默认值
}
```

**移植要点**：
1. ADC 用**注入通道（Injected Channel）多路同步采样**是三相电流采样的标准做法，避免相位/时间偏差。
2. 零点标定要有"标定结果合理性校验"（本工程用"偏离默认值±阈值"判断），不能盲目信任一次标定结果——采样电路虚焊/损坏时标定出来的零点会离谱，不做校验会导致后续电流值整体漂移但没有任何报错。
3. **ADC通道到物理相序的映射、放大器增益、分流电阻实际阻值，每个新硬件都必须重新核实**，本工程注释里明确写了这几处都是"假设值，需上板验证"，是最容易踩坑、也最需要在移植checklist里第一件事就确认的地方。

---

## 6. 通用滤波器 / 数字逻辑小工具库（`Bsp/Algorithmic.c` / `Algorithmic.h`）

这一组函数**与FOC完全无耦合**，可以整体拷贝到任何需要"数字信号处理小零件"的工程：

```c
// 滑动平均滤波（固定窗口环形缓冲）
typedef struct { uint16_t buffer[ADC_FILTER_SIZE]; uint8_t index,count; uint32_t sum; bool is_full; } ADC_Filter_t;
void     ADC_Filter_Init(ADC_Filter_t *f);
uint16_t ADC_Moving_Average_Filter(ADC_Filter_t *f, uint16_t new_value);   // O(1)增量式，不用每次重新求和

// 一阶低通(IIR)，系数alpha可调，越小滤波越强
float Low_Pass_Filter(float last_value, uint16_t new_value, float alpha) {
    return alpha*new_value + (1.0f-alpha)*last_value;
}

// 中位值滤波(冒泡排序取中间，适合小窗口去脉冲干扰)
uint16_t ADC_Median_Filter(uint16_t *buffer, uint8_t size);

// 一维卡尔曼滤波(标量版，Q=过程噪声 R=测量噪声 P=估计误差协方差)
typedef struct { float Q,R,P,K,x; } Kalman_Filter_t;
void  Kalman_Filter_Init(Kalman_Filter_t *kf, float Q, float R, float P, float initial_value);
float Kalman_Filter_Update(Kalman_Filter_t *kf, float measurement) {
    kf->P += kf->Q;                                   // 预测
    kf->K = kf->P / (kf->P + kf->R);                  // 增益
    kf->x += kf->K * (measurement - kf->x);            // 更新估计
    kf->P = (1 - kf->K) * kf->P;                       // 更新协方差
    return kf->x;
}

// 边沿检测器 / T触发器 —— 按键消抖、脉冲计数、状态翻转类需求的通用小零件
uint8_t Edge_Detector_Update(Edge_Detector_t *e, uint8_t signal);   // 返回 bit0=上升沿 bit1=下降沿
uint8_t T_FlipFlop_Update(T_FlipFlop_t *t, uint8_t clk, uint8_t T); // clk上升沿且T=1时翻转Q_state
```

选型建议（移植时按噪声特性挑一个，不是越复杂越好）：
- 只是想去掉高频抖动、对相位延迟不敏感 → 一阶低通，参数最少，最省资源。
- 需要精确"过去N个采样的平均值" → 滑动平均。
- 干扰是偶发脉冲尖峰（而不是持续噪声）→ 中位值滤波，均值类滤波器对脉冲无效。
- 有明确的过程/测量噪声模型、需要动态调整信任度 → 卡尔曼滤波，计算量比前三种都大。

---

## 7. 附录：无感BEMF观测器（`Bsp/SMO.c`，当前工程未启用）

`Bsp/SMO.c` 实现了一版滑模观测器(SMO)估计反电动势 + 锁相环(PLL)估算电角度/转速，**当前未接入 `foc_task.c` 主流程，是保留但未激活的代码**，移植前必须重新验证正确性，不要直接当成可信实现搬走。已知情况：

- 核心思路：用电流预测误差过一个类似继电特性的"滑模面"函数 `Vab = h*(预测电流-实际电流)`（限幅在±h内），低通滤波后作为反电动势估计的等效输入，再用 PLL（`PLL_SMO`）把估计出的反电动势相位跟踪出来，得到估算电角度 `Theta_fore_New` 和角速度 `We_fore`。
- 依赖大量**外部全局变量**（`R, Ld, Lq, T, flux, PLL_Kp, PLL_Ki` 等电机参数和PLL增益），拷贝这份代码时必须把这些参数按新电机重新辨识，不能沿用本工程数值。
- 曾经修复过一个bug：`PLL_SMO()` 里的积分器 `I_Partern` 原来是**函数内的普通局部变量**（没有 `static`），每次调用都在读栈上的垃圾初值，导致积分永远无法真正累加、锁相环形同虚设——现已加上 `static`。这类"积分器/滤波器状态必须跨调用persist"的坑在滤波器/观测器类代码里非常常见，移植任何"看起来是纯函数、实际内部有状态累积"的算法时都要重点检查这一点。
- 建议：如果新工程要做真正的无感方案，把这份代码当"算法思路参考"而不是"可直接编译使用的模块"，重新过一遍电机参数辨识、积分器初始化、限幅阈值的取值逻辑。

---

## 8. 移植步骤 Checklist

1. **数据结构**：拷贝 `Iabc/Ialpbe/Iqd/Udq/Ualpbe/SVPWM_Struct/PIDController` 六个结构体定义。
2. **PID**：拷贝 `pid.c/.h` 整体，不用改；`FOC_LOOP_DT_S`（或等价的dt常量）按新工程控制环频率重新定义。
3. **坐标变换+SVPWM**：拷贝 `foc_drv.c/.h`；确认目标MCU是否有CMSIS-DSP，没有就把 `arm_sin_f32/arm_cos_f32` 换成 `sinf/cosf`；`FOC_PWM_PERIOD` 按新工程定时器ARR重新定义；`Udc` 按新工程母线电压获取方式（写死常量 or 实时ADC采样）处理。
4. **电流采样**：按新硬件的放大器增益/分流电阻/ADC参考电压重新算 `ADC_TO_AMP` 系数；确认ADC通道到物理相序的映射需要上板验证。
5. **模式状态机**：按新工程实际需要的运行模式定义枚举和切换逻辑，保留"切模式清PID历史+输出归零"这条规则。
6. **PWM限幅**：确认所有会写定时器比较寄存器的路径（开环/每一种闭环模式）都过了 `PWM_LimitCompare` 或等价限幅，这是最容易漏改的一处。
7. **通用滤波器**：`Algorithmic.c/.h` 可整体原样拷贝，按第6节选型建议直接用。
8. **无感观测器（可选）**：仅在需要做无感方案时才引入 `SMO.c`，引入前重新验证第7节提到的坑。
