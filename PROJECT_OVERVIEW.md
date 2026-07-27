# STM32_Sensorless_BLDC 项目概述（AI 写作参考文件）

> 本文件用于给 AI 写文档/报告时提供背景参考，内容基于对代码仓库的实际阅读整理，尽量只写"代码里能验证的事实"，不做推测性描述。更新日期：2026-07-27。

---

## 1. 项目是什么

一个基于 **STM32F407（Cortex-M4）+ Keil MDK-ARM + FreeRTOS** 的无刷直流电机（BLDC）**磁场定向控制（FOC）**固件工程。工程名 `STM32_Sensorless_BLDC` 虽然带"Sensorless"（无感），但**当前实际跑的是有感方案**：用 AS5600 磁编码器读机械角度做闭环，工程里保留了一份无感观测器（SMO，滑模观测器）代码但目前没有接入主控制流程（见第 9 节"已知问题"）。

固件同时集成了：
- 三档 FOC 控制环（电流环 / 速度环 / 位置环）+ 开环启动过程
- 一块 SPI TFT 屏幕的遥测数值显示，以及一个独立的 LVGL 动画演示模块
- 一颗 MPU9250 九轴 IMU 的读取代码（当前未启动对应任务，见第 4 节）
- 一套自定义的上位机 UART 通信协议（二进制帧 + ASCII 文本命令两种格式），用于实时调参、读遥测、模式切换

## 2. 硬件平台

| 项目 | 内容 |
|---|---|
| MCU | STM32F407（Cortex-M4F），启动文件 `startup_stm32f407xx.s` |
| 系统时钟 | PLL 由内部 **HSI 16MHz** 驱动（`main.c` `SystemClock_Config()`），PLLM=8, PLLN=168, PLLP=2 → **SYSCLK=168MHz**；AHB=168MHz，APB1=42MHz，APB2=84MHz（定时器时钟因预分频≠1翻倍，TIMx时钟=168MHz） |
| 角度传感器 | AS5600 磁编码器，接 **I2C2**（100kHz），驱动 `Bsp/AS5600.c` |
| 电流传感 | INA240A2QPWRQ1 电流检测放大器（增益50V/V，供电3.3V）+ 分流电阻，经 ADC1 **注入通道**（Injected Channel，3路同步采样）读取三相电流，驱动 `Bsp/current_sense.c` |
| IMU | MPU9250 九轴传感器（含DMP），驱动在 `Algorithm/util/inv_mpu*` + `Bsp/MPU9250-DMP.c`，通过 I2C1 |
| 显示 | SPI TFT 屏（引脚定义见 `main.h`: `TFT_RES/DC/CS` 在 GPIOF, `TFT_BL` 在 GPIOG），驱动 `Bsp/lcd.c` + `Bsp/lcd_init.c`；另外集成了 LVGL 图形库（`LVGL/` 目录，第三方库） |
| 上位机通信 | USART1（二进制/ASCII命令下行，DMA接收+IDLE中断分帧）、USART2（调试遥测输出，DMA发送） |
| 电机驱动 | 三相桥用 **TIM2** 中心对齐PWM（CH2/CH3/CH4，`FOC_PWM_PERIOD=3360`），母线电压 `Udc` 当前是**代码里写死的常量 12.6f**，没有实时ADC采样反馈（见第9节） |
| 母线电压 | 无独立采样通道，见"已知问题" |
| 其他外设 | CAN1、SPI1/SPI2、UART4/5、USART3/6 均有初始化代码但未在核心FOC/通信流程中使用（工程模板遗留） |

## 3. 关键定时器 / 中断资源分配

| 定时器/中断 | 用途 | 频率/周期 | 优先级(抢占,0最高) |
|---|---|---|---|
| **TIM10** | FOC 控制主循环入口（在 `HAL_TIM_PeriodElapsedCallback` 里调用 `FOC_ModeDispatch()`） | Prescaler=83, ARR=1000, TIMxCLK=168MHz → 计数时钟2MHz → **约2kHz（0.5ms一拍）** | 4（由 `HAL_TIM_Base_MspInit` 的 TIM10 分支设置，是该向量最终生效的值） |
| **TIM2** | 三相PWM输出，中心对齐模式，ARR=3360 | — | — |
| **TIM9** | 独立1kHz tick 计数（`TIM9_ISR_CNT`），未见承担控制逻辑 | ARR=1000, Prescaler=83 → 约2kHz | 5 |
| **TIM3** | 已启动中断但未见业务代码使用 | — | — |
| **I2C2_EV/ER** | AS5600 DMA读取完成回调 (`HAL_I2C_MemRxCpltCallback`) | — | 3（高于TIM10，DMA完成能抢占FOC ISR） |
| **USART1** | 上位机下行指令帧（DMA接收 + IDLE中断分帧，`stm32f4xx_it.c: USART1_IRQHandler`） | — | 5 |
| **USART2** | 上位机遥测/调试输出（DMA发送） | — | 5 |
| **IWDG** | 独立看门狗（2026-07-27新增，寄存器级直接配置，未走HAL模块），约500ms超时，在TIM10 ISR每拍喂狗 | — | 独立时钟域(LSI) |

FOC 控制环完全跑在 **TIM10 硬件中断**里，不受 FreeRTOS 任务调度和任务优先级影响；这是本工程一个核心设计前提，多处注释里都强调了这一点。

## 4. 软件架构：FreeRTOS 任务清单

`Core/Src/freertos.c` 里实际创建的任务（`MX_FREERTOS_Init()`）：

| 任务 | 优先级 | 栈大小 | 状态 | 作用 |
|---|---|---|---|---|
| `LcdTask_Entry` | Low | 4KB | **运行中** | 每150ms刷新一次TFT屏上的FOC遥测数值（模式/RPM/角度/三相电流），受 `g_lcd_enable` 开关控制 |
| `LvglTimerTask_Entry` | High | 8KB | **运行中** | 每5ms驱动一次LVGL的tick/timer_handler，受 `g_lvgl_demo_enable` 开关控制，和LcdTask互斥使用同一块LCD/SPI总线 |
| `UARTTask_Entry` | High | 4KB | **运行中** | 解析上位机下行帧、处理遥测推流(`$TEL`)/调试推流(`$SPD`)节拍 |
| `IMU9250Task_Entry` | Normal | 4KB | **未创建**（`freertos.c:151`被注释掉） | 读取MPU9250 DMP姿态角，通过队列`IMUQueueHandle`发布 |
| `AngleTask_Entry` | Normal | 2KB | **未创建**（`freertos.c:153`被注释掉），函数体本身也只是空转`osDelay(5)` | 预留 |

另有4个消息队列：`IMUQueueHandle`(8×IMU_Euler_t)、`PIDQueueHandle`(6×PID_Param_t)、`FOCQueueHandle`(8×FOC_Data_t，当前无生产者/消费者代码引用)、`UARTQueueHandle`(1×UART_Frame_t)。

**重要**：IMU 相关任务当前未启动，MPU9250 的初始化调用只存在于 `App/Init_file.c` 的 `Init_All()`（同样未被调用，`main.c:295`被注释）。也就是说**当前固件实际运行时不做IMU姿态解算**，相关代码是保留但未激活的功能分支。写报告时如果涉及"多传感器融合/姿态感知"，需要明确这部分目前是未启用状态，不要当成在跑的功能来描述。

## 5. 目录结构速查

```
App/                    应用层任务代码
  foc_task.c/.h          FOC模式分发与四种控制环(IDLE/开环/电流/速度/位置)
  uart_task.c/.h          上位机通信协议解析与遥测输出
  lcd_task.c/.h            TFT遥测数值显示任务
  lvgl_demo_task.c/.h      LVGL动画演示任务(独立于lcd_task,互斥共用LCD)
  IMU_task.c/.h            MPU9250读取任务(当前未启动)
  Init_file.c/inc/init_file.h  遗留初始化封装(当前未调用,与IMU_task内容重复)

Bsp/                    板级驱动/算法层
  foc_drv.c/.h            Clarke/Park变换、SVPWM、PWM输出、角度归一化
  pid.c/.h                通用PID(位置式+增量式两种实现)
  current_sense.c/.h       三相电流ADC注入采样、零点标定
  AS5600.c/.h              磁编码器I2C(阻塞/DMA两种读取方式)驱动
  SMO.c/.h, Algorithmic.c/.h   无感观测器(SMO/PLL)与通用滤波器算法库,
                                **当前未被foc_task.c或main.c调用,是死代码**
  lcd.c/lcd_init.c/lcdfont.c   TFT屏底层驱动(SPI+DMA)
  MPU9250-DMP.c/.h         MPU9250 DMP驱动

Algorithm/util/          MPU9250相关的第三方/移植驱动(InvenSense DMP库、I2C适配层)

Core/Src, Core/Inc       STM32CubeMX生成的HAL初始化代码(main.c/tim.c/adc.c/...)
                          + FreeRTOS胶水代码(freertos.c)
                          + 中断向量实现(stm32f4xx_it.c)

Drivers/                 STM32 HAL库 + CMSIS(含DSP库arm_math,Park/Clarke变换里用到的
                          arm_sin_f32/arm_cos_f32就来自这里)

LVGL/                    LVGL图形库源码(第三方,未做业务修改)

Middlewares/              ST中间件(FreeRTOS内核源码等)

upmachine/                上位机侧Python脚本(与UART协议配套的PC端工具)

MDK-ARM/                 Keil工程文件(.uvprojx等),本仓库的实际编译工程
```

## 6. FOC 控制核心链路

固件定义5种运行模式（`FOC_Mode_t`，`App/inc/foc_task.h`）：

```
FOC_MODE_IDLE = 0       // 上电默认,PWM归零(50%对称占空,三相压差为0),角度/电流仍在采样
FOC_MODE_OPEN_LOOP = 1  // 开环强拖启动: 先对齐(500ms)再按线性ramp升频(3000ms)到目标频率
FOC_MODE_CURRENT = 2    // 只跑Id/Iq电流环,target由上位机直接写入
FOC_MODE_SPEED = 3      // 速度环输出作为Iq target,再走电流环
FOC_MODE_POSITION = 4   // 位置环(角度误差按最短路径归一化)输出速度target,再走速度环→电流环
```

`FOC_ModeDispatch()`（`App/foc_task.c`）是 TIM10 中断里每拍调用的总入口，按 `g_foc_mode` 分发；模式切换瞬间会调用 `PID_Reset()` 清空四个PID(电流D/Q、速度、位置)的积分/误差历史，并把PWM打回50%占空，避免残留积分导致输出突跳。

每一拍闭环控制的处理链路（以速度环为例）：

```
angle_proc()                          // 取上一拍DMA读回的AS5600角度,顺带发起下一拍新的DMA读取
  → Mech_Angle(归一化到0~2π), Elec_Angle = 7×Mech_Angle (7 = 极对数 OPEN_LOOP_POLE_PAIRS)
Current_read()                        // 读ADC注入通道三相电流,换算成安培
Clarke_transform → Park_transform     // abc → αβ → dq
PID_Position_Calculate(速度PID)        // 输出作为 PID_Current_Q.target
PID_Position_Calculate(Id/Iq电流PID)   // 输出 Udq_M0.Ud / Udq_M0.Uq
SVPWM()                                // 逆Park变换 + 扇区判断 + 七段式SVPWM占空比计算
PWM_TIM2_Set(PWM_LimitCompare(...)×3)  // 写入TIM2 CCR,2026-07-27已补上限幅(见第9节)
```

- **PID**：位置式实现（`Bsp/pid.c: PID_Position_Calculate`），2026-07-27前的版本积分项是"每次调用固定加一次error"，隐含假设调用频率恒定；现已改成显式传入 `dt` 参数，按 `error*dt` 累加积分、`(error-lastError)/dt` 做微分，Ki/Kd 是标准的"每秒"物理量。当前 `FOC_LOOP_DT_S = 0.0005f`（对应TIM10实测2kHz），三处闭环函数和 `MechSpeed_Update`/开环电角度积分共用这一个宏，改动TIM10时钟配置时必须同步改这里。
- **角度采集**：`AS5600.c` 提供阻塞（`AS5600_UpdateAngle`）和DMA（`AS5600_UpdateAngle_DMA` + `HAL_I2C_MemRxCpltCallback`异步回填）两种读取方式。当前 `angle_proc()` 用的是DMA方式：本拍先取"上一拍已经读回"的角度值使用，用完立即为下一拍发起新的非阻塞DMA读取，避免阻塞I2C读(约0.4~0.5ms)占满2kHz(0.5ms)控制周期，代价是引入约一拍的角度延迟。
- **开环启动**（`OpenLoop_Control`）：先500ms对齐（固定Ud电压把转子拉到已知电角度0点），再3000ms内电角频率从0线性爬升到目标频率，同时Uq电压同比例爬升，用于无编码器反馈情况下的强拖启动。
- **PWM限幅**：`PWM_LimitCompare()`（`Bsp/foc_drv.c`）把浮点占空比钳到`[0, FOC_PWM_PERIOD]`再转uint16_t。2026-07-27之前只有开环路径调用了它，三个闭环函数是直接强转，已在同日修复(详见第9节)。

## 7. 电流采样细节（重要，涉及硬件标定）

- ADC1 使用**注入通道**（Injected Channel）同步采3路：`InjectedRank1→ADC1_IN2(PA2)`、`Rank2→ADC1_IN3(PA3)`、`Rank3→ADC1_IN4(PA4)`，采样时间28周期。
- 相序映射（`Current_ReadRaw`）：`raw_adc[0](Ia)←Rank3`, `raw_adc[1](Ib)←Rank2`, `raw_adc[2](Ic)←Rank1`。**代码注释里作者明确写这是未经硬件最终确认的假设**，需要上板验证。
- 换算系数 `CURRENT_ADC_TO_AMP`（`Bsp/inc/current_sense.h`）基于：ADC参考电压3.3V、12位满量程4096、INA240增益50V/V、分流电阻**假定5mΩ**（注释称原理图丝印标1mΩ与实物不符，以实物5mΩ为准，同样需要现场核实）。
- 零点标定：`Current_CalibrateOffset(sample_count)` 多次采样取平均作为新零点，若偏离默认值 `CURRENT_OFFSET_DEFAULT_ADC(2048)` 超过 `CURRENT_OFFSET_FAULT_TOL_ADC(600)`，判定该相模拟前端异常，不采信标定结果，置位 `Diag_CurrentFault` 对应bit。只允许在 `FOC_MODE_IDLE` 下通过UART命令 `0x87 RECALIBRATE` 触发。

## 8. 上位机通信协议（`App/uart_task.c`）

USART1 是命令下行通道，DMA接收 + 空闲线中断（IDLE）分帧，支持两种帧格式：

**1) 二进制帧**（4字节固定格式 `0xFE 0xEF <device> <arg0> <arg1> 0x23 0x24`，帧头`FE EF`/帧尾`23 24`）：由状态机 `UART_ProcessInTimer()` 解析，`device`字段对应一组预定义命令字：

| 命令字 | 含义 |
|---|---|
| 0x80 | 读遥测(单次) |
| 0x81/0x82 | 开启/关闭遥测周期推流(`$TEL`) |
| 0x83 | 读四个PID参数 |
| 0x84 | 读遥测+PID+调试计数 |
| 0x85 | 读调试计数(`$DBG`,TIM9/TIM10中断计数) |
| 0x86 | 读原始ADC(`$RAW`) |
| 0x87 | 重新标定电流零点(仅IDLE模式) |
| 0x90 | 设置FOC模式 |
| 0x91 | DISARM(强制切回IDLE) |
| 0x93/0x94 | 开关LCD遥测显示 / LVGL动画演示 |
| 0x00~0x08 | 各PID(Id/Iq/Speed/Position)及开环调试参数(对齐电压/运行电压/目标频率)的步进式增减调节，步长由`data[2]`按10^-n换算 |

**2) ASCII 文本帧**（以`$`开头，`#`结尾，逗号分隔，帧头识别为`rx1_frame_buffer[0]=='$'`）：

- `$WPID,<ID|IQ|SPD|POS>,<target>,<kp>,<ki>,<kd>#` — 一次性写某个PID的目标值和三个增益
- `$MODE,<n>#` — 设置FOC模式
- `$LCD,<0|1>#` / `$LVGL,<0|1>#` — 开关LCD遥测 / LVGL演示

USART2 是调试输出通道：`$SPD,rpm,mech_angle,elec_angle#`（每约100ms一次，固定`osDelay(1)`节拍下计数触发）和开环模式下的`$ADC,tick,ia,ib,ic#`原始ADC推流（DMA发送，忙标志防止重入）。

遥测输出统一走 `printf`（重定向到 USART1，见 `main.c: PUTCHAR_PROTOTYPE`），格式如 `$TEL,...#`、`$PID,ID,...#`、`$RAW,...#`。

`upmachine/` 目录下是配套的PC端Python脚本，实现了这套协议的上位机侧。

## 9. 已知问题 / 近期修复记录

详见仓库根目录 `CODE_REVIEW_REPORT.md`（2026-07-27 完整代码审查报告）。截至本文档更新时，已完成修复：

- ✅ 三个闭环控制函数（电流/速度/位置环）PWM输出补上 `PWM_LimitCompare()` 限幅，此前只有开环路径有限幅，闭环路径PID饱和时存在负浮点转uint16_t导致PWM卡死全通/全断的风险。
- ✅ 新增独立看门狗（IWDG，寄存器级实现，约500ms超时，TIM10 ISR每拍喂狗），此前工程完全没有看门狗。
- ✅ NMI/HardFault/MemManage/BusFault/UsageFault 五个异常入口新增 `FOC_EmergencyPWMOff()`，直接清零TIM2 CH2/3/4输出使能位，此前异常发生后CPU死循环但PWM仍按最后一次占空比继续输出。
- ✅ 修复共享中断向量 `TIM1_UP_TIM10_IRQn` 的NVIC优先级被设置两次（`tim.c`里TIM1和TIM10各设一次，靠初始化顺序偶然覆盖出正确值）的隐患。
- ✅ 修复 `Bsp/SMO.c: PLL_SMO()` 里锁相环积分器 `I_Partern` 未加 `static` 导致读取未初始化栈内存的问题（该文件当前是死代码，暂不影响运行，但修复后如果以后要接入无感方案不会一开始就是坏的）。

**尚待处理/需要硬件确认的问题**（未在代码里擅自改动）：

- `Udc`（母线电压，`main.c` 里是写死常量 `12.6f`）没有ADC实时采样，SVPWM调制深度计算依赖这个假设值是否准确。
- 电流采样的ADC通道→物理相序映射、分流电阻实际阻值（假定5mΩ）均需上板实测核实。
- 系统时钟用内部HSI而非外部晶振，控制环dt/UART波特率/I2C时钟精度依赖HSI稳定性。
- `tim.c` 里NVIC优先级的修复位于CubeMX生成代码区（未被`USER CODE`标记保护），如果以后重新用CubeMX生成代码，这处修复可能被覆盖，需要重新检查。

## 10. 全局关键变量/数据结构速查

| 符号 | 定义位置 | 说明 |
|---|---|---|
| `g_foc_mode` (`volatile FOC_Mode_t`) | `foc_task.c` | 当前FOC运行模式，UART命令0x90/0x91或`$MODE`写入 |
| `PID_Current_D/Q`, `PID_Speed`, `PID_Position` (`PIDController`) | `main.c` | 四级PID实例，初始限幅：电流环±4.0，速度环±15.0，位置环±15.0；初始增益仅电流环有非零kp=0.0517，速度/位置环kp/ki/kd初值为0，需上位机在线整定 |
| `Mech_Angle`/`Elec_Angle`/`Mech_RPM` | `foc_task.c`声明,`main.c`定义 | 机械角(0~2π)、电角度(=7×机械角)、机械转速(RPM) |
| `Iabc_M0`/`Ialpbe_M0`/`Iqd_M0`/`Udq_M0`/`Ualpbe_M0`/`SVPWM_M0` | 类型定义在`main.h`,实例在`main.c` | 三相电流/αβ电流/dq电流/dq电压/αβ电压/SVPWM中间量，"M0"后缀表示"电机0"(单电机工程,预留多电机扩展命名) |
| `Diag_RawAdc/Diag_RawVolt/Diag_CurrentFault` | `current_sense.c` | 电流采样诊断量，供上位机`$RAW`/`$TEL`读取 |
| `ad_val_orig[3]`/`current_adc_offset[3]` | `current_sense.c` | 三相原始ADC计数与零点偏置 |
| `M0` (`AS5600`) | `main.c`定义,`AS5600.h`声明类型 | AS5600编码器实例，挂在`hi2c2` |
| `Udc` | `main.c` | 母线电压，写死12.6f，见第9节 |

## 11. 构建环境

- IDE/工具链：**Keil MDK-ARM**（`.uvprojx`工程文件在 `MDK-ARM/`），编译器为 ARM Compiler（AC5/AC6，具体见工程配置）
- RTOS：FreeRTOS（通过CMSIS-RTOS2 `cmsis_os2`接口调用，`osThreadNew`/`osMessageQueueNew`等）
- 图形库：LVGL（第三方，`LVGL/`目录，未做业务定制修改）
- DSP库：CMSIS-DSP（`arm_sin_f32`/`arm_cos_f32`用于Park/逆Park变换）
- 本环境（本次审查/修复所在环境）未安装Keil命令行工具（`UV4.exe`不可用），无法自动化编译验证，修改后需要在实际Keil环境里编译确认。
