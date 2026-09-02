# FOC 串口读取协议

## 串口参数

- 波特率：115200
- 数据位：8
- 停止位：1
- 校验：None
- 下行命令帧：7 字节固定长度（二进制），或以 `$` 开头、`#` 结尾的 ASCII 行命令

## 调试模式说明

固件支持 4 种运行时可切换的调试模式（`FOC_Mode_t`，通过 `0x90`/`0x91`/ASCII `$MODE` 切换，
无需重新编译烧录）：

| 模式值 | 名称 | 说明 |
|---:|---|---|
| 0 | IDLE | 默认模式，上电即处于该状态。只刷新角度/电流用于监控，PWM 恒为 50% 占空（三相零压差），电机不受力 |
| 1 | OPEN_LOOP | 开环强拖：先按 `Ud` 对齐，再按设定电角频率斜坡加速旋转，不使用编码器反馈闭环 |
| 2 | CURRENT | 电流环：只跑 Id/Iq 电流 PID，Id/Iq 目标值直接由上位机设置 |
| 3 | SPEED | 电流环 + 速度环级联：速度 PID 输出作为 Iq 目标 |
| 4 | POSITION | 位置环 + 速度环 + 电流环三级级联：位置 PID（角度误差按最短路径归一化到 `[-π,π]`）输出作为速度目标 |

模式切换的瞬间，固件会自动清空 D/Q/速度/位置四个 PID 的积分与历史误差，并把 PWM 输出瞬间拉回
50% 零压差，避免残留积分/占空比造成冲击电流或转速突变。

**安全须知**：
- 切换到新模式前建议先发送 `0x91`（DISARM）回到 IDLE，确认参数设置好后再切到目标模式。
- `0x87`（RECALIBRATE）只允许在 IDLE 模式下执行，防止把电机运行中的工作点误当作零电流基准。
- 各环目标值复用已有的 PID 写入命令：电流环 Id/Iq 用 `ID`/`IQ`，速度环用 `SPD`，位置环用 `POS`
  （见下方 PID 帧与 `$WPID` 命令）。位置环 `POS` 的 `target` 单位是机械角 rad，取值不限，固件内部
  按最短路径归一化，跨 0/2π 边界不会反转。
- 本机实测机械正 RPM 对应负 Iq，因此速度环内部已自动转换该符号关系；速度 PID 输出（即 Iq 目标）
  被限制为 `±1.5 A`。
- 切入 `SPEED` 模式后，若速度目标非零，固件先按目标方向施加 `0.5 A` Iq、持续 300 ms，使转子
  脱离静摩擦后再交给速度 PID。交接时速度斜坡从当前实测转速继续，避免产生反向制动；速度目标为零时
  不会施加该起动电流。

## LCD 遥测屏幕

板载 LCD（ST7789，240x280，`Bsp/lcd.c`/`Bsp/lcd_init.c` 的 `_DMA` 系列函数经 SPI1+DMA 驱动）
在 `App/lcd_task.c` 里显示当前模式、转速(RPM)、机械角度(度)、三相电流 Ia/Ib/Ic，每 150ms 刷新一次
数值区域（标签只画一次，不整屏重绘）。这条链路完全独立于 FOC 控制（FOC 在 TIM10 硬件中断里跑，
不受任务调度影响），LCD 任务本身优先级也调到了 UART 任务之下，不会抢占上位机通信。

默认开机即显示（`g_lcd_enable` 默认 1），可通过 `0x93`（二进制）或 ASCII `$LCD,n#` 运行时开关：
关闭时背光熄灭且不再触碰 SPI 总线。

## 直流有刷电机（第二台电机，独立于主 BLDC）

`Bsp/dc_motor.c`：项目里的第二台电机，正交编码器测速/测位置（TIM4 Encoder Mode，PD12/PD13，
4 倍频计数；线数按 `Bsp/dc_motor.c` 里 `DC_MOTOR_ENCODER_LINES_PER_REV` 改成实际编码器规格），
H 桥双 PWM 驱动（TIM3 CH1=IN1/PA6、CH2=IN2/PA7，TB6612/DRV8833 式接线：IN1 给 PWM、IN2 拉低
=正转，IN1 拉低、IN2 给 PWM=反转，PWM 频率 20kHz）。控制节拍复用 BLDC 同一颗 TIM10 定时器，
和主 BLDC 互不干扰、可同时独立运行。

三种模式（`DCMotor_Mode_t`，通过 `0x95`/ASCII `$DCMODE` 切换，和主 BLDC 的 `g_foc_mode` 完全独立）：

| 模式值 | 名称 | 说明 |
|---:|---|---|
| 0 | IDLE | 默认模式，PWM 占空为 0（自由滑行） |
| 1 | SPEED | 速度环：PID 输出直接就是占空比 `[-1,1]` |
| 2 | POSITION | 位置环 + 速度环级联：位置 PID（角度误差按最短路径归一化）输出作为速度目标 |

目标值/增益复用已有的 PID 写入命令：速度环用 `DCSPD`，位置环用 `DCPOS`（见下方 PID 帧与
`$WPID` 命令），`DCPOS` 的 `target` 单位是机械角 rad，取值不限，固件内部按最短路径归一化。
模式切换瞬间会自动清空 `DCSPD`/`DCPOS` 的积分与历史误差、把占空比拉回 0，避免冲击。

## 上位机 -> 下位机

### 二进制帧（默认路径）

帧格式：

```text
FE EF CMD ARG0 ARG1 23 24
```

字段说明：

- `FE EF`：帧头
- `CMD`：命令字
- `ARG0`：参数 0
- `ARG1`：参数 1
- `23 24`：帧尾，即 ASCII `#` `$`

### ASCII 行命令（可选，供串口终端手动调试）

以 `$` 开头、`#` 结尾的一行文本，例如：

```text
$WPID,IQ,2.0,0.05,0.001,0#
$MODE,3#
$LCD,0#
$SAVEPID#
$LOADPID#
$DCMODE,1#
$WPID,DCSPD,500,0.01,0.001,0#
```

## 命令表

| CMD | 名称 | ARG0 | ARG1 | 说明 |
|---:|---|---:|---:|---|
| `0x80` | 读取一次遥测 | 0 | 0 | 返回一帧 `$TEL` |
| `0x81` | 开始周期上传 | 周期 ms | 0 | `ARG0=0` 时默认 50 ms |
| `0x82` | 停止周期上传 | 0 | 0 | 停止 `$TEL` 周期输出 |
| `0x83` | 读取 PID | 0 | 0 | 返回 4 行 `$PID` |
| `0x88` | 保存 PID 到 Flash | 0 | 0 | 把当前 4 路 PID(含 target)写入片内 Flash，仅 IDLE 模式下有效 |
| `0x89` | 从 Flash 加载 PID | 0 | 0 | 用 Flash 里保存的参数覆盖当前 4 路 PID，仅 IDLE 模式下有效，返回 `$PID` x4 |
| `0x84` | 读取全部 | 0 | 0 | 返回 `$TEL` + `$PID` + `$DBG` |
| `0x85` | 读取调试计数 | 0 | 0 | 返回 `$DBG` |
| `0x86` | 读取原始 ADC | 0 | 0 | 返回一帧 `$RAW`（原始计数/电压/标定偏置/故障位） |
| `0x87` | 重新标定电流零点 | 0 | 0 | 仅 IDLE 模式下有效，返回 `$ACK,CAL#` + `$RAW`，忙时返回 `$ERR,CAL,BUSY#` |
| `0x92` | 电角度对齐 | 0 | 0 | 仅 IDLE 模式下有效。以 0.8V d 轴电压锁定 500ms 后计算 AS5600 电角度偏置；先返回 `$ACK,ALIGN,START#`，完成后返回 `$ACK,ALIGN,DONE,<offset_rad>#`。采样饱和则返回 `$ERR,ALIGN,ADC_SATURATED#` |
| `0x90` | 设置运行模式 | 模式值(0~4) | 0 | 返回 `$ACK,MODE,<n>#` |
| `0x91` | 断电/复位为 IDLE | 0 | 0 | 等价于 `0x90 0` |
| `0x93` | LCD 显示开关 | 0/1 | 0 | 返回 `$ACK,LCD,<n>#`，关闭时背光熄灭且不再刷新 |
| `0x95` | 设置直流电机模式 | 模式值(0~2) | 0 | 返回 `$ACK,DCMODE,<n>#`，和主BLDC模式(`0x90`)相互独立 |
| `0x01` | 调节 Id 环 PID/target | 见下 | 步进档位 | `ARG1`: `0x01/0x11`=kp±，`0x02/0x12`=ki±，`0x03/0x13`=kd±，`0x04/0x14`=target± |
| `0x02` | 调节 Iq 环 PID/target | 见下 | 步进档位 | 同上 |
| `0x03` | 快速调节 Iq target | - | `0x01`=+2A，`0x02`=-2A | 粗调 |
| `0x04` | 调节速度环 PID/target | 见下 | 步进档位 | 同 `0x01` |
| `0x05` | 调节位置环 PID/target | 见下 | 步进档位 | 同 `0x01`，target 单位 rad |
| `0x06` | 调节开环对齐电压 Ud | `0x01/0x11`=±step | 步进档位 | 只在 OPEN_LOOP 模式生效 |
| `0x07` | 调节开环运行电压 Uq | `0x01/0x11`=±step | 步进档位 | 只在 OPEN_LOOP 模式生效 |
| `0x08` | 调节开环目标电角频率 | `0x01/0x11`=±step | 步进档位 | 只在 OPEN_LOOP 模式生效 |

`ARG1`（对 0x01/0x02/0x04/0x05/0x06/0x07/0x08）低字节表示步进档位，换算为步进值
`step = 10^(-ARG1)`（例如 `ARG1=3` 对应步进 0.001）。

## 下位机 -> 上位机

### 遥测帧

```text
$TEL,Ia,Ib,Ic,Id,Iq,RPM,MechAngle,ElecAngle,IdKp,IdKi,IdKd,IqKp,IqKi,IqKd,SpeedKp,SpeedKi,SpeedKd,PosKp,PosKi,PosKd,IqTarget,SpeedTarget,Mode,RawA,RawB,RawC,VoltA,VoltB,VoltC,Fault,LcdEnable,DcAngle,DcRPM,DcDuty,DcMode,SmoTheta,SmoWe,SmoRPM,SmoEa,SmoEb,SmoVfA,SmoVfB#\r\n
```

新增字段说明：

- `Mode`：当前 `FOC_Mode_t`（0~4，见上表）
- `RawA/RawB/RawC`：三相电流采样 ADC 原始计数（0~4095），对应 Ia/Ib/Ic
- `VoltA/VoltB/VoltC`：对应引脚换算后的电压（V）
- `Fault`：电流零点异常位掩码，bit0/1/2 分别对应 A/B/C 相；置位表示该相标定出的零点偏离 2048 计数超过约 0.48V，怀疑该相模拟前端（INA240 供电/参考电压/接线）有问题
- `LcdEnable`：LCD 遥测屏幕当前是否开启（0/1）
- `DcAngle`/`DcRPM`：直流有刷电机（第二台电机）的机械角(rad)/转速(RPM)，编码器实测值
- `DcDuty`：直流电机当前PWM占空比，`[-1,1]`，正负表示转向
- `DcMode`：直流电机当前 `DCMotor_Mode_t`（0~2，见上表），和 `Mode` 字段（主BLDC）互相独立
- `SmoTheta`：滑模观测器（SMO，`Bsp/SMO.c`）估算的电角度（rad，0~2π），和 `ElecAngle`（AS5600实测）同单位可直接对比
- `SmoWe`：SMO 估算的电角速度（rad/s）
- `SmoRPM`：`SmoWe` 换算成机械 RPM，和 `RPM` 字段同单位可直接对比
- `SmoEa`/`SmoEb`：SMO 估算的反电动势 alpha/beta 分量
- `SmoVfA`/`SmoVfB`：SMO 内部滑模面低通滤波后的值（PLL 输入），用于判断信号是否干净/是否饱和

闭环前必须执行一次 `0x92` 电角度对齐。未完成对齐时，固件会拒绝进入 `CURRENT`、`SPEED` 与
`POSITION` 模式并返回 `$ERR,MODE,NOT_ALIGNED#`。对齐过程中转子会被锁定，请先空载、限流并确认
电机可安全转动。闭环运行中若任一相电流 ADC 读数进入 `0~8` 或 `4087~4095`，固件会立即回到
`IDLE`，避免使用饱和失真的电流反馈继续驱动。

注意：SMO 目前只是"影子"估算（`App/foc_task.c` 里 `SMO_ShadowUpdate()`），用真实电流/电压/AS5600角度
喂给观测器做估算，但**不参与**实际 PID 闭环输出，纯粹用于和 `ElecAngle`/`RPM` 对比调试用。

`DcAngle`/`DcRPM`/`DcDuty`/`DcMode`/`SmoTheta`/`SmoWe`/`SmoRPM`/`SmoEa`/`SmoEb`/`SmoVfA`/`SmoVfB`
都是追加在末尾的新字段，`upmachine/foc_uart_host.py` 的 `parse_tel()` 按精确字段数校验，新增字段
必须和固件的 `$TEL` 输出同步，两边字段数对不上会导致整行解析失败。

### 原始 ADC 帧

```text
$RAW,adcA,voltA,offA,adcB,voltB,offB,adcC,voltC,offC,fault#\r\n
```

- `adcX`：当前原始 ADC 计数
- `voltX`：换算电压
- `offX`：当前使用的零电流偏置（标定值，若该相 fault 则仍为默认值 2048）
- `fault`：同 `$TEL` 里的 Fault 字段

**排查建议（拿到实物后）**：只接 3.3V 逻辑电、不接 12V 母线时读 `$RAW`，三相应接近 `2048` 计数 /
`1.65V` 附近（若明显偏离甚至到 0 或满量程，说明 MCU 侧供电/ADC 引脚有问题，与 12V 母线无关）；
接上 12V 并执行 `0x87` 重新标定后再读 `$RAW`，仍应接近零点，若某相 `fault=1` 或电压长期钉在某个
固定值（如接近 3.3V 或 0V），基本可判断是该相 INA240 供电/参考电压/走线的问题，而非固件换算公式
问题（固件换算公式已核实与历史提交等价，多次代码审查未发现回归）。

### PID 帧

```text
$PID,NAME,Kp,Ki,Kd,Target,Output,Integral#\r\n
```

`NAME` 取值：

- `ID`：d 轴电流环（主BLDC）
- `IQ`：q 轴电流环（主BLDC）
- `SPD`：速度环（主BLDC）
- `POS`：位置环（主BLDC）
- `DCSPD`：速度环（直流电机）
- `DCPOS`：位置环（直流电机）

### 调试计数帧

```text
$DBG,TIM9_ISR_CNT,TIM10_ISR_CNT#\r\n
```

### 应答/错误帧

```text
$ACK,MODE,<n>#\r\n        // 主BLDC模式切换成功
$ACK,DCMODE,<n>#\r\n      // 直流电机模式切换成功
$ACK,CAL#\r\n             // 重新标定成功
$ACK,ALIGN,START#\r\n     // 电角度对齐已启动
$ACK,ALIGN,DONE,<rad>#\r\n // 电角度偏置已计算并应用
$ACK,OL_UD,<v>#\r\n       // 开环对齐电压已更新
$ACK,OL_UQ,<v>#\r\n       // 开环运行电压已更新
$ACK,OL_HZ,<v>#\r\n       // 开环目标电角频率已更新
$ACK,PID,<loop>,...#\r\n  // PID 参数已写入（见现有 $WPID 说明）
$ACK,LCD,<n>#\r\n         // LCD显示开关已更新
$ACK,PIDSAVE#\r\n         // PID参数已写入Flash
$ACK,PIDLOAD#\r\n         // PID参数已从Flash加载(随后跟4行$PID)
$ERR,CAL,BUSY#\r\n        // 非IDLE模式下请求重新标定，被拒绝
$ERR,ALIGN,BUSY#\r\n      // 非IDLE模式下请求电角度对齐，被拒绝
$ERR,ALIGN,ADC_SATURATED#\r\n // 对齐时检测到相电流 ADC 饱和
$ERR,MODE,NOT_ALIGNED#\r\n // 尚未完成电角度对齐，拒绝闭环模式
$ERR,PIDSAVE,BUSY#\r\n    // 非IDLE模式下请求保存PID，被拒绝
$ERR,PIDSAVE,FAIL#\r\n    // Flash擦除/编程失败
$ERR,PIDLOAD,BUSY#\r\n    // 非IDLE模式下请求加载PID，被拒绝
$ERR,PIDLOAD,EMPTY#\r\n   // Flash里没有保存过有效PID参数(magic/校验和不匹配)
$ERR,PID,UNKNOWN_LOOP#\r\n
$ERR,UNKNOWN_CMD#\r\n
```

## PID 参数持久化（Flash）

固件默认的 PID 增益只是编译期写死的初值，掉电/复位后会丢失上位机现场调好的参数。
现支持把当前 6 路 PID（主BLDC的Id/Iq/速度/位置 + 直流电机的速度/位置，含各自 `target`）保存进
片内 Flash 最后一个扇区（`Bsp/pid_storage.c`，STM32F407xE 512KB 器件的 Sector 7，地址
`0x08060000`），下次上电时 `main()` 会自动尝试加载：Flash 里有校验通过的数据就覆盖编译期默认值，
没有（第一次用/校验失败）则保持默认值不受影响。加入直流电机两路PID后 Flash 数据格式版本从
"PID1"升到"PID2"，改动前烧录过的旧格式数据magic对不上，会被当成无效数据丢弃，不会误解析。

- `0x88`/`$SAVEPID#`：保存当前 6 路 PID 到 Flash。
- `0x89`/`$LOADPID#`：从 Flash 重新加载 6 路 PID（会覆盖当前正在使用的参数）。

**限制**：保存/加载要求主BLDC(`g_foc_mode`)和直流电机(`g_dc_motor_mode`)**同时**处于 `IDLE`
才允许执行（原因见下方"安全须知"），任一台不是 IDLE 都会返回 `$ERR,PIDSAVE,BUSY#` /
`$ERR,PIDLOAD,BUSY#`。

**安全须知（重要）**：保存操作要擦除一个 128KB 的 Flash 扇区，STM32F4 是单 bank 器件，擦除/编程
期间 CPU 无法从 Flash 取指，最坏情况下会卡住 CPU 长达约 2 秒——两台电机的控制节拍
（`FOC_ModeDispatch`/`DCMotor_ControlTick`）共用同一颗 TIM10 硬件中断，只要其中一台还在转，
这颗中断被整体延迟都可能导致失步、过流或直流电机顿挫。因此保存/加载前必须用 `0x91`（DISARM，
主BLDC）和 `0x95` 传 0（直流电机）分别切回 IDLE，确认两台电机都已停转再操作。

## 硬件相序映射（待与实物核实）

`Bsp/current_sense.c` 里 `Current_ReadRaw()` 当前假设的映射关系：

| 逻辑相 | ADC 注入序列 | STM32 引脚 |
|---|---|---|
| Ia | InjectedRank3 | PA4 (ADC1_IN4) |
| Ib | InjectedRank2 | PA3 (ADC1_IN3) |
| Ic | InjectedRank1 | PA2 (ADC1_IN2) |

这是代码里现有的假设，尚未与原理图/实物核实。如果拿到板子后发现某一相电流方向或相序不对，
只需调整 `Current_ReadRaw()` 里的下标映射，不影响 Clarke/Park/SVPWM 等其余算法。

## 直流电机引脚分配（待与实物核实）

固件加直流电机支持时，`.ioc` 里排查了一遍全部引脚占用，挑的是当时完全没被任何外设占用、
也没被其他定时器实际启用过的引脚，但这是"没有冲突"的推断，不是拿到原理图确认过的走线——
如果实际接线用的是别的引脚，需要同步改 `Core/Src/tim.c`（`MX_TIM3_Init`/`MX_TIM4_Init`/
`HAL_TIM_MspPostInit`/`HAL_TIM_Encoder_MspInit`）和 `Bsp/dc_motor.c` 顶部的宏定义：

| 用途 | 定时器/通道 | STM32 引脚 | 备注 |
|---|---|---|---|
| H桥 IN1（PWM） | TIM3_CH1 | PA6 | 20kHz，`Bsp/dc_motor.c` 里 `DC_MOTOR_TIM3_CH_IN1` |
| H桥 IN2（PWM） | TIM3_CH2 | PA7 | 20kHz，`Bsp/dc_motor.c` 里 `DC_MOTOR_TIM3_CH_IN2` |
| 编码器 A相 | TIM4_CH1 | PD12 | 上拉输入，AF2 |
| 编码器 B相 | TIM4_CH2 | PD13 | 上拉输入，AF2 |

TIM3 在改动前虽然在 `.ioc` 里配置了4通道PWM（PA6/PA7/PB0/PB1），但代码里从未调用过
`HAL_TIM_PWM_Start`，是彻底空闲的外设，本次改动只启用了其中 CH1/CH2；TIM4 在改动前完全没有
在 `.ioc`/代码里出现过，是全新占用的定时器。`DC_MOTOR_ENCODER_LINES_PER_REV`（编码器线数，
`Bsp/dc_motor.c`）也是占位值，必须按实际编码器规格改，否则测出来的转速/角度会整体错一个比例。

## 备注

- `FOC_UART_Master.exe` 是较早版本的预编译上位机，协议可能落后于本文档；后续调试请使用
  `foc_uart_host.py`（随本次改动一起更新），或同目录下用 `pyinstaller --onefile --console
  --name foc_uart_host foc_uart_host.py` 重新打包出的 `foc_uart_host.exe`（免装 Python/pyserial，
  命令行用法与 `.py` 完全一致，改完协议后需要重新打包才能同步）。
- `foc_uart_gui.py` / `foc_uart_gui.exe`：图形界面版，双击 `.exe` 直接打开窗口即可用，不用敲命令行
  参数。内部复用 `foc_uart_host.py` 里的协议常量和解析函数，两边协议保持同步。改完协议后用
  `pyinstaller --onefile --windowed --name foc_uart_gui foc_uart_gui.py` 重新打包。
  界面观感参照 VOFA+（嵌入式圈常用的串口示波器）重做过：深色主题，左侧是示波器风格的黑底波形区
  （Tkinter Canvas 手画，无 matplotlib 依赖）+ 波形下方的数字读数条（模式/RPM/角度/Fault/LCD），
  右侧是通道列表（色块+勾选框+实时数值，对应 Ia/Ib/Ic/Id/Iq/RPM/MechAngle，各通道各自独立归一化
  到画布高度，所以电流和转速这种量纲差很大的量能同框看趋势）以及 PID/控制两个选项卡（读写PID、
  存到Flash/从Flash加载、切模式、DISARM、LCD开关），底部是深色终端风格的收发日志。需要先点
  “开始上传”让 `$TEL` 持续上报波形才有数据。旧版 `FOC_UART_Master.exe`（PyInstaller+Tkinter
  编译，源码从未提交进本仓库，无法找回）也带波形窗口，这个界面是照其思路、参考 VOFA+ 布局重新
  实现的替代品。
  「调试」选项卡另外补了 `0x84`(读取全部)/`0x85`(读调试计数，解析`$DBG`并显示 TIM9/TIM10 中断计数)/
  `$RAW`结构化表格；「控制」选项卡补了 LVGL 开关(`0x94`)和 OPEN_LOOP 模式下 Ud对齐电压/Uq运行电压/
  目标电角频率的 +/- 微调(`0x06`/`0x07`/`0x08`，仅支持步进调节，当前值来自设备`$ACK,OL_*`回执，
  未调整前显示`--`)——这几个命令固件早就支持、CLI版`foc_uart_host.py`也有一部分，但GUI最初漏做了，
  已核对固件命令表逐条补齐。
- LCD 的 SPI/DMA 排查记录：`Bsp/lcd_init.c`/`Bsp/lcd.c` 里实际编译进工程的 `_DMA` 系列函数
  自始至终用的是真正的 `hspi1`（SPI1，PB3/PB4/PB5，DMA2_Stream3/Channel3 正确对应 SPI1_TX
  硬件请求线），DMA 路径是通的；`SPI_BAUDRATEPRESCALER_2` 在 PCLK2=84MHz 下给出 SPI1 时钟
  42MHz，已是 HAL 能配置到的最高档位。曾经怀疑过的 `hspi1.Instance` 错位成 SPI2 导致 DMA
  挂错请求线的 bug，只存在于 `Bsp/lcd_dma.c`/`Bsp/lcd_init_dma.c` 这两个从未被加入 Keil 工程
  （`.uvprojx` 里没有注册）的孤儿文件里，已删除，不影响实际运行。
