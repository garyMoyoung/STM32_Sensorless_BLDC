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
```

## 命令表

| CMD | 名称 | ARG0 | ARG1 | 说明 |
|---:|---|---:|---:|---|
| `0x80` | 读取一次遥测 | 0 | 0 | 返回一帧 `$TEL` |
| `0x81` | 开始周期上传 | 周期 ms | 0 | `ARG0=0` 时默认 50 ms |
| `0x82` | 停止周期上传 | 0 | 0 | 停止 `$TEL` 周期输出 |
| `0x83` | 读取 PID | 0 | 0 | 返回 4 行 `$PID` |
| `0x84` | 读取全部 | 0 | 0 | 返回 `$TEL` + `$PID` + `$DBG` |
| `0x85` | 读取调试计数 | 0 | 0 | 返回 `$DBG` |
| `0x86` | 读取原始 ADC | 0 | 0 | 返回一帧 `$RAW`（原始计数/电压/标定偏置/故障位） |
| `0x87` | 重新标定电流零点 | 0 | 0 | 仅 IDLE 模式下有效，返回 `$ACK,CAL#` + `$RAW`，忙时返回 `$ERR,CAL,BUSY#` |
| `0x90` | 设置运行模式 | 模式值(0~4) | 0 | 返回 `$ACK,MODE,<n>#` |
| `0x91` | 断电/复位为 IDLE | 0 | 0 | 等价于 `0x90 0` |
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
$TEL,Ia,Ib,Ic,Id,Iq,RPM,MechAngle,ElecAngle,IdKp,IdKi,IdKd,IqKp,IqKi,IqKd,SpeedKp,SpeedKi,SpeedKd,PosKp,PosKi,PosKd,IqTarget,SpeedTarget,Mode,RawA,RawB,RawC,VoltA,VoltB,VoltC,Fault#\r\n
```

新增字段说明：

- `Mode`：当前 `FOC_Mode_t`（0~4，见上表）
- `RawA/RawB/RawC`：三相电流采样 ADC 原始计数（0~4095），对应 Ia/Ib/Ic
- `VoltA/VoltB/VoltC`：对应引脚换算后的电压（V）
- `Fault`：电流零点异常位掩码，bit0/1/2 分别对应 A/B/C 相；置位表示该相标定出的零点偏离 2048 计数超过约 0.48V，怀疑该相模拟前端（INA240 供电/参考电压/接线）有问题

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

- `ID`：d 轴电流环
- `IQ`：q 轴电流环
- `SPD`：速度环
- `POS`：位置环

### 调试计数帧

```text
$DBG,TIM9_ISR_CNT,TIM10_ISR_CNT#\r\n
```

### 应答/错误帧

```text
$ACK,MODE,<n>#\r\n        // 模式切换成功
$ACK,CAL#\r\n             // 重新标定成功
$ACK,OL_UD,<v>#\r\n       // 开环对齐电压已更新
$ACK,OL_UQ,<v>#\r\n       // 开环运行电压已更新
$ACK,OL_HZ,<v>#\r\n       // 开环目标电角频率已更新
$ACK,PID,<loop>,...#\r\n  // PID 参数已写入（见现有 $WPID 说明）
$ERR,CAL,BUSY#\r\n        // 非IDLE模式下请求重新标定，被拒绝
$ERR,PID,UNKNOWN_LOOP#\r\n
$ERR,UNKNOWN_CMD#\r\n
```

## 硬件相序映射（待与实物核实）

`Core/Src/main.c` 里 `Current_ReadRaw()` 当前假设的映射关系：

| 逻辑相 | ADC 注入序列 | STM32 引脚 |
|---|---|---|
| Ia | InjectedRank3 | PA4 (ADC1_IN4) |
| Ib | InjectedRank2 | PA3 (ADC1_IN3) |
| Ic | InjectedRank1 | PA2 (ADC1_IN2) |

这是代码里现有的假设，尚未与原理图/实物核实。如果拿到板子后发现某一相电流方向或相序不对，
只需调整 `Current_ReadRaw()` 里的下标映射，不影响 Clarke/Park/SVPWM 等其余算法。

## 备注

- `FOC_UART_Master.exe` 是较早版本的预编译上位机，协议可能落后于本文档；后续调试请使用
  `foc_uart_host.py`（随本次改动一起更新）。
