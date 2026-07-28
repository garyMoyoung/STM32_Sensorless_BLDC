#  UART 上位机通信协议 —— 可移植模块说明

> 本文件从 `STM32_Sensorless_BLDC` 工程中提炼出"MCU ⇄ PC 上位机"通信这一整套设计，供以后新工程直接复用。
> 涉及的源文件：`App/uart_task.c` / `App/inc/uart_task.h`、`Core/Src/stm32f4xx_it.c`（`USART1_IRQHandler`）、
> `Core/Src/main.c`（DMA接收启动、`printf`重定向）、`Core/Inc/main.h`（帧相关结构体/宏）、
> `upmachine/foc_uart_host.py`（PC端配套脚本）。
> 移植时把本文件当 checklist 用：先看"设计要点"理解为什么这样做，再照"移植步骤"抄结构、改内容。

---

## 1. 设计目标 / 适用场景

这套协议解决的是"单片机跑一个实时控制/采集循环，PC 通过一根 UART 在线读遥测、改参数、切模式"这类通用需求，不止适用于电机控制：

- **下行（PC→MCU）**：变长命令帧，同时支持二进制帧（省带宽、适合高频调参步进）和 ASCII 文本帧（人可读、适合一次性写一组浮点参数）。
- **上行（MCU→PC）**：`printf` 重定向到串口，直接打印 `$XXX,...#` 格式的文本行，PC 侧按行解析，天然可以在串口调试助手里肉眼看。
- **接收方式**：DMA 循环接收 + 空闲线（IDLE）中断分帧，而不是逐字节中断 + 状态机——这样 MCU 主循环/中断里没有任何一次性搬运一整帧的开销，帧长度也不需要预先知道。

---

## 2. 硬件与外设配置要点

| 项目 | 本工程配置 | 移植时注意 |
|---|---|---|
| 下行命令 UART | USART1，115200-8-N-1 | 波特率按需改，帧协议本身与波特率无关 |
| 下行接收方式 | `HAL_UART_Receive_DMA(&huart1, rx1_buffer, BUFFER_SIZE)` 常驻一块循环缓冲区 + `__HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE)` 开空闲线中断 | `BUFFER_SIZE` 要大于一次最长命令帧；本工程取 256 |
| 分帧时机 | `USART1_IRQHandler` 里检测到 `UART_FLAG_IDLE` → 清标志 → `HAL_UART_DMAStop` → 用 `__HAL_DMA_GET_COUNTER` 反推本次实际收了多少字节 → 拷出到 `rx1_frame_buffer` → 置 `recv1_end_flag=1` → 重新挂 DMA 继续收 | 这是本设计的核心技巧：**用DMA计数器剩余值反推收到的长度，不需要约定固定帧长**。别的工程只要MCU支持UART DMA+IDLE中断就能照搬 |
| 上行遥测/调试 UART | USART2（本工程专门拆出一路，避免和下行指令混在同一根线上） | 也可以复用同一路 UART，只是要注意别把自己发的遥测又当成命令收回来 |
| 主循环轮询点 | `UARTTask_Entry`（FreeRTOS任务，1ms周期）里调用 `UART_ProcessInTimer()` 消费 `recv1_end_flag` | 没有 RTOS 的工程可以直接在 `main()` 的 `while(1)` 里轮询 |

---

## 3. 帧格式设计

### 3.1 二进制帧（定长 7 字节，适合高频/步进类命令）

```
帧头(2B)   命令字(1B)  数据0(1B)  数据1(1B)  帧尾(2B)
0xFE 0xEF   CMD         ARG0       ARG1       0x23 0x24
```

- 帧头/帧尾用两字节而不是一字节，是为了降低"数据里刚好出现这个值"造成误同步的概率。
- **接收状态机**（`RxState` 枚举 + `FrameRxHandler` 结构体）：`WAIT_HEAD1 → WAIT_HEAD2 → WAIT_DEVICE → WAIT_data1 → WAIT_data2 → WAIT_TAIL1 → WAIT_TAIL2`，每收到一个不匹配帧头/帧尾的字节就退回 `WAIT_HEAD1` 重新同步，天然具备抗丢字节/抗噪声重同步能力。
- **坑（已踩过，移植时直接绕开）**：`WAIT_data1`/`WAIT_data2` 阶段收到的字节是**完整的数据字节（0~255）**，不是"剩余长度"字段，不能拿 `LEN_MAX` 之类的阈值去过滤，否则数值较大的合法参数（比如某个 ARG=200）会被误判卡死在该状态永远收不到帧尾。见 `uart_task.c` 里 `WAIT_data1` 分支的注释。

```c
// Core/Inc/main.h 里的核心结构体，原样复用
typedef enum {
    WAIT_HEAD1, WAIT_HEAD2, WAIT_DEVICE,
    WAIT_data1, WAIT_data2, WAIT_TAIL1, WAIT_TAIL2
} RxState;

typedef struct {
    uint8_t  rxBuff[DOWN_FRAME_LEN_MAX];
    uint8_t  device;     // 命令字
    uint8_t  data[2];    // ARG0 / ARG1
    RxState  state;
    bool     frameOK;
} FrameRxHandler;

typedef struct {
    uint8_t data[3];      // [0]=命令字 [1]=ARG0 [2]=ARG1
    uint8_t flag;         // 1=有效帧
} UART_Frame_t;
```

### 3.2 ASCII 文本帧（`$` 开头 `#` 结尾，适合一次写多个浮点参数）

```
$CMD,arg1,arg2,...#
```

- 分帧方式更简单：只要 `rx1_frame_buffer[0] == '$'`，整段收到的数据直接当字符串处理，用 `strtok(line, ",")` 按逗号切分，`strchr(line, '#')` 找到结尾截断。
- 二进制帧和 ASCII 帧**共用同一个 DMA+IDLE 接收缓冲区**，靠首字节 `'$'` 还是 `0xFE` 来分流到两套解析逻辑（见 `UART_ProcessInTimer`）。这样不用维护两套接收状态机。

```c
static void UART_ProcessAsciiCommand(char *line)
{
    char *end = strchr(line, '#');
    if (end != NULL) { *end = '\0'; }

    char *token = strtok(line, ",");
    // 后续 strtok(NULL, ",") 依次取各字段
    if (token && strcmp(token, "$WPID") == 0) { /* ... */ }
}
```

---

## 4. 命令分发与"参数试调"设计模式

`ProcessDataFrame(uint8_t *data, uint8_t flag)` 是二进制帧的统一入口，`data[0]`=命令字，`data[1]`=ARG0：

| 命令字区间 | 用途 | 复用价值 |
|---|---|---|
| 固定功能码（本工程 `0x80~0x94`） | 读遥测/开关推流/切模式/使能外设等"一次性动作" | 直接照搬命令分发框架，功能码按新工程需求重新定义 |
| `0x00~0x08`：分组步进调节码 | 每个组号对应一个"待调参数集"（比如某个PID的 kp/ki/kd/target），`ARG0` 的值域 `{0x01,0x11,0x02,0x12,0x03,0x13,0x04,0x14}` 分别表示 "kp+/kp-/ki+/ki-/kd+/kd-/target+/target-"，步长由第三个字节 `data[2]` 按 `10^-n` 换算（`calculate_step_size`） | **这是最值得移植的通用模式**：不需要每改一个参数就设计一条新命令，靠"选中哪个参数集 + 加/减 + 步长"三元组就能覆盖任意数量的可调浮点参数，非常适合上位机做旋钮/按键式在线调参 UI |

**"影子参数 + 统一提交"模式**（避免调参过程中中途状态被闭环任务读到半山腰的值）：

```c
// 每个可调参数集先复制一份"临时副本"，所有 +/- 步进只改临时副本
static PID_Param_t Id_temp, Iq_temp, Speed_temp, Position_temp;

// 一次分发流程结束后才把临时副本整体应用回真正生效的对象
if ((pid_to_apply != NULL) && (param_to_apply != NULL)) {
    PID_param_set(pid_to_apply, param_to_apply->kp, param_to_apply->ki, param_to_apply->kd);
    pid_to_apply->target = param_to_apply->target;
}
```

移植到新工程时，把 `Id_temp/Iq_temp/...` 换成新工程里任意"结构体里有若干浮点字段、需要在线微调"的对象即可，模式不变。

---

## 5. 命令字表（本工程实例，移植时按需增删）

**二进制命令**：

| 命令字 | 名称 | 说明 |
|---|---|---|
| `0x80` | READ_TELEMETRY | 单次读一帧遥测 `$TEL,...#` |
| `0x81` / `0x82` | STREAM_ON / STREAM_OFF | 开/关周期性遥测推流，`0x81` 的 ARG0 是推流周期(ms) |
| `0x83` | READ_PID | 读所有PID当前参数 `$PID,...#` |
| `0x84` | READ_ALL | 遥测+PID+调试计数一次性全读 |
| `0x85` | READ_DEBUG | 读调试计数 `$DBG,...#` |
| `0x86` | READ_RAW_ADC | 读原始ADC诊断量 `$RAW,...#` |
| `0x87` | RECALIBRATE | 触发某种"仅空闲态允许"的重新标定动作 |
| `0x90` | SET_MODE | 切换运行模式 |
| `0x91` | DISARM | 强制切回安全/空闲模式 |
| `0x93`/`0x94` | 外设使能开关 | 本工程是 LCD 遥测显示 / LVGL 动画演示 |
| `0x00~0x08` | 参数步进调节 | 见第4节 |

**ASCII命令**：

| 命令 | 格式 | 说明 |
|---|---|---|
| `$WPID` | `$WPID,<组名>,<target>,<kp>,<ki>,<kd>#` | 一次性写某一组PID的目标值+三个增益 |
| `$MODE` | `$MODE,<n>#` | 切换运行模式 |
| `$LCD` / `$LVGL` | `$LCD,<0\|1>#` | 开关某外设 |

**上行遥测格式**：`$TEL,...#`、`$RAW,...#`、`$PID,<组名>,kp,ki,kd,target,output,integral#`、`$DBG,...#`，以及一次性应答 `$ACK,...#` / `$ERR,...#`。字段个数固定，PC 侧按逗号数量严格校验（见第6节），任何一边改字段都要同步改另一边。

---

## 6. PC 端配套脚本设计（`upmachine/foc_uart_host.py`）

```python
HEAD = bytes([0xFE, 0xEF])
TAIL = bytes([0x23, 0x24])

def make_frame(cmd, arg0=0, arg1=0):
    return HEAD + bytes([cmd & 0xFF, arg0 & 0xFF, arg1 & 0xFF]) + TAIL

def parse_tel(line):
    if not line.startswith("$TEL,"):
        return None
    values = line.split(",")[1:]
    if len(values) != len(TEL_FIELDS):   # 字段数严格校验，对不上直接判无效，不强行解析
        return None
    return dict(zip(TEL_FIELDS, (float(v) for v in values)))
```

要点：
- `TEL_FIELDS`/`RAW_FIELDS` 是**字段名有序列表**，必须和MCU侧 `printf` 里 `%.Nf` 的顺序、个数逐一对应——这是最容易出现"协议两端各自改一下就对不上"的地方，建议移植后写一个小脚本或单测跑一遍字段计数校验。
- `argparse` 组织成"一个子命令对应一类交互"（`once/stream/mode/pidset/...`），而不是把所有命令字堆成一堆位置参数，新工程直接照抄这个 CLI 结构，改子命令列表即可。
- 单字节参数（如 `--period`）在拼帧前要做范围校验（`1~255`），否则 `arg0 & 0xFF` 会把超范围值悄悄截断成完全不同的数值，且没有任何报错——**这是本工程实际踩过的一个坑，移植时记得带上这层校验**。

---

## 7. 移植步骤 Checklist

1. **确认硬件**：目标UART外设是否支持 DMA 接收 + IDLE 中断（STM32 系列基本都支持）。
2. **拷贝文件**：`App/uart_task.c/h` 整体拷贝为新文件，`Core/Inc/main.h` 里的 `FrameRxHandler`/`UART_Frame_t`/`RxState`/帧相关宏一并拷贝。
3. **拷贝中断胶水代码**：`USART1_IRQHandler` 里 IDLE 检测 + DMA 计数反推长度那一段（第2节表格里的逻辑）。
4. **改命令字表**：删掉电机相关的命令（PID步进、FOC模式等），按新工程需求重新定义 `0x80` 往上的功能码；如果不需要"参数试调"模式，`0x00~0x08` 那一段和 `Id_temp` 等影子变量可以整体删除。
5. **改遥测格式**：`UART_PrintTelemetry` 等函数里的 `printf` 字段换成新工程实际要上报的变量，同步改 PC 脚本里的 `TEL_FIELDS` 顺序和个数。
6. **改 `printf` 重定向**：`main.c` 里 `fputc`/`__io_putchar` 调 `UART1_SendByte`，确认新工程的目标UART句柄。
7. **PC 脚本**：`foc_uart_host.py` 复制后改常量表（`CMD_*`、`*_FIELDS`）和子命令分支即可，`make_frame`/`send_cmd`/`read_lines` 这些底层函数不用动。
8. **联调**：先用 `once`/`all` 这类"一次性读"命令验证收发通不通，再验证 `stream`/`pidset` 这类高频/写入类命令。
