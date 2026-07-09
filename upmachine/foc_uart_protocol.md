# FOC 串口读取协议

## 串口参数

- 波特率：115200
- 数据位：8
- 停止位：1
- 校验：None
- 下行命令帧：7 字节固定长度

## 上位机 -> 下位机

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

## 命令表

| CMD | 名称 | ARG0 | ARG1 | 说明 |
|---:|---|---:|---:|---|
| `0x80` | 读取一次遥测 | 0 | 0 | 返回一帧 `$TEL` |
| `0x81` | 开始周期上传 | 周期 ms | 0 | `ARG0=0` 时默认 50 ms |
| `0x82` | 停止周期上传 | 0 | 0 | 停止 `$TEL` 周期输出 |
| `0x83` | 读取 PID | 0 | 0 | 返回 4 行 `$PID` |
| `0x84` | 读取全部 | 0 | 0 | 返回 `$TEL` + `$PID` |

## 下位机 -> 上位机

### 遥测帧

```text
$TEL,Ia,Ib,Ic,Id,Iq,RPM,MechAngle,ElecAngle,IdKp,IdKi,IdKd,IqKp,IqKi,IqKd,SpeedKp,SpeedKi,SpeedKd,PosKp,PosKi,PosKd,IqTarget,SpeedTarget#\r\n
```

### PID 帧

```text
$PID,NAME,Kp,Ki,Kd,Target,Output,Integral#\r\n
```

`NAME` 取值：

- `ID`：d 轴电流环
- `IQ`：q 轴电流环
- `SPD`：速度环
- `POS`：位置环

注意：当前工程里位置环变量已添加并初始化，但 FOC 控制主循环暂未使用位置环参与计算；协议先预留并返回它的参数。
