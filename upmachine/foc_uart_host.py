#!/usr/bin/env python3
"""
FOC/BLDC 上位机读取脚本

依赖：pip install pyserial
示例：
  python foc_uart_host.py COM6 once
  python foc_uart_host.py COM6 pid
  python foc_uart_host.py COM6 all
  python foc_uart_host.py COM6 stream --period 50 --csv foc_log.csv
  python foc_uart_host.py COM6 stop
  python foc_uart_host.py COM6 raw
  python foc_uart_host.py COM6 recal
  python foc_uart_host.py COM6 mode --value speed
  python foc_uart_host.py COM6 disarm
  python foc_uart_host.py COM6 pidset --loop IQ --target 2.0 --kp 0.05 --ki 0.001 --kd 0
"""
import argparse
import csv
import time
from dataclasses import dataclass
from typing import Optional

import serial

HEAD = bytes([0xFE, 0xEF])
TAIL = bytes([0x23, 0x24])

CMD_READ_TELEMETRY = 0x80
CMD_STREAM_ON = 0x81
CMD_STREAM_OFF = 0x82
CMD_READ_PID = 0x83
CMD_READ_ALL = 0x84
CMD_READ_DEBUG = 0x85
CMD_READ_RAW_ADC = 0x86
CMD_RECALIBRATE = 0x87
CMD_SET_MODE = 0x90
CMD_DISARM = 0x91

MODE_NAMES = {
    "idle": 0,
    "open": 1,
    "openloop": 1,
    "current": 2,
    "speed": 3,
    "position": 4,
}
MODE_LABELS = {0: "IDLE", 1: "OPEN_LOOP", 2: "CURRENT", 3: "SPEED", 4: "POSITION"}

TEL_FIELDS = [
    "Ia", "Ib", "Ic", "Id", "Iq", "RPM", "MechAngle", "ElecAngle",
    "IdKp", "IdKi", "IdKd", "IqKp", "IqKi", "IqKd",
    "SpeedKp", "SpeedKi", "SpeedKd", "PosKp", "PosKi", "PosKd",
    "IqTarget", "SpeedTarget",
    "Mode", "RawA", "RawB", "RawC", "VoltA", "VoltB", "VoltC", "Fault",
]

RAW_FIELDS = [
    "AdcA", "VoltA", "OffA",
    "AdcB", "VoltB", "OffB",
    "AdcC", "VoltC", "OffC",
    "Fault",
]


def make_frame(cmd: int, arg0: int = 0, arg1: int = 0) -> bytes:
    return HEAD + bytes([cmd & 0xFF, arg0 & 0xFF, arg1 & 0xFF]) + TAIL


def send_cmd(ser: serial.Serial, cmd: int, arg0: int = 0, arg1: int = 0) -> None:
    ser.write(make_frame(cmd, arg0, arg1))
    ser.flush()


def send_ascii(ser: serial.Serial, line: str) -> None:
    ser.write(line.encode("ascii"))
    ser.flush()


def clean_line(raw: bytes) -> str:
    return raw.decode("ascii", errors="ignore").strip().strip("#")


def parse_tel(line: str) -> Optional[dict]:
    if not line.startswith("$TEL,"):
        return None
    parts = line.split(",")
    values = parts[1:]
    if len(values) != len(TEL_FIELDS):
        return None
    row = {"host_time": time.time()}
    row.update({name: float(value) for name, value in zip(TEL_FIELDS, values)})
    return row


def parse_raw(line: str) -> Optional[dict]:
    if not line.startswith("$RAW,"):
        return None
    parts = line.split(",")
    values = parts[1:]
    if len(values) != len(RAW_FIELDS):
        return None
    return dict(zip(RAW_FIELDS, (float(v) for v in values)))


def fault_phases(fault_mask: float) -> str:
    mask = int(fault_mask)
    phases = [name for bit, name in ((0, "A"), (1, "B"), (2, "C")) if mask & (1 << bit)]
    return ",".join(phases) if phases else "none"


def print_line(line: str) -> None:
    if line.startswith("$TEL,"):
        row = parse_tel(line)
        if row:
            mode = MODE_LABELS.get(int(row["Mode"]), str(row["Mode"]))
            print(
                f"[{mode}] Ia={row['Ia']:.3f} Ib={row['Ib']:.3f} Ic={row['Ic']:.3f} "
                f"Id={row['Id']:.3f} Iq={row['Iq']:.3f} "
                f"RPM={row['RPM']:.1f} Angle={row['MechAngle']:.3f} "
                f"Fault={fault_phases(row['Fault'])}"
            )
        else:
            print(line)
    elif line.startswith("$RAW,"):
        row = parse_raw(line)
        if row:
            print(
                f"A: adc={row['AdcA']:.0f} volt={row['VoltA']:.3f} off={row['OffA']:.0f}  "
                f"B: adc={row['AdcB']:.0f} volt={row['VoltB']:.3f} off={row['OffB']:.0f}  "
                f"C: adc={row['AdcC']:.0f} volt={row['VoltC']:.3f} off={row['OffC']:.0f}  "
                f"Fault={fault_phases(row['Fault'])}"
            )
        else:
            print(line)
    else:
        print(line)


def read_lines(ser: serial.Serial, seconds: float, csv_path: Optional[str] = None) -> None:
    writer = None
    csv_file = None
    try:
        if csv_path:
            csv_file = open(csv_path, "w", newline="", encoding="utf-8")
            writer = csv.DictWriter(csv_file, fieldnames=["host_time"] + TEL_FIELDS)
            writer.writeheader()

        deadline = time.time() + seconds if seconds > 0 else None
        while deadline is None or time.time() < deadline:
            raw = ser.readline()
            if not raw:
                continue
            line = clean_line(raw)
            print_line(line)
            row = parse_tel(line)
            if writer and row:
                writer.writerow(row)
                csv_file.flush()
    finally:
        if csv_file:
            csv_file.close()


def main() -> None:
    parser = argparse.ArgumentParser(description="FOC 串口上位机")
    parser.add_argument("port", help="串口号，例如 COM6")
    parser.add_argument(
        "mode_cmd",
        choices=["once", "pid", "all", "stream", "stop", "raw", "recal", "mode", "disarm", "pidset"],
    )
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--period", type=int, default=50, help="stream 周期，单位 ms")
    parser.add_argument("--seconds", type=float, default=10, help="读取持续时间；stream 下 0 表示一直读")
    parser.add_argument("--csv", help="保存 TEL 数据到 CSV")
    parser.add_argument(
        "--value", choices=sorted(MODE_NAMES.keys()), help="mode 子命令: idle/open/current/speed/position"
    )
    parser.add_argument("--loop", choices=["ID", "IQ", "SPD", "POS"], help="pidset 子命令: 目标环")
    parser.add_argument("--target", type=float, default=0.0, help="pidset 子命令: 目标值")
    parser.add_argument("--kp", type=float, default=0.0, help="pidset 子命令: Kp")
    parser.add_argument("--ki", type=float, default=0.0, help="pidset 子命令: Ki")
    parser.add_argument("--kd", type=float, default=0.0, help="pidset 子命令: Kd")
    args = parser.parse_args()

    with serial.Serial(args.port, args.baud, timeout=0.2) as ser:
        time.sleep(0.2)
        ser.reset_input_buffer()

        if args.mode_cmd == "once":
            send_cmd(ser, CMD_READ_TELEMETRY)
            read_lines(ser, 1.0, args.csv)
        elif args.mode_cmd == "pid":
            send_cmd(ser, CMD_READ_PID)
            read_lines(ser, 1.0, args.csv)
        elif args.mode_cmd == "all":
            send_cmd(ser, CMD_READ_ALL)
            read_lines(ser, 1.0, args.csv)
        elif args.mode_cmd == "stream":
            send_cmd(ser, CMD_STREAM_ON, args.period)
            read_lines(ser, args.seconds, args.csv)
        elif args.mode_cmd == "stop":
            send_cmd(ser, CMD_STREAM_OFF)
            read_lines(ser, 0.5, args.csv)
        elif args.mode_cmd == "raw":
            send_cmd(ser, CMD_READ_RAW_ADC)
            read_lines(ser, 1.0, None)
        elif args.mode_cmd == "recal":
            send_cmd(ser, CMD_RECALIBRATE)
            read_lines(ser, 1.0, None)
        elif args.mode_cmd == "mode":
            if not args.value:
                parser.error("mode 子命令需要 --value {idle,open,current,speed,position}")
            send_cmd(ser, CMD_SET_MODE, MODE_NAMES[args.value])
            read_lines(ser, 0.5, None)
        elif args.mode_cmd == "disarm":
            send_cmd(ser, CMD_DISARM)
            read_lines(ser, 0.5, None)
        elif args.mode_cmd == "pidset":
            if not args.loop:
                parser.error("pidset 子命令需要 --loop {ID,IQ,SPD,POS}")
            line = f"$WPID,{args.loop},{args.target},{args.kp},{args.ki},{args.kd}#\r\n"
            send_ascii(ser, line)
            read_lines(ser, 1.0, None)


if __name__ == "__main__":
    main()
