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

TEL_FIELDS = [
    "Ia", "Ib", "Ic", "Id", "Iq", "RPM", "MechAngle", "ElecAngle",
    "IdKp", "IdKi", "IdKd", "IqKp", "IqKi", "IqKd",
    "SpeedKp", "SpeedKi", "SpeedKd", "PosKp", "PosKi", "PosKd",
    "IqTarget", "SpeedTarget",
]


def make_frame(cmd: int, arg0: int = 0, arg1: int = 0) -> bytes:
    return HEAD + bytes([cmd & 0xFF, arg0 & 0xFF, arg1 & 0xFF]) + TAIL


def send_cmd(ser: serial.Serial, cmd: int, arg0: int = 0, arg1: int = 0) -> None:
    ser.write(make_frame(cmd, arg0, arg1))
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


def print_line(line: str) -> None:
    if line.startswith("$TEL,"):
        row = parse_tel(line)
        if row:
            print(
                f"Ia={row['Ia']:.3f} Ib={row['Ib']:.3f} Ic={row['Ic']:.3f} "
                f"Id={row['Id']:.3f} Iq={row['Iq']:.3f} "
                f"RPM={row['RPM']:.1f} Angle={row['MechAngle']:.3f}"
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
    parser.add_argument("mode", choices=["once", "pid", "all", "stream", "stop"])
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--period", type=int, default=50, help="stream 周期，单位 ms")
    parser.add_argument("--seconds", type=float, default=10, help="读取持续时间；stream 下 0 表示一直读")
    parser.add_argument("--csv", help="保存 TEL 数据到 CSV")
    args = parser.parse_args()

    with serial.Serial(args.port, args.baud, timeout=0.2) as ser:
        time.sleep(0.2)
        ser.reset_input_buffer()

        if args.mode == "once":
            send_cmd(ser, CMD_READ_TELEMETRY)
            read_lines(ser, 1.0, args.csv)
        elif args.mode == "pid":
            send_cmd(ser, CMD_READ_PID)
            read_lines(ser, 1.0, args.csv)
        elif args.mode == "all":
            send_cmd(ser, CMD_READ_ALL)
            read_lines(ser, 1.0, args.csv)
        elif args.mode == "stream":
            send_cmd(ser, CMD_STREAM_ON, args.period)
            read_lines(ser, args.seconds, args.csv)
        elif args.mode == "stop":
            send_cmd(ser, CMD_STREAM_OFF)
            read_lines(ser, 0.5, args.csv)


if __name__ == "__main__":
    main()
