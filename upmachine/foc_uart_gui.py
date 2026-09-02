#!/usr/bin/env python3
"""
FOC/BLDC 上位机 GUI（双击直接打开，无需命令行参数）

界面参考 VOFA+（嵌入式圈常用的串口示波器）的观感重做：深色主题、大幅示波器风格波形区、
右侧通道列表（色块+勾选+实时数值），控制面板收进右侧选项卡。协议层完全复用
foc_uart_host.py 里已经写好的常量/帧格式/解析函数，两边协议保持同步。命令行版仍然
保留在 foc_uart_host.py，批量脚本/自动化调试请继续用命令行版。
"""
import queue
import threading
import tkinter as tk
from collections import deque
from tkinter import ttk, messagebox

import serial
import serial.tools.list_ports

import foc_uart_host as proto

LOOP_ORDER = ["ID", "IQ", "SPD", "POS", "DCSPD", "DCPOS"]
LOOP_LABELS = {
    "ID": "电流环 Id", "IQ": "电流环 Iq", "SPD": "速度环", "POS": "位置环",
    "DCSPD": "速度环(直流电机)", "DCPOS": "位置环(直流电机)",
}
MODE_ORDER = ["idle", "open", "current", "speed", "position"]
MODE_DISPLAY = {
    "idle": "IDLE (空闲)",
    "open": "OPEN_LOOP (开环强拖)",
    "current": "CURRENT (电流环)",
    "speed": "SPEED (速度环)",
    "position": "POSITION (位置环)",
}
DC_MODE_ORDER = ["idle", "speed", "position"]
DC_MODE_DISPLAY = {
    "idle": "IDLE (空闲)",
    "speed": "SPEED (速度环)",
    "position": "POSITION (位置环)",
}

WAVE_CHANNELS = [
    "Ia", "Ib", "Ic", "Id", "Iq", "RPM", "MechAngle", "DcAngle", "DcRPM", "DcDuty",
    # 滑模观测器(SMO)影子估算,和RPM/MechAngle/ElecAngle同单位方便叠加对比收敛情况
    "SmoRPM", "SmoTheta", "SmoEa", "SmoEb",
]
# 仿示波器/VOFA+ 通道配色：高饱和度，深色背景下辨识度高
WAVE_COLORS = {
    "Ia": "#ffd400", "Ib": "#00e5ff", "Ic": "#ff4d6d",
    "Id": "#5ec8ff", "Iq": "#ff9d3f", "RPM": "#00ff9c", "MechAngle": "#c792ea",
    "DcAngle": "#ff6ec7", "DcRPM": "#7cff6e", "DcDuty": "#8ab4ff",
    "SmoRPM": "#ffffff", "SmoTheta": "#ffb86c", "SmoEa": "#69f0ae", "SmoEb": "#40c4ff",
}
WAVE_DEFAULT_ON = {"RPM", "Id", "Iq"}
WAVE_MAXLEN = 300

# ---- 深色主题配色 ----
BG_APP = "#1b1e23"      # 全局背景
BG_PANEL = "#22262e"    # 面板/侧栏背景
BG_INPUT = "#2a2f38"    # 输入控件背景
BG_CANVAS = "#0e1116"   # 波形/日志区背景，接近示波器黑
GRID_COLOR = "#232830"
BORDER_COLOR = "#3a4049"
FG_TEXT = "#d7dbe0"
FG_DIM = "#8a90a0"
ACCENT = "#3ba7ff"
ACCENT_FG = "#0b0d10"
DANGER = "#ff5470"


def parse_pid_line(line: str):
    """解析 $PID,NAME,Kp,Ki,Kd,Target,Output,Integral"""
    if not line.startswith("$PID,"):
        return None
    parts = line.split(",")
    if len(parts) != 8:
        return None
    name = parts[1]
    try:
        kp, ki, kd, target, output, integral = (float(v) for v in parts[2:8])
    except ValueError:
        return None
    return {
        "name": name, "kp": kp, "ki": ki, "kd": kd,
        "target": target, "output": output, "integral": integral,
    }


def parse_dbg_line(line: str):
    """解析 $DBG,TIM9_ISR_CNT,TIM10_ISR_CNT"""
    if not line.startswith("$DBG,"):
        return None
    parts = line.split(",")
    if len(parts) != 3:
        return None
    try:
        tim9, tim10 = (int(float(v)) for v in parts[1:3])
    except ValueError:
        return None
    return {"tim9": tim9, "tim10": tim10}


def parse_ack_value(line: str, tag: str):
    """解析 $ACK,<tag>,<v># 里的单个数值,例如 $ACK,OL_UD,1.2500#"""
    prefix = f"$ACK,{tag},"
    if not line.startswith(prefix):
        return None
    try:
        return float(line[len(prefix):])
    except ValueError:
        return None


OL_STEP_CHOICES = ["1", "0.1", "0.01", "0.001"]
OL_STEP_DIGITS = {"1": 0, "0.1": 1, "0.01": 2, "0.001": 3}


class FocGui:
    def __init__(self, root: tk.Tk):
        self.root = root
        self.root.title("FOC 上位机")
        self.root.configure(bg=BG_APP)
        self.root.geometry("1180x760")
        self.root.minsize(920, 600)

        self.ser = None
        self.rx_thread = None
        self.rx_stop = threading.Event()
        self.rx_queue = queue.Queue()

        self.port_var = tk.StringVar()
        self.baud_var = tk.StringVar(value="115200")
        self.status_var = tk.StringVar(value="未连接")
        self.stream_period_var = tk.StringVar(value="50")

        self.readout_vars = {k: tk.StringVar(value="--") for k in (
            "Mode", "RPM", "MechAngle", "SmoRPM", "SmoTheta", "Fault", "LcdEnable")}
        self.wave_value_vars = {ch: tk.StringVar(value="--") for ch in WAVE_CHANNELS}
        self.pid_vars = {
            loop: {f: tk.StringVar(value="0") for f in ("kp", "ki", "kd", "target")}
            for loop in LOOP_ORDER
        }
        self.mode_select_var = tk.StringVar(value=MODE_DISPLAY["idle"])

        self.dbg_vars = {"tim9": tk.StringVar(value="--"), "tim10": tk.StringVar(value="--")}
        self.raw_vars = {ch: {"adc": tk.StringVar(value="--"), "volt": tk.StringVar(value="--"),
                               "off": tk.StringVar(value="--")} for ch in ("A", "B", "C")}
        self.raw_fault_var = tk.StringVar(value="--")
        self.lvgl_state_var = tk.StringVar(value="--")
        self.ol_step_var = tk.StringVar(value="0.1")
        self.ol_vars = {"ud": tk.StringVar(value="--"), "uq": tk.StringVar(value="--"),
                         "hz": tk.StringVar(value="--")}

        self.dc_mode_select_var = tk.StringVar(value=DC_MODE_DISPLAY["idle"])
        self.dc_readout_vars = {k: tk.StringVar(value="--") for k in ("Mode", "Angle", "RPM", "Duty")}

        self.wave_buffers = {ch: deque(maxlen=WAVE_MAXLEN) for ch in WAVE_CHANNELS}
        self.wave_enable = {ch: tk.BooleanVar(value=ch in WAVE_DEFAULT_ON) for ch in WAVE_CHANNELS}
        self.wave_paused = False
        self.wave_dirty = False

        self._setup_style()
        self._build_ui()
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)
        self.root.after(50, self._poll_queue)

    # ---------------- 深色主题 ----------------
    def _setup_style(self):
        style = ttk.Style(self.root)
        try:
            style.theme_use("clam")
        except tk.TclError:
            pass

        style.configure(".", background=BG_APP, foreground=FG_TEXT,
                         fieldbackground=BG_INPUT, bordercolor=BORDER_COLOR,
                         darkcolor=BG_PANEL, lightcolor=BG_PANEL, troughcolor=BG_INPUT)
        style.configure("TFrame", background=BG_APP)
        style.configure("Panel.TFrame", background=BG_PANEL)
        style.configure("TLabel", background=BG_APP, foreground=FG_TEXT)
        style.configure("Panel.TLabel", background=BG_PANEL, foreground=FG_TEXT)
        style.configure("Dim.TLabel", background=BG_APP, foreground=FG_DIM)
        style.configure("Dim.Panel.TLabel", background=BG_PANEL, foreground=FG_DIM)

        style.configure("TButton", background=BG_INPUT, foreground=FG_TEXT,
                         bordercolor=BORDER_COLOR, focusthickness=1, padding=(8, 4))
        style.map("TButton", background=[("active", "#3a4150")],
                   foreground=[("disabled", FG_DIM)])
        style.configure("Accent.TButton", background=ACCENT, foreground=ACCENT_FG, padding=(10, 5))
        style.map("Accent.TButton", background=[("active", "#5cb8ff")])
        style.configure("Danger.TButton", background="#3a1f27", foreground=DANGER, padding=(8, 4))
        style.map("Danger.TButton", background=[("active", "#4d2530")])

        style.configure("TCheckbutton", background=BG_APP, foreground=FG_TEXT)
        style.map("TCheckbutton", background=[("active", BG_APP)])
        style.configure("Panel.TCheckbutton", background=BG_PANEL, foreground=FG_TEXT)
        style.map("Panel.TCheckbutton", background=[("active", BG_PANEL)])

        style.configure("TEntry", fieldbackground=BG_INPUT, foreground=FG_TEXT,
                         insertcolor=FG_TEXT, bordercolor=BORDER_COLOR)
        style.configure("TCombobox", fieldbackground=BG_INPUT, foreground=FG_TEXT,
                         background=BG_INPUT, arrowcolor=FG_TEXT, bordercolor=BORDER_COLOR)
        style.map("TCombobox", fieldbackground=[("readonly", BG_INPUT)],
                   foreground=[("readonly", FG_TEXT)])
        self.root.option_add("*TCombobox*Listbox.background", BG_INPUT)
        self.root.option_add("*TCombobox*Listbox.foreground", FG_TEXT)
        self.root.option_add("*TCombobox*Listbox.selectBackground", ACCENT)
        self.root.option_add("*TCombobox*Listbox.selectForeground", ACCENT_FG)

        style.configure("TNotebook", background=BG_APP, bordercolor=BORDER_COLOR)
        style.configure("TNotebook.Tab", background=BG_PANEL, foreground=FG_DIM, padding=(12, 6))
        style.map("TNotebook.Tab", background=[("selected", BG_APP)],
                   foreground=[("selected", ACCENT)])

        style.configure("TLabelframe", background=BG_PANEL, bordercolor=BORDER_COLOR)
        style.configure("TLabelframe.Label", background=BG_PANEL, foreground=ACCENT)

        style.configure("Vertical.TScrollbar", background=BG_PANEL, troughcolor=BG_APP,
                         bordercolor=BG_APP, arrowcolor=FG_DIM)

    # ---------------- UI 构建 ----------------
    def _build_ui(self):
        self._build_toolbar()

        body = ttk.Frame(self.root)
        body.pack(fill="both", expand=True, padx=8, pady=(4, 4))
        body.columnconfigure(0, weight=1)
        body.columnconfigure(1, weight=0)
        body.rowconfigure(0, weight=1)

        self._build_wave_panel(body)
        self._build_side_panel(body)
        self._build_log_panel()

        self._refresh_ports()

    def _build_toolbar(self):
        bar = tk.Frame(self.root, bg=BG_PANEL)
        bar.pack(fill="x")
        inner = ttk.Frame(bar, style="Panel.TFrame")
        inner.pack(fill="x", padx=8, pady=6)

        ttk.Label(inner, text="串口", style="Panel.TLabel").pack(side="left", padx=(0, 4))
        self.port_combo = ttk.Combobox(inner, textvariable=self.port_var, width=12, state="readonly")
        self.port_combo.pack(side="left", padx=4)
        ttk.Button(inner, text="刷新", command=self._refresh_ports).pack(side="left", padx=4)

        ttk.Label(inner, text="波特率", style="Panel.TLabel").pack(side="left", padx=(12, 4))
        ttk.Entry(inner, textvariable=self.baud_var, width=8).pack(side="left", padx=4)

        self.connect_btn = ttk.Button(inner, text="● 连接", style="Accent.TButton",
                                       command=self._toggle_connect)
        self.connect_btn.pack(side="left", padx=(12, 8))
        ttk.Label(inner, textvariable=self.status_var, style="Dim.Panel.TLabel").pack(side="left")

        sep = ttk.Frame(inner, style="Panel.TFrame", width=1)
        sep.pack(side="left", fill="y", padx=16)

        ttk.Button(inner, text="读取一次", command=lambda: self._send_cmd(proto.CMD_READ_TELEMETRY)).pack(side="left", padx=4)
        ttk.Label(inner, text="周期(ms)", style="Panel.TLabel").pack(side="left", padx=(12, 2))
        ttk.Entry(inner, textvariable=self.stream_period_var, width=5).pack(side="left")
        ttk.Button(inner, text="▶ 开始上传", command=self._start_stream).pack(side="left", padx=4)
        ttk.Button(inner, text="■ 停止上传", command=lambda: self._send_cmd(proto.CMD_STREAM_OFF)).pack(side="left", padx=4)

    def _build_wave_panel(self, parent):
        left = ttk.Frame(parent)
        left.grid(row=0, column=0, sticky="nsew", padx=(0, 6))
        left.rowconfigure(1, weight=1)
        left.columnconfigure(0, weight=1)

        wave_tools = tk.Frame(left, bg=BG_APP)
        wave_tools.grid(row=0, column=0, sticky="ew", pady=(0, 4))
        self.wave_pause_btn = ttk.Button(wave_tools, text="❚❚ 暂停", command=self._toggle_wave_pause)
        self.wave_pause_btn.pack(side="left")
        ttk.Button(wave_tools, text="清空波形", command=self._clear_wave).pack(side="left", padx=6)
        ttk.Label(wave_tools, text="各通道独立归一化显示 · 需先“开始上传”",
                  style="Dim.TLabel").pack(side="left", padx=12)

        self.wave_canvas = tk.Canvas(left, bg=BG_CANVAS, highlightthickness=1,
                                      highlightbackground=BORDER_COLOR)
        self.wave_canvas.grid(row=1, column=0, sticky="nsew")
        self.wave_canvas.bind("<Configure>", lambda e: self._redraw_wave())

        readout = tk.Frame(left, bg=BG_CANVAS, highlightthickness=1, highlightbackground=BORDER_COLOR)
        readout.grid(row=2, column=0, sticky="ew", pady=(4, 0))
        items = [("Mode", "模式", ACCENT), ("RPM", "RPM", WAVE_COLORS["RPM"]),
                 ("MechAngle", "角度(rad)", WAVE_COLORS["MechAngle"]),
                 ("SmoRPM", "SMO RPM(估算)", WAVE_COLORS["SmoRPM"]),
                 ("SmoTheta", "SMO角度(估算)", WAVE_COLORS["SmoTheta"]),
                 ("Fault", "Fault", DANGER), ("LcdEnable", "LCD", FG_TEXT)]
        for i, (key, cn, color) in enumerate(items):
            cell = tk.Frame(readout, bg=BG_CANVAS)
            cell.pack(side="left", padx=14, pady=6)
            tk.Label(cell, text=cn, bg=BG_CANVAS, fg=FG_DIM, font=("Consolas", 9)).pack(anchor="w")
            tk.Label(cell, textvariable=self.readout_vars[key], bg=BG_CANVAS, fg=color,
                     font=("Consolas", 14, "bold")).pack(anchor="w")

    def _build_side_panel(self, parent):
        right = ttk.Frame(parent, style="Panel.TFrame", width=300)
        right.grid(row=0, column=1, sticky="ns")
        right.grid_propagate(False)

        notebook = ttk.Notebook(right)
        notebook.pack(fill="both", expand=True, padx=6, pady=6)

        notebook.add(self._build_channel_tab(notebook), text="通道")
        notebook.add(self._build_pid_tab(notebook), text="PID")
        notebook.add(self._build_control_tab(notebook), text="控制")
        notebook.add(self._build_debug_tab(notebook), text="调试")

    def _build_channel_tab(self, parent):
        tab = ttk.Frame(parent, style="Panel.TFrame")

        quick = ttk.Frame(tab, style="Panel.TFrame")
        quick.pack(fill="x", padx=6, pady=(8, 4))
        ttk.Button(quick, text="全选", command=lambda: self._set_all_channels(True)).pack(side="left")
        ttk.Button(quick, text="全不选", command=lambda: self._set_all_channels(False)).pack(side="left", padx=6)

        for ch in WAVE_CHANNELS:
            row = tk.Frame(tab, bg=BG_PANEL)
            row.pack(fill="x", padx=6, pady=3)
            swatch = tk.Frame(row, bg=WAVE_COLORS[ch], width=12, height=12)
            swatch.pack(side="left", padx=(2, 6))
            ttk.Checkbutton(row, text=ch, style="Panel.TCheckbutton", variable=self.wave_enable[ch],
                             command=self._redraw_wave).pack(side="left")
            tk.Label(row, textvariable=self.wave_value_vars[ch], bg=BG_PANEL, fg=WAVE_COLORS[ch],
                     font=("Consolas", 10), width=10, anchor="e").pack(side="right")
        return tab

    def _build_pid_tab(self, parent):
        tab = ttk.Frame(parent, style="Panel.TFrame")

        for r, loop in enumerate(LOOP_ORDER):
            box = ttk.LabelFrame(tab, text=LOOP_LABELS[loop])
            box.pack(fill="x", padx=6, pady=5)
            fields = [("Kp", "kp"), ("Ki", "ki"), ("Kd", "kd"), ("Target", "target")]
            for c, (label, key) in enumerate(fields):
                cell = ttk.Frame(box, style="Panel.TFrame")
                cell.grid(row=0, column=c, padx=3, pady=4)
                ttk.Label(cell, text=label, style="Dim.Panel.TLabel").pack()
                ttk.Entry(cell, textvariable=self.pid_vars[loop][key], width=8).pack()
            ttk.Button(box, text="写入", command=lambda l=loop: self._write_pid(l)).grid(
                row=0, column=len(fields), padx=6)

        ttk.Button(tab, text="读取全部PID", command=lambda: self._send_cmd(proto.CMD_READ_PID)).pack(
            fill="x", padx=6, pady=(10, 3))
        flash_row = ttk.Frame(tab, style="Panel.TFrame")
        flash_row.pack(fill="x", padx=6)
        ttk.Button(flash_row, text="保存到Flash", style="Accent.TButton",
                   command=lambda: self._send_cmd(proto.CMD_SAVE_PID)).pack(side="left", expand=True, fill="x")
        ttk.Button(flash_row, text="从Flash加载",
                   command=lambda: self._send_cmd(proto.CMD_LOAD_PID)).pack(side="left", expand=True, fill="x", padx=(6, 0))
        ttk.Label(tab, text="保存/加载仅在 IDLE 模式下有效", style="Dim.Panel.TLabel").pack(
            anchor="w", padx=6, pady=(4, 0))
        return tab

    def _build_control_tab(self, parent):
        tab = ttk.Frame(parent, style="Panel.TFrame")

        mode_box = ttk.LabelFrame(tab, text="运行模式")
        mode_box.pack(fill="x", padx=6, pady=(8, 5))
        self.mode_combo = ttk.Combobox(mode_box, textvariable=self.mode_select_var, state="readonly",
                                        values=[MODE_DISPLAY[m] for m in MODE_ORDER])
        self.mode_combo.pack(fill="x", padx=6, pady=(6, 4))
        ttk.Button(mode_box, text="设置模式", command=self._set_mode).pack(fill="x", padx=6, pady=(0, 4))
        ttk.Button(mode_box, text="DISARM · 回到 IDLE", style="Danger.TButton",
                   command=lambda: self._send_cmd(proto.CMD_DISARM)).pack(fill="x", padx=6, pady=(0, 6))

        dc_box = ttk.LabelFrame(tab, text="直流有刷电机（第二台电机）")
        dc_box.pack(fill="x", padx=6, pady=5)
        self.dc_mode_combo = ttk.Combobox(dc_box, textvariable=self.dc_mode_select_var, state="readonly",
                                           values=[DC_MODE_DISPLAY[m] for m in DC_MODE_ORDER])
        self.dc_mode_combo.pack(fill="x", padx=6, pady=(6, 4))
        ttk.Button(dc_box, text="设置模式", command=self._set_dc_mode).pack(fill="x", padx=6, pady=(0, 4))
        ttk.Button(dc_box, text="DISARM · 回到 IDLE", style="Danger.TButton",
                   command=lambda: self._send_cmd(proto.CMD_SET_DC_MODE, 0)).pack(fill="x", padx=6, pady=(0, 6))
        dc_readout = ttk.Frame(dc_box, style="Panel.TFrame")
        dc_readout.pack(fill="x", padx=6, pady=(0, 6))
        for label, key in (("模式", "Mode"), ("角度(rad)", "Angle"), ("RPM", "RPM"), ("占空比", "Duty")):
            cell = ttk.Frame(dc_readout, style="Panel.TFrame")
            cell.pack(side="left", expand=True, fill="x")
            ttk.Label(cell, text=label, style="Dim.Panel.TLabel").pack()
            ttk.Label(cell, textvariable=self.dc_readout_vars[key], style="Panel.TLabel").pack()

        lcd_box = ttk.LabelFrame(tab, text="LCD 屏幕")
        lcd_box.pack(fill="x", padx=6, pady=5)
        lcd_row = ttk.Frame(lcd_box, style="Panel.TFrame")
        lcd_row.pack(fill="x", padx=6, pady=6)
        ttk.Button(lcd_row, text="开", command=lambda: self._send_cmd(proto.CMD_SET_LCD_ENABLE, 1)).pack(
            side="left", expand=True, fill="x")
        ttk.Button(lcd_row, text="关", command=lambda: self._send_cmd(proto.CMD_SET_LCD_ENABLE, 0)).pack(
            side="left", expand=True, fill="x", padx=(6, 0))

        lvgl_box = ttk.LabelFrame(tab, text="LVGL 演示")
        lvgl_box.pack(fill="x", padx=6, pady=5)
        lvgl_row = ttk.Frame(lvgl_box, style="Panel.TFrame")
        lvgl_row.pack(fill="x", padx=6, pady=6)
        ttk.Button(lvgl_row, text="开", command=lambda: self._send_cmd(proto.CMD_SET_LVGL_ENABLE, 1)).pack(
            side="left", expand=True, fill="x")
        ttk.Button(lvgl_row, text="关", command=lambda: self._send_cmd(proto.CMD_SET_LVGL_ENABLE, 0)).pack(
            side="left", expand=True, fill="x", padx=(6, 0))
        ttk.Label(lvgl_box, textvariable=self.lvgl_state_var, style="Dim.Panel.TLabel").pack(
            anchor="w", padx=6, pady=(0, 6))

        ol_box = ttk.LabelFrame(tab, text="开环强拖 OPEN_LOOP 参数微调")
        ol_box.pack(fill="x", padx=6, pady=5)
        step_row = ttk.Frame(ol_box, style="Panel.TFrame")
        step_row.pack(fill="x", padx=6, pady=(6, 4))
        ttk.Label(step_row, text="步进", style="Dim.Panel.TLabel").pack(side="left")
        ttk.Combobox(step_row, textvariable=self.ol_step_var, values=OL_STEP_CHOICES, width=8,
                     state="readonly").pack(side="left", padx=6)

        ol_rows = [("对齐电压 Ud (V)", "ud", proto.CMD_OL_ADJUST_UD),
                   ("运行电压 Uq (V)", "uq", proto.CMD_OL_ADJUST_UQ),
                   ("目标电角频率 (Hz)", "hz", proto.CMD_OL_ADJUST_HZ)]
        for label, key, cmd in ol_rows:
            row = ttk.Frame(ol_box, style="Panel.TFrame")
            row.pack(fill="x", padx=6, pady=3)
            ttk.Label(row, text=label, style="Panel.TLabel", width=15).pack(side="left")
            ttk.Label(row, textvariable=self.ol_vars[key], style="Dim.Panel.TLabel", width=8).pack(side="left")
            ttk.Button(row, text="−", width=3, command=lambda c=cmd: self._adjust_open_loop(c, False)).pack(
                side="left", padx=(6, 2))
            ttk.Button(row, text="+", width=3, command=lambda c=cmd: self._adjust_open_loop(c, True)).pack(
                side="left")
        ttk.Label(ol_box, text="仅 OPEN_LOOP 模式下生效；数值只能靠 +/- 微调，\n"
                               "当前值来自设备的确认回执，未调整前显示为 --",
                  style="Dim.Panel.TLabel", justify="left").pack(anchor="w", padx=6, pady=(2, 6))
        return tab

    def _build_debug_tab(self, parent):
        tab = ttk.Frame(parent, style="Panel.TFrame")

        read_box = ttk.LabelFrame(tab, text="批量读取")
        read_box.pack(fill="x", padx=6, pady=(8, 5))
        ttk.Button(read_box, text="读取全部 (TEL+PID+DBG)",
                   command=lambda: self._send_cmd(proto.CMD_READ_ALL)).pack(fill="x", padx=6, pady=(6, 3))
        row1 = ttk.Frame(read_box, style="Panel.TFrame")
        row1.pack(fill="x", padx=6, pady=(0, 6))
        ttk.Button(row1, text="读取原始ADC", command=lambda: self._send_cmd(proto.CMD_READ_RAW_ADC)).pack(
            side="left", expand=True, fill="x")
        ttk.Button(row1, text="读取调试计数", command=lambda: self._send_cmd(proto.CMD_READ_DEBUG)).pack(
            side="left", expand=True, fill="x", padx=(6, 0))
        ttk.Button(read_box, text="重新标定电流零点",
                   command=lambda: self._send_cmd(proto.CMD_RECALIBRATE)).pack(fill="x", padx=6, pady=(0, 6))
        ttk.Button(read_box, text="电角度对齐（仅 IDLE）",
                   command=lambda: self._send_cmd(proto.CMD_ELECTRICAL_ALIGN)).pack(fill="x", padx=6, pady=(0, 6))

        dbg_box = ttk.LabelFrame(tab, text="调试计数 ($DBG)")
        dbg_box.pack(fill="x", padx=6, pady=5)
        for label, key in (("TIM9_ISR_CNT", "tim9"), ("TIM10_ISR_CNT", "tim10")):
            row = ttk.Frame(dbg_box, style="Panel.TFrame")
            row.pack(fill="x", padx=6, pady=2)
            ttk.Label(row, text=label, style="Panel.TLabel", width=14).pack(side="left")
            ttk.Label(row, textvariable=self.dbg_vars[key], style="Dim.Panel.TLabel").pack(side="left")
        ttk.Frame(dbg_box, style="Panel.TFrame", height=4).pack()

        raw_box = ttk.LabelFrame(tab, text="原始ADC ($RAW)")
        raw_box.pack(fill="x", padx=6, pady=5)
        header = ttk.Frame(raw_box, style="Panel.TFrame")
        header.pack(fill="x", padx=6, pady=(6, 2))
        for text, w in (("相", 4), ("ADC", 8), ("电压V", 8), ("零点", 8)):
            ttk.Label(header, text=text, style="Dim.Panel.TLabel", width=w).pack(side="left")
        for ch in ("A", "B", "C"):
            row = ttk.Frame(raw_box, style="Panel.TFrame")
            row.pack(fill="x", padx=6, pady=1)
            ttk.Label(row, text=ch, style="Panel.TLabel", width=4).pack(side="left")
            ttk.Label(row, textvariable=self.raw_vars[ch]["adc"], style="Panel.TLabel", width=8).pack(side="left")
            ttk.Label(row, textvariable=self.raw_vars[ch]["volt"], style="Panel.TLabel", width=8).pack(side="left")
            ttk.Label(row, textvariable=self.raw_vars[ch]["off"], style="Panel.TLabel", width=8).pack(side="left")
        fault_row = ttk.Frame(raw_box, style="Panel.TFrame")
        fault_row.pack(fill="x", padx=6, pady=(4, 6))
        ttk.Label(fault_row, text="Fault", style="Dim.Panel.TLabel").pack(side="left")
        ttk.Label(fault_row, textvariable=self.raw_fault_var, style="Panel.TLabel").pack(side="left", padx=6)
        return tab

    def _build_log_panel(self):
        log_frame = tk.Frame(self.root, bg=BG_PANEL)
        log_frame.pack(fill="both", padx=8, pady=(0, 8))

        header = ttk.Frame(log_frame, style="Panel.TFrame")
        header.pack(fill="x", padx=6, pady=(6, 2))
        ttk.Label(header, text="日志", style="Panel.TLabel").pack(side="left")
        ttk.Button(header, text="清空日志", command=self._clear_log).pack(side="right")

        text_row = ttk.Frame(log_frame, style="Panel.TFrame")
        text_row.pack(fill="both", expand=True, padx=6, pady=(0, 6))
        self.log_text = tk.Text(text_row, height=8, bg=BG_CANVAS, fg="#9be564",
                                 insertbackground=FG_TEXT, relief="flat",
                                 font=("Consolas", 9), state="disabled")
        self.log_text.pack(side="left", fill="both", expand=True)
        scrollbar = ttk.Scrollbar(text_row, command=self.log_text.yview)
        scrollbar.pack(side="right", fill="y")
        self.log_text["yscrollcommand"] = scrollbar.set

    # ---------------- 串口连接 ----------------
    def _refresh_ports(self):
        ports = [p.device for p in serial.tools.list_ports.comports()]
        self.port_combo["values"] = ports
        if ports and self.port_var.get() not in ports:
            self.port_var.set(ports[0])

    def _toggle_connect(self):
        if self.ser is not None:
            self._disconnect()
        else:
            self._connect()

    def _connect(self):
        port = self.port_var.get()
        if not port:
            messagebox.showwarning("提示", "请先选择串口")
            return
        try:
            baud = int(self.baud_var.get())
            self.ser = serial.Serial(port, baud, timeout=0.2)
        except (serial.SerialException, ValueError) as exc:
            messagebox.showerror("连接失败", str(exc))
            self.ser = None
            return

        self.rx_stop.clear()
        self.rx_thread = threading.Thread(target=self._rx_loop, daemon=True)
        self.rx_thread.start()

        self.status_var.set(f"已连接 {port}@{baud}")
        self.connect_btn.config(text="● 断开", style="Danger.TButton")
        self._log(f"[连接] {port}@{baud}")

    def _disconnect(self):
        self.rx_stop.set()
        if self.rx_thread is not None:
            self.rx_thread.join(timeout=1.0)
        if self.ser is not None:
            try:
                self.ser.close()
            except serial.SerialException:
                pass
        self.ser = None
        self.status_var.set("未连接")
        self.connect_btn.config(text="● 连接", style="Accent.TButton")
        self._log("[断开]")

    def _on_close(self):
        self._disconnect()
        self.root.destroy()

    def _rx_loop(self):
        while not self.rx_stop.is_set():
            try:
                raw = self.ser.readline()
            except serial.SerialException:
                break
            if raw:
                self.rx_queue.put(raw)

    # ---------------- 发送命令 ----------------
    def _send_cmd(self, cmd, arg0=0, arg1=0):
        if self.ser is None:
            messagebox.showwarning("提示", "请先连接串口")
            return
        try:
            proto.send_cmd(self.ser, cmd, arg0, arg1)
        except serial.SerialException as exc:
            messagebox.showerror("发送失败", str(exc))
            return
        self._log(f">> CMD 0x{cmd:02X} arg0={arg0} arg1={arg1}")

    def _send_ascii(self, line):
        if self.ser is None:
            messagebox.showwarning("提示", "请先连接串口")
            return
        try:
            proto.send_ascii(self.ser, line)
        except serial.SerialException as exc:
            messagebox.showerror("发送失败", str(exc))
            return
        self._log(f">> {line.strip()}")

    def _start_stream(self):
        try:
            period = int(self.stream_period_var.get())
        except ValueError:
            messagebox.showwarning("提示", "周期必须是整数(ms)")
            return
        if not 1 <= period <= 255:
            messagebox.showwarning("提示", "周期必须在 1~255 之间")
            return
        self._send_cmd(proto.CMD_STREAM_ON, period)

    def _write_pid(self, loop):
        vars_ = self.pid_vars[loop]
        try:
            kp = float(vars_["kp"].get())
            ki = float(vars_["ki"].get())
            kd = float(vars_["kd"].get())
            target = float(vars_["target"].get())
        except ValueError:
            messagebox.showwarning("提示", "Kp/Ki/Kd/Target 必须是数字")
            return
        self._send_ascii(f"$WPID,{loop},{target},{kp},{ki},{kd}#\r\n")

    def _set_mode(self):
        display = self.mode_select_var.get()
        mode_key = next((m for m in MODE_ORDER if MODE_DISPLAY[m] == display), None)
        if mode_key is None:
            return
        self._send_cmd(proto.CMD_SET_MODE, proto.MODE_NAMES[mode_key])

    def _set_dc_mode(self):
        display = self.dc_mode_select_var.get()
        mode_key = next((m for m in DC_MODE_ORDER if DC_MODE_DISPLAY[m] == display), None)
        if mode_key is None:
            return
        self._send_cmd(proto.CMD_SET_DC_MODE, proto.DC_MODE_NAMES[mode_key])

    def _set_all_channels(self, value):
        for var in self.wave_enable.values():
            var.set(value)
        self._redraw_wave()

    def _adjust_open_loop(self, cmd, plus):
        digit = OL_STEP_DIGITS.get(self.ol_step_var.get(), 1)
        field_code = proto.FIELD_STEP_PLUS if plus else proto.FIELD_STEP_MINUS
        self._send_cmd(cmd, field_code, digit)

    # ---------------- 接收处理 ----------------
    def _poll_queue(self):
        try:
            while True:
                raw = self.rx_queue.get_nowait()
                self._handle_line(proto.clean_line(raw))
        except queue.Empty:
            pass
        if self.wave_dirty:
            self.wave_dirty = False
            self._redraw_wave()
        self.root.after(50, self._poll_queue)

    def _handle_line(self, line):
        if not line:
            return
        self._log(f"<< {line}")

        row = proto.parse_tel(line)
        if row is not None:
            self.readout_vars["Mode"].set(proto.MODE_LABELS.get(int(row["Mode"]), str(row["Mode"])))
            self.readout_vars["RPM"].set(f"{row['RPM']:.1f}")
            self.readout_vars["MechAngle"].set(f"{row['MechAngle']:.3f}")
            self.readout_vars["SmoRPM"].set(f"{row['SmoRPM']:.1f}")
            self.readout_vars["SmoTheta"].set(f"{row['SmoTheta']:.3f}")
            self.readout_vars["Fault"].set(proto.fault_phases(row["Fault"]))
            self.readout_vars["LcdEnable"].set("开" if row["LcdEnable"] else "关")
            self.dc_readout_vars["Mode"].set(proto.DC_MODE_LABELS.get(int(row["DcMode"]), str(row["DcMode"])))
            self.dc_readout_vars["Angle"].set(f"{row['DcAngle']:.3f}")
            self.dc_readout_vars["RPM"].set(f"{row['DcRPM']:.1f}")
            self.dc_readout_vars["Duty"].set(f"{row['DcDuty']:.2f}")
            if not self.wave_paused:
                for ch in WAVE_CHANNELS:
                    self.wave_buffers[ch].append(row[ch])
                    self.wave_value_vars[ch].set(f"{row[ch]:.3f}")
                self.wave_dirty = True
            return

        pid_row = parse_pid_line(line)
        if pid_row is not None and pid_row["name"] in self.pid_vars:
            vars_ = self.pid_vars[pid_row["name"]]
            vars_["kp"].set(f"{pid_row['kp']:.5f}")
            vars_["ki"].set(f"{pid_row['ki']:.5f}")
            vars_["kd"].set(f"{pid_row['kd']:.5f}")
            vars_["target"].set(f"{pid_row['target']:.5f}")
            return

        dbg_row = parse_dbg_line(line)
        if dbg_row is not None:
            self.dbg_vars["tim9"].set(str(dbg_row["tim9"]))
            self.dbg_vars["tim10"].set(str(dbg_row["tim10"]))
            return

        raw_row = proto.parse_raw(line)
        if raw_row is not None:
            for ch, key in (("A", "A"), ("B", "B"), ("C", "C")):
                self.raw_vars[ch]["adc"].set(f"{raw_row[f'Adc{key}']:.0f}")
                self.raw_vars[ch]["volt"].set(f"{raw_row[f'Volt{key}']:.3f}")
                self.raw_vars[ch]["off"].set(f"{raw_row[f'Off{key}']:.0f}")
            self.raw_fault_var.set(proto.fault_phases(raw_row["Fault"]))
            return

        for tag, var in (("OL_UD", self.ol_vars["ud"]), ("OL_UQ", self.ol_vars["uq"]),
                          ("OL_HZ", self.ol_vars["hz"])):
            v = parse_ack_value(line, tag)
            if v is not None:
                var.set(f"{v:.4f}")
                return

        lvgl_v = parse_ack_value(line, "LVGL")
        if lvgl_v is not None:
            self.lvgl_state_var.set("开" if lvgl_v else "关")
            return

    # ---------------- 波形 ----------------
    def _toggle_wave_pause(self):
        self.wave_paused = not self.wave_paused
        self.wave_pause_btn.config(text="▶ 继续" if self.wave_paused else "❚❚ 暂停")

    def _clear_wave(self):
        for buf in self.wave_buffers.values():
            buf.clear()
        self._redraw_wave()

    def _redraw_wave(self):
        c = self.wave_canvas
        c.delete("all")
        width = c.winfo_width()
        height = c.winfo_height()
        if width < 20 or height < 20:
            return
        margin = 14

        for i in range(9):
            x = margin + i * (width - 2 * margin) / 8
            c.create_line(x, margin, x, height - margin, fill=GRID_COLOR)
        for i in range(7):
            y = margin + i * (height - 2 * margin) / 6
            c.create_line(margin, y, width - margin, y, fill=GRID_COLOR)
        c.create_rectangle(margin, margin, width - margin, height - margin, outline=BORDER_COLOR)

        for ch in WAVE_CHANNELS:
            if not self.wave_enable[ch].get():
                continue
            buf = self.wave_buffers[ch]
            n = len(buf)
            if n < 2:
                continue
            vmin = min(buf)
            vmax = max(buf)
            if vmax - vmin < 1e-9:
                vmax = vmin + 1.0
            points = []
            for i, v in enumerate(buf):
                x = margin + i * (width - 2 * margin) / (n - 1)
                y = height - margin - (v - vmin) / (vmax - vmin) * (height - 2 * margin)
                points.extend([x, y])
            c.create_line(*points, fill=WAVE_COLORS[ch], width=1.6, smooth=True)

    # ---------------- 日志 ----------------
    def _log(self, text):
        self.log_text.config(state="normal")
        self.log_text.insert("end", text + "\n")
        self.log_text.see("end")
        self.log_text.config(state="disabled")

    def _clear_log(self):
        self.log_text.config(state="normal")
        self.log_text.delete("1.0", "end")
        self.log_text.config(state="disabled")


def main():
    root = tk.Tk()
    FocGui(root)
    root.mainloop()


if __name__ == "__main__":
    main()
