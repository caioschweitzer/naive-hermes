import subprocess
import atexit
import customtkinter as ctk
import tkinter as tk
import socket
import threading
import queue
import struct
import math
import time
from collections import deque
import matplotlib
matplotlib.use("TkAgg")
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import proto.robot_comm_pb2 as proto

def set_x11_keys_autorepeat(enable: bool):
    """Ativa ou desativa o autorepeat das teclas WASDQE no Linux/X11 para evitar lag de buffer."""
    try:
        if enable:
            subprocess.run(["xset", "r", "24", "r", "25", "r", "26", "r", "38", "r", "39", "r", "40"],
                           stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, check=False)
            subprocess.run(["xset", "r", "on"],
                           stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, check=False)
        else:
            subprocess.run(["xset", "-r", "24", "-r", "25", "-r", "26", "-r", "38", "-r", "39", "-r", "40"],
                           stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, check=False)
    except Exception:
        pass

atexit.register(lambda: set_x11_keys_autorepeat(True))


# Mapeamento de erros (máscaras de bits) para metadados visuais e severidade
ROBOT_ERRORS_CONFIG = {
    proto.ERR_CRITICAL_BATTERY: {
        "name": "ERR_CRITICAL_BATTERY",
        "label": "Bateria Crítica",
        "badge": "🪫 BAT CRÍTICA",
        "severity": "critical",
        "fg_color": "#78281F",
        "text_color": "#FADBD8",
        "border_color": "#E74C3C"
    },
    proto.ERR_KICKER_OVERVOLTAGE: {
        "name": "ERR_KICKER_OVERVOLTAGE",
        "label": "Sobretensão Chute",
        "badge": "⚡ SOBRETENSÃO",
        "severity": "critical",
        "fg_color": "#78281F",
        "text_color": "#FADBD8",
        "border_color": "#E74C3C"
    },
    proto.ERR_ROBOT_FLIPPED: {
        "name": "ERR_ROBOT_FLIPPED",
        "label": "Robô Tombado",
        "badge": "🔄 TOMBADO",
        "severity": "critical",
        "fg_color": "#78281F",
        "text_color": "#FADBD8",
        "border_color": "#E74C3C"
    },
    proto.ERR_STM32_TIMEOUT: {
        "name": "ERR_STM32_TIMEOUT",
        "label": "STM32 Timeout",
        "badge": "🔌 STM32 TIMEOUT",
        "severity": "critical",
        "fg_color": "#78281F",
        "text_color": "#FADBD8",
        "border_color": "#E74C3C"
    },
    proto.ERR_LOW_BATTERY: {
        "name": "ERR_LOW_BATTERY",
        "label": "Bateria Baixa",
        "badge": "⚠️ BAT BAIXA",
        "severity": "warning",
        "fg_color": "#7E5109",
        "text_color": "#FDEBD0",
        "border_color": "#F39C12"
    },
    proto.ERR_IMU_FAIL: {
        "name": "ERR_IMU_FAIL",
        "label": "Falha no MPU",
        "badge": "🧭 MPU FAIL",
        "severity": "warning",
        "fg_color": "#7E5109",
        "text_color": "#FDEBD0",
        "border_color": "#F39C12"
    },
    proto.ERR_MOTOR_1_STALL: {
        "name": "ERR_MOTOR_1_STALL",
        "label": "Motor 1 Travado",
        "badge": "🛑 STALL M1",
        "severity": "warning",
        "fg_color": "#6E2C00",
        "text_color": "#EDBB99",
        "border_color": "#E67E22"
    },
    proto.ERR_MOTOR_2_STALL: {
        "name": "ERR_MOTOR_2_STALL",
        "label": "Motor 2 Travado",
        "badge": "🛑 STALL M2",
        "severity": "warning",
        "fg_color": "#6E2C00",
        "text_color": "#EDBB99",
        "border_color": "#E67E22"
    },
    proto.ERR_MOTOR_3_STALL: {
        "name": "ERR_MOTOR_3_STALL",
        "label": "Motor 3 Travado",
        "badge": "🛑 STALL M3",
        "severity": "warning",
        "fg_color": "#6E2C00",
        "text_color": "#EDBB99",
        "border_color": "#E67E22"
    },
    proto.ERR_MOTOR_4_STALL: {
        "name": "ERR_MOTOR_4_STALL",
        "label": "Motor 4 Travado",
        "badge": "🛑 STALL M4",
        "severity": "warning",
        "fg_color": "#6E2C00",
        "text_color": "#EDBB99",
        "border_color": "#E67E22"
    },
    proto.ERR_BALL_SENSOR_DIRTY: {
        "name": "ERR_BALL_SENSOR_DIRTY",
        "label": "Sensor de Bola Sujo",
        "badge": "⚽ SENSOR BOLA",
        "severity": "info",
        "fg_color": "#1B4F72",
        "text_color": "#D4E6F1",
        "border_color": "#3498DB"
    },
}

def parse_robot_error_flags(error_flags: int):
    """
    Decodifica a máscara de bits e retorna lista de metadados dos erros ativos.
    Possui fallback automático para novos enums do Protobuf caso surjam no futuro.
    """
    if not error_flags:
        return []

    active_errors = []
    for bit_mask, meta in ROBOT_ERRORS_CONFIG.items():
        if error_flags & bit_mask:
            active_errors.append(meta)

    # Fallback dinâmico para novos bits não catalogados no dicionário
    known_mask = 0
    for bit_mask in ROBOT_ERRORS_CONFIG.keys():
        known_mask |= bit_mask

    unrecognized_bits = error_flags & (~known_mask)
    if unrecognized_bits:
        for val in proto.RobotError.values():
            if val != 0 and (unrecognized_bits & val):
                try:
                    name = proto.RobotError.Name(val)
                except Exception:
                    name = f"ERR_0x{val:X}"
                active_errors.append({
                    "name": name,
                    "label": name.replace("ERR_", "").replace("_", " "),
                    "badge": f"⚠️ {name.replace('ERR_', '')}",
                    "severity": "warning",
                    "fg_color": "#7E5109",
                    "text_color": "#FDEBD0",
                    "border_color": "#F39C12"
                })
                unrecognized_bits &= ~val

        if unrecognized_bits:
            active_errors.append({
                "name": f"UNKNOWN_0x{unrecognized_bits:X}",
                "label": f"Erro 0x{unrecognized_bits:X}",
                "badge": f"⚠️ 0x{unrecognized_bits:X}",
                "severity": "warning",
                "fg_color": "#5D6D7E",
                "text_color": "#EAECEE",
                "border_color": "#BDC3C7"
            })

    return active_errors

ROBOT_TIMEOUT_SECONDS = 3.0  # Tempo limite (s) sem pacotes para considerar robô desconectado

# Parâmetros físicos padrão de robô SSL (Small Size League - RoboCup)
ROBOT_RADIUS_R = 0.0915   # Distância do centro do robô ao centro da roda (m)
WHEEL_RADIUS_Rr = 0.020    # Raio da roda omnidirecional (m)
WHEEL_ANGLES_RAD = [
    math.radians(45),    # M1: Frontal Direita (FR) - 45°
    math.radians(135),   # M2: Frontal Esquerda (FL) - 135°
    math.radians(225),   # M3: Traseira Esquerda (BL) - 225°
    math.radians(315)    # M4: Traseira Direita (BR) - 315°
]

def calculate_omni4_inverse_kinematics(vx, vy, vw, r=WHEEL_RADIUS_Rr, R=ROBOT_RADIUS_R, angles=WHEEL_ANGLES_RAD):
    """
    Cinemática inversa de robô omnidirecional de 4 rodas (Small Size League).
    Retorna as velocidades angulares esperadas das rodas [w1, w2, w3, w4] em rad/s.
    Equação da roda i: w_i = (1 / r) * (-sin(alpha_i) * vx + cos(alpha_i) * vy + R * vw)
    """
    return [(-math.sin(a) * vx + math.cos(a) * vy + R * vw) / r for a in angles]


class RobotDataHistory:
    """Armazena histórico recente de telemetria e comandos com limite fixo de memória."""
    def __init__(self, maxlen=1200):
        self.maxlen = maxlen
        self.timestamps = deque(maxlen=maxlen)
        self.wheels_real = [deque(maxlen=maxlen) for _ in range(4)]
        self.wheels_exp = [deque(maxlen=maxlen) for _ in range(4)]
        self.vx = deque(maxlen=maxlen)
        self.vy = deque(maxlen=maxlen)
        self.vw = deque(maxlen=maxlen)
        self.battery = deque(maxlen=maxlen)
        self.kicker = deque(maxlen=maxlen)

    def add_point(self, t, battery, kicker, wheels_real, wheels_exp, motion):
        self.timestamps.append(t)
        self.battery.append(battery)
        self.kicker.append(kicker)
        for i in range(4):
            r_val = wheels_real[i] if (wheels_real and i < len(wheels_real)) else 0.0
            self.wheels_real[i].append(r_val)
            e_val = wheels_exp[i] if (wheels_exp and i < len(wheels_exp)) else 0.0
            self.wheels_exp[i].append(e_val)
        self.vx.append(motion[0] if motion else 0.0)
        self.vy.append(motion[1] if motion else 0.0)
        self.vw.append(motion[2] if motion else 0.0)

    def clear(self):
        self.timestamps.clear()
        for i in range(4):
            self.wheels_real[i].clear()
            self.wheels_exp[i].clear()
        self.vx.clear()
        self.vy.clear()
        self.vw.clear()
        self.battery.clear()
        self.kicker.clear()


class RobotPlotWindow(ctk.CTkToplevel):
    """Janela independente para visualização de variáveis no tempo em tempo real."""
    SIGNAL_OPTIONS = [
        "M1: Real vs Esperada",
        "M2: Real vs Esperada",
        "M3: Real vs Esperada",
        "M4: Real vs Esperada",
        "Todas as Rodas (Real)",
        "Todas as Rodas (Esperada)",
        "Velocidades Robô (Vx, Vy, Vw)",
        "Bateria (V)",
        "Chute (V)"
    ]

    WINDOW_OPTIONS = {
        "5s": 5.0,
        "10s": 10.0,
        "15s": 15.0,
        "30s": 30.0
    }

    def __init__(self, master, robot_id, history):
        super().__init__(master)
        self.robot_id = robot_id
        self.history = history
        self.master = master

        self.title(f"📈 Telemetria em Tempo Real - Robô ID {robot_id}")
        self.geometry("800x520")
        self.minsize(550, 380)

        self.is_paused = False
        self.is_running = True
        self.window_seconds = 10.0
        self.update_timer_id = None

        self._build_ui()
        self.protocol("WM_DELETE_WINDOW", self._on_close)
        self._update_plot()

    def _build_ui(self):
        # Barra superior com controles
        ctrl_frame = ctk.CTkFrame(self, corner_radius=8, fg_color="#2b2b2b")
        ctrl_frame.pack(fill="x", padx=12, pady=(10, 5))

        ctk.CTkLabel(ctrl_frame, text="Sinal:").pack(side="left", padx=(10, 5), pady=8)
        self.cb_signal = ctk.CTkOptionMenu(ctrl_frame, values=self.SIGNAL_OPTIONS, width=220)
        self.cb_signal.pack(side="left", padx=5)

        ctk.CTkLabel(ctrl_frame, text="Janela:").pack(side="left", padx=(15, 5))
        self.cb_window = ctk.CTkOptionMenu(ctrl_frame, values=list(self.WINDOW_OPTIONS.keys()), width=80,
                                           command=self._on_window_change)
        self.cb_window.set("10s")
        self.cb_window.pack(side="left", padx=5)

        self.btn_pause = ctk.CTkButton(ctrl_frame, text="⏸ Pausar", width=85, command=self._toggle_pause)
        self.btn_pause.pack(side="left", padx=10)

        ctk.CTkButton(ctrl_frame, text="🗑 Limpar", width=75, fg_color="#555555", hover_color="#333333",
                      command=self._clear_data).pack(side="left", padx=5)

        # Matplotlib Canvas com tema Dark
        self.fig = Figure(figsize=(8, 4.5), dpi=100, facecolor="#242424")
        self.ax = self.fig.add_subplot(111, facecolor="#1a1a1a")
        self.canvas = FigureCanvasTkAgg(self.fig, master=self)
        self.canvas.get_tk_widget().pack(fill="both", expand=True, padx=12, pady=(5, 12))

    def _on_window_change(self, value):
        self.window_seconds = self.WINDOW_OPTIONS.get(value, 10.0)

    def _toggle_pause(self):
        self.is_paused = not self.is_paused
        self.btn_pause.configure(text="▶ Retomar" if self.is_paused else "⏸ Pausar")

    def _clear_data(self):
        self.history.clear()

    def _update_plot(self):
        if not self.is_running:
            return

        if not self.is_paused:
            self._render_graph()

        self.update_timer_id = self.after(50, self._update_plot)

    def _render_graph(self):
        t_now = time.time()
        w_sec = self.window_seconds
        timestamps = list(self.history.timestamps)

        self.ax.clear()
        self.ax.set_facecolor("#1a1a1a")
        self.fig.patch.set_facecolor("#242424")
        self.ax.tick_params(colors="#aaaaaa", labelsize=9)
        for spine in self.ax.spines.values():
            spine.set_color("#444444")
        self.ax.grid(True, linestyle="--", alpha=0.3, color="#666666")
        self.ax.set_xlim(-w_sec, 0)
        self.ax.set_xlabel("Tempo relativo (s)", color="#aaaaaa", fontsize=9)

        if not timestamps:
            self.ax.text(0.5, 0.5, "Aguardando dados de telemetria...", color="#777777",
                         ha="center", va="center", transform=self.ax.transAxes, fontsize=11)
            self.ax.set_ylim(-1.0, 1.0)
            self.ax.axhline(0, color="#555555", linestyle="-", linewidth=0.9, alpha=0.8, zorder=1)
            self.canvas.draw_idle()
            return

        # Filtra pontos dentro da janela [-w_sec, 0]
        cutoff = t_now - w_sec
        idx_start = 0
        for i, t in enumerate(timestamps):
            if t >= cutoff:
                idx_start = i
                break

        rel_t = [t - t_now for t in timestamps[idx_start:]]
        if not rel_t:
            self.ax.set_ylim(-1.0, 1.0)
            self.ax.axhline(0, color="#555555", linestyle="-", linewidth=0.9, alpha=0.8, zorder=1)
            self.canvas.draw_idle()
            return

        signal = self.cb_signal.get()

        if signal.startswith("M") and "Real vs Esperada" in signal:
            wheel_idx = int(signal[1]) - 1
            y_real = list(self.history.wheels_real[wheel_idx])[idx_start:]
            y_exp = list(self.history.wheels_exp[wheel_idx])[idx_start:]
            self.ax.plot(rel_t, y_real, label=f"M{wheel_idx+1} Real (rad/s)", color="#00e5ff", linewidth=1.8)
            self.ax.plot(rel_t, y_exp, label=f"M{wheel_idx+1} Esperada (rad/s)", color="#ff9100", linestyle="--", linewidth=1.8)
            self.ax.set_ylabel("rad/s", color="#bbbbbb")
            self.ax.set_title(f"Robô ID {self.robot_id} - Roda {wheel_idx+1} (Real vs Esperada)", color="#ffffff", fontsize=11)

        elif signal == "Todas as Rodas (Real)":
            colors = ["#ffd600", "#00e5ff", "#00e676", "#e040fb"]
            for i in range(4):
                y = list(self.history.wheels_real[i])[idx_start:]
                self.ax.plot(rel_t, y, label=f"M{i+1} Real", color=colors[i], linewidth=1.6)
            self.ax.set_ylabel("rad/s", color="#bbbbbb")
            self.ax.set_title(f"Robô ID {self.robot_id} - Telemetria Real das 4 Rodas", color="#ffffff", fontsize=11)

        elif signal == "Todas as Rodas (Esperada)":
            colors = ["#ffd600", "#00e5ff", "#00e676", "#e040fb"]
            for i in range(4):
                y = list(self.history.wheels_exp[i])[idx_start:]
                self.ax.plot(rel_t, y, label=f"M{i+1} Exp", color=colors[i], linewidth=1.6)
            self.ax.set_ylabel("rad/s", color="#bbbbbb")
            self.ax.set_title(f"Robô ID {self.robot_id} - Velocidades Esperadas das 4 Rodas", color="#ffffff", fontsize=11)

        elif signal == "Velocidades Robô (Vx, Vy, Vw)":
            self.ax.plot(rel_t, list(self.history.vx)[idx_start:], label="Vx (m/s)", color="#ff5252", linewidth=1.6)
            self.ax.plot(rel_t, list(self.history.vy)[idx_start:], label="Vy (m/s)", color="#69f0ae", linewidth=1.6)
            self.ax.plot(rel_t, list(self.history.vw)[idx_start:], label="Vw (rad/s)", color="#448aff", linewidth=1.6)
            self.ax.set_ylabel("Velocidade", color="#bbbbbb")
            self.ax.set_title(f"Robô ID {self.robot_id} - Comandos de Velocidade", color="#ffffff", fontsize=11)

        elif signal == "Bateria (V)":
            self.ax.plot(rel_t, list(self.history.battery)[idx_start:], label="Bateria (V)", color="#ffd740", linewidth=2.0)
            self.ax.set_ylabel("Volts (V)", color="#bbbbbb")
            self.ax.set_title(f"Robô ID {self.robot_id} - Tensão da Bateria", color="#ffffff", fontsize=11)

        elif signal == "Chute (V)":
            self.ax.plot(rel_t, list(self.history.kicker)[idx_start:], label="Chute (V)", color="#ff5252", linewidth=2.0)
            self.ax.set_ylabel("Volts (V)", color="#bbbbbb")
            self.ax.set_title(f"Robô ID {self.robot_id} - Tensão do Chute", color="#ffffff", fontsize=11)

        # Garante que o zero (0.0) sempre permaneça visível no eixo Y
        y_min, y_max = self.ax.get_ylim()
        y_min = min(y_min, 0.0)
        y_max = max(y_max, 0.0)
        if y_min == y_max:
            y_min, y_max = -1.0, 1.0
        else:
            margin = 0.06 * (y_max - y_min)
            y_min -= margin
            y_max += margin
        self.ax.set_ylim(y_min, y_max)

        # Linha de referência no zero
        self.ax.axhline(0, color="#555555", linestyle="-", linewidth=0.9, alpha=0.8, zorder=1)

        self.ax.legend(facecolor="#252525", edgecolor="#444444", labelcolor="#ffffff", loc="upper left", fontsize=8)
        self.canvas.draw_idle()

    def _on_close(self):
        self.is_running = False
        if self.update_timer_id is not None:
            self.after_cancel(self.update_timer_id)
            self.update_timer_id = None
        self.master.plot_windows.pop(self.robot_id, None)
        self.destroy()


class RobotInterface:
    def __init__(self, msg_queue, UDP_IP="192.168.142.255", UDP_PORT=5000, TCP_IP="0.0.0.0", TCP_PORT=5001):
        self.running = True
        self.msg_queue = msg_queue
        self.UDP_IP = UDP_IP
        self.UDP_PORT = UDP_PORT
        self.TCP_IP = TCP_IP
        self.TCP_PORT = TCP_PORT

        self.udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.udp_sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)

        self.tcp_thread = threading.Thread(target=self._tcp_server_task)
        self.tcp_thread.daemon = True
        self.tcp_thread.start()

    def _send_udp_packet(self, packet):
        msg = packet.SerializeToString()
        self.udp_sock.sendto(msg, (self.UDP_IP, self.UDP_PORT))

    def send_motion_command(self, robot_id, vx, vy, vw, kick_h=0, kick_v=0):
        packet = proto.RobotPacket()
        packet.robot_id = int(robot_id)
        packet.motion.vel_x = float(vx)
        packet.motion.vel_y = float(vy)
        packet.motion.vel_w = float(vw)
        packet.motion.kick_h = int(kick_h)
        packet.motion.kick_v = int(kick_v)
        self._send_udp_packet(packet)

    def send_info_request(self, robot_id, info_index):
        packet = proto.RobotPacket()
        packet.robot_id = int(robot_id)
        packet.request.info_index = int(info_index)
        self._send_udp_packet(packet)

    def send_config_command(self, robot_id, param_id, value):
        packet = proto.RobotPacket()
        packet.robot_id = int(robot_id)
        packet.config.param_id = int(param_id)
        if isinstance(value, str):
            packet.config.text_value = value
        else:
            packet.config.value = float(value)
        self._send_udp_packet(packet)

    def _tcp_server_task(self):
        server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server.bind((self.TCP_IP, self.TCP_PORT))
        server.listen(5)
        while self.running:
            try:
                server.settimeout(1.0)
                client, addr = server.accept()
                threading.Thread(target=self._handle_robot_client, args=(client, addr), daemon=True).start()
            except socket.timeout:
                continue
            except Exception:
                break

    def _handle_robot_client(self, client_socket, addr):
        associated_robots = set()
        with client_socket:
            while self.running:
                try:
                    raw_msg_len = client_socket.recv(4)
                    if not raw_msg_len: break
                    msg_len = struct.unpack('<I', raw_msg_len)[0]
                    data = b''
                    while len(data) < msg_len:
                        chunk = client_socket.recv(msg_len - len(data))
                        if not chunk: break
                        data += chunk
                    packet = proto.RobotPacket()
                    packet.ParseFromString(data)
                    associated_robots.add(packet.robot_id)
                    self.msg_queue.put(packet)
                except:
                    break
        # Notifica a interface imediatamente sobre a desconexão do socket dos robôs associados
        for rid in associated_robots:
            self.msg_queue.put(("ROBOT_DISCONNECTED", rid))

    def broadcast_TCP_IP(self, ip: str):
        self.send_config_command(0xFF, 1, ip)


class RobotDashboard(ctk.CTk):
    def __init__(self):
        super().__init__()
        self.title("Controle de Frota - Naive Hermes")
        self.geometry("1100x900")
        ctk.set_appearance_mode("dark")

        self.keys_pressed = {"w": False, "a": False, "s": False, "d": False, "q": False, "e": False}
        self._key_release_timers = {}
        self._last_entry_focus_time = 0.0

        # Desativa auto-repeat das teclas de condução no nível do SO/X11
        set_x11_keys_autorepeat(False)

        self.bind("<KeyPress>", self._on_key_press)
        self.bind("<KeyRelease>", self._on_key_release)
        self.bind("<FocusOut>", self._on_focus_out)
        self.bind_all("<Button-1>", self._on_any_click, add="+")
        self.bind_all("<Escape>", lambda e: self.focus_set())
        self.bind_class("EntryFilter", "<KeyPress>", self._on_entry_key_press)
        self.bind_class("Entry", "<FocusIn>", self._on_entry_focus_in, add="+")

        self.msg_queue = queue.Queue()
        self.network = RobotInterface(msg_queue=self.msg_queue)
        self.robots_ui = {}
        self.plot_windows = {}
        self.robots_history = {}

        self.info_options = {"0 - IP da ESP32": 0, "1 - MAC da ESP32": 1, "2 - Solicitar Telemetria": 2}
        self.config_options = {"0 - Novo ID": 0, "1 - IP TCP": 1, "2 - Porta TCP": 2, "3 - Porta UDP": 3}

        self._build_header()
        self.scrollable_frame = ctk.CTkScrollableFrame(self, fg_color="transparent")
        self.scrollable_frame.pack(fill="both", expand=True, padx=20, pady=10)
        self.process_queue()

    def _open_robot_plot(self, robot_id):
        if robot_id in self.plot_windows:
            win = self.plot_windows[robot_id]
            try:
                win.lift()
                win.focus_force()
                return
            except:
                self.plot_windows.pop(robot_id, None)

        history = self.robots_history.setdefault(robot_id, RobotDataHistory())
        win = RobotPlotWindow(self, robot_id, history)
        self.plot_windows[robot_id] = win

    def _on_entry_focus_in(self, event):
        self._last_entry_focus_time = time.time()
        self._on_focus_out()
        w = event.widget
        tags = w.bindtags()
        if "EntryFilter" not in tags:
            w.bindtags(("EntryFilter",) + tags)

    def _on_entry_key_press(self, event):
        # Bloqueia qualquer caractere WASDQE residual que chegue logo após focar no campo
        if time.time() - self._last_entry_focus_time < 0.35:
            if event.keysym.lower() in ("w", "a", "s", "d", "q", "e"):
                return "break"

    def _on_any_click(self, event):
        w = event.widget
        is_entry = False
        while w is not None:
            if isinstance(w, (tk.Entry, tk.Text, ctk.CTkEntry)):
                is_entry = True
                break
            w = getattr(w, "master", None)

        if not is_entry:
            self.focus_set()

    def _on_key_press(self, event):
        focused = self.focus_get()
        if isinstance(focused, (tk.Entry, tk.Text)):
            return

        key = event.keysym.lower()
        if key in self.keys_pressed:
            timer = self._key_release_timers.pop(key, None)
            if timer is not None:
                self.after_cancel(timer)
            self.keys_pressed[key] = True
            return "break"

    def _on_key_release(self, event):
        focused = self.focus_get()
        if isinstance(focused, (tk.Entry, tk.Text)):
            return

        key = event.keysym.lower()
        if key in self.keys_pressed:
            timer = self._key_release_timers.pop(key, None)
            if timer is not None:
                self.after_cancel(timer)
            # Como o autorepeat do X11 para essas teclas está desativado, o release é imediato
            self.keys_pressed[key] = False
            return "break"

    def _on_focus_out(self, event=None):
        for timer in list(self._key_release_timers.values()):
            try:
                self.after_cancel(timer)
            except Exception:
                pass
        self._key_release_timers.clear()
        for k in self.keys_pressed:
            self.keys_pressed[k] = False

    def _build_header(self):
        header_frame = ctk.CTkFrame(self, corner_radius=10)
        header_frame.pack(fill="x", padx=20, pady=(20, 10))

        vcmd_int = (self.register(lambda P: P.isdigit() or P == ""), "%P")
        vcmd_ip = (self.register(lambda P: all(c in "0123456789." for c in P) and P.count(".") <= 3), "%P")

        ctk.CTkLabel(header_frame, text="Adicionar Robô (ID):").pack(side="left", padx=15, pady=15)
        self.entry_new_robot = ctk.CTkEntry(header_frame, width=80)
        self.entry_new_robot._entry.configure(validate="key", validatecommand=vcmd_int)
        self.entry_new_robot.pack(side="left", padx=10)
        self.entry_new_robot.bind("<Return>", lambda e: self._add_robot_manual())
        ctk.CTkButton(header_frame, text="Adicionar Painel", command=self._add_robot_manual).pack(side="left", padx=10)

        # Separador visual
        ctk.CTkLabel(header_frame, text="|", text_color="gray").pack(side="left", padx=10)

        # Campo de IP e botão de broadcast
        ctk.CTkLabel(header_frame, text="IP TCP:").pack(side="left", padx=(5, 5), pady=15)
        self.entry_tcp_ip = ctk.CTkEntry(header_frame, width=130, placeholder_text="192.168.0.100")
        self.entry_tcp_ip._entry.configure(validate="key", validatecommand=vcmd_ip)
        self.entry_tcp_ip.pack(side="left", padx=5)
        self.entry_tcp_ip.bind("<Return>", lambda e: (self._broadcast_tcp_ip(), self.focus_set()))
        ctk.CTkButton(header_frame, text="Broadcast IP", command=self._broadcast_tcp_ip).pack(side="left", padx=10)

    def _broadcast_tcp_ip(self):
        ip = self.entry_tcp_ip.get().strip()
        if ip:
            self.network.broadcast_TCP_IP(ip)
        self.focus_set()

    def _add_robot_manual(self):
        robot_id = self.entry_new_robot.get().strip()
        if robot_id.isdigit():
            self._get_or_create_robot_panel(int(robot_id))
            self.entry_new_robot.delete(0, 'end')
        self.focus_set()

    def _get_or_create_robot_panel(self, robot_id):
        if robot_id in self.robots_ui:
            return self.robots_ui[robot_id]

        panel = ctk.CTkFrame(self.scrollable_frame, corner_radius=15)
        panel.pack(fill="x", pady=10)

        # Cabeçalho do robô com ID, badges de status e botão do gráfico
        title_f = ctk.CTkFrame(panel, fg_color="transparent")
        title_f.pack(fill="x", padx=20, pady=(15, 5))
        ctk.CTkLabel(title_f, text=f"🤖 Robô ID: {robot_id}", font=("Roboto", 18, "bold"), text_color="#1f6aa5").pack(side="left")

        # Container para os badges dinâmicos de status/alerta
        badge_container = ctk.CTkFrame(title_f, fg_color="transparent")
        badge_container.pack(side="left", padx=(12, 0))

        ctk.CTkButton(title_f, text="📈 Abrir Gráfico", width=120, height=28, fg_color="#1f6aa5", hover_color="#144870",
                      command=lambda: self._open_robot_plot(robot_id)).pack(side="right")

        # --- TELEMETRIA ---
        tele_f = ctk.CTkFrame(panel, fg_color="transparent")
        tele_f.pack(fill="x", padx=20)

        lbl_bat = ctk.CTkLabel(tele_f, text="🔋 Bateria: -- V")
        lbl_bat.grid(row=0, column=0, sticky="w", padx=(0, 20))

        lbl_ball = ctk.CTkLabel(tele_f, text="⚽ Sensor: --")
        lbl_ball.grid(row=0, column=1, sticky="w", padx=(0, 20))

        lbl_wheels = ctk.CTkLabel(tele_f, text="⚙️ Rodas Real: [--]")
        lbl_wheels.grid(row=0, column=2, sticky="w", padx=(0, 20))

        lbl_exp = ctk.CTkLabel(tele_f, text="🎯 Rodas Exp: [M1: 0.0, M2: 0.0, M3: 0.0, M4: 0.0] rad/s", text_color="#5dade2")
        lbl_exp.grid(row=0, column=3, sticky="w")

        lbl_imu = ctk.CTkLabel(tele_f, text="🧭 IMU: Roll: -- | Pitch: -- | Yaw: -- | Gyro Z: --")
        lbl_imu.grid(row=1, column=0, columnspan=3, sticky="w", pady=(2, 0))

        # --- CONFIGURAÇÃO E REQUESTS ---
        actions_f = ctk.CTkFrame(panel, fg_color="transparent")
        actions_f.pack(fill="x", padx=20, pady=5)

        cb_req = ctk.CTkOptionMenu(actions_f, values=list(self.info_options.keys()), width=140)
        cb_req.grid(row=0, column=0, padx=5, pady=2)
        ctk.CTkButton(actions_f, text="Request", width=70,
                      command=lambda: self.network.send_info_request(robot_id, self.info_options[cb_req.get()])).grid(
            row=0, column=1)
        lbl_resp = ctk.CTkLabel(actions_f, text="Resp: --", text_color="gray")
        lbl_resp.grid(row=0, column=2, padx=10)

        cb_conf = ctk.CTkOptionMenu(actions_f, values=list(self.config_options.keys()), width=140)
        cb_conf.grid(row=1, column=0, padx=5, pady=2)
        ent_conf = ctk.CTkEntry(actions_f, placeholder_text="valor", width=100)
        ent_conf.grid(row=1, column=1, padx=5)
        ent_conf.bind("<Return>", lambda e: (self._send_config(robot_id, cb_conf.get(), ent_conf.get()), self.focus_set()))
        ctk.CTkButton(actions_f, text="Set Config", width=80,
                      command=lambda: (self._send_config(robot_id, cb_conf.get(), ent_conf.get()), self.focus_set())).grid(row=1, column=2)

        # --- MODO CONDUÇÃO ---
        drive_f = ctk.CTkFrame(panel, fg_color="#2b2b2b", corner_radius=10)
        drive_f.pack(fill="x", padx=20, pady=(5, 15))

        ctk.CTkLabel(drive_f, text="🎮 CONDUÇÃO WASD", font=("Roboto", 11, "bold")).grid(row=0, column=0, padx=10)
        ctk.CTkLabel(drive_f, text="Vel Max:").grid(row=0, column=1, padx=5)
        ent_speed = ctk.CTkEntry(drive_f, width=50)
        vcmd_speed = (self.register(lambda P: all(c in "0123456789." for c in P) and P.count(".") <= 1), "%P")
        ent_speed._entry.configure(validate="key", validatecommand=vcmd_speed)
        ent_speed.insert(0, "1.5")
        ent_speed.grid(row=0, column=2, padx=5)
        ent_speed.bind("<Return>", lambda e: self.focus_set())

        lbl_curr = ctk.CTkLabel(drive_f, text="Vx: 0.0 | Vy: 0.0 | Vw: 0.0", text_color="#777777")
        lbl_curr.grid(row=0, column=3, padx=15)

        switch_drive = ctk.CTkSwitch(drive_f, text="Ativar Teclado", command=lambda: self._toggle_drive(robot_id))
        switch_drive.grid(row=0, column=4, padx=10)

        btn_kicker = ctk.CTkButton(
            drive_f,
            text="⚡ Kicker: OFF",
            font=("Roboto", 11, "bold"),
            width=120,
            height=28,
            corner_radius=8,
            fg_color="#3a3a3a",
            hover_color="#4a4a4a",
            command=lambda: self._toggle_kicker(robot_id)
        )
        btn_kicker.grid(row=0, column=5, padx=10)

        self.robots_ui[robot_id] = {
            "panel": panel, "badge_container": badge_container, "current_error_flags": None,
            "last_packet_time": None, "is_connected": False,
            "lbl_bat": lbl_bat, "lbl_ball": lbl_ball, "lbl_wheels": lbl_wheels, "lbl_imu": lbl_imu, "lbl_resp": lbl_resp,
            "ent_speed": ent_speed, "lbl_current_v": lbl_curr, "lbl_expected_wheels": lbl_exp,
            "switch_drive": switch_drive, "btn_kicker": btn_kicker, "is_driving": False, "is_kicking": False, "drive_timer_id": None,
            "latest_motion": (0.0, 0.0, 0.0), "latest_exp": [0.0, 0.0, 0.0, 0.0],
            "latest_real": [0.0, 0.0, 0.0, 0.0], "latest_bat": 0.0, "latest_kicker": 0.0
        }
        # Inicia aguardando conexão
        lbl_waiting = ctk.CTkLabel(
            badge_container,
            text="⏳ AGUARDANDO",
            font=("Roboto", 11, "bold"),
            fg_color="#2C3E50",
            text_color="#BDC3C7",
            corner_radius=8,
            padx=8,
            pady=2
        )
        lbl_waiting.pack(side="left", padx=2)
        return self.robots_ui[robot_id]

    def _handle_robot_disconnect(self, robot_id):
        """Dispara o alerta local de robô desconectado na interface."""
        if robot_id not in self.robots_ui:
            return
        ui = self.robots_ui[robot_id]

        # Se já estava marcado como desconectado, não recria widgets
        if not ui.get("is_connected", False) and ui.get("last_packet_time") is not None:
            return

        ui["is_connected"] = False
        ui["current_error_flags"] = None
        if ui.get("is_kicking", False):
            ui["is_kicking"] = False
            btn = ui.get("btn_kicker")
            if btn:
                btn.configure(text="⚡ Kicker: OFF", fg_color="#3a3a3a", hover_color="#4a4a4a")
        panel = ui["panel"]
        badge_container = ui["badge_container"]

        for child in badge_container.winfo_children():
            child.destroy()

        lbl_disc = ctk.CTkLabel(
            badge_container,
            text="🔌 DESCONECTADO",
            font=("Roboto", 11, "bold"),
            fg_color="#641E16",
            text_color="#FADBD8",
            corner_radius=8,
            padx=8,
            pady=2
        )
        lbl_disc.pack(side="left", padx=2)
        panel.configure(border_width=2, border_color="#922B21")

    def _update_robot_status_badge(self, robot_id, error_flags, is_alert=False):
        """Atualiza dinamicamente os badges de status e a borda do card do robô."""
        if robot_id not in self.robots_ui:
            return
        ui = self.robots_ui[robot_id]

        # Evita reconstruir widgets se não houve mudança de estado (a menos que seja um alerta explícito)
        if not is_alert and ui.get("current_error_flags") == error_flags:
            return

        ui["current_error_flags"] = error_flags
        badge_container = ui["badge_container"]
        panel = ui["panel"]

        # Limpa badges anteriores
        for child in badge_container.winfo_children():
            child.destroy()

        active_errors = parse_robot_error_flags(error_flags)

        if not active_errors:
            # Estado normal: Badge verde discreto e sem borda de alerta
            lbl_ok = ctk.CTkLabel(
                badge_container,
                text="🟢 OK",
                font=("Roboto", 11, "bold"),
                fg_color="#143b23",
                text_color="#2ecc71",
                corner_radius=8,
                padx=8,
                pady=2
            )
            lbl_ok.pack(side="left", padx=2)
            panel.configure(border_width=0)
        else:
            # Define destaque da borda baseado na severidade máxima
            has_critical = any(e.get("severity") == "critical" for e in active_errors)
            has_warning = any(e.get("severity") == "warning" for e in active_errors)

            if has_critical:
                border_color = "#E74C3C"
            elif has_warning:
                border_color = "#F39C12"
            else:
                border_color = "#3498DB"

            panel.configure(border_width=2, border_color=border_color)

            # Se veio de um pacote assíncrono de alerta, adiciona tag indicativa
            if is_alert:
                lbl_alert_tag = ctk.CTkLabel(
                    badge_container,
                    text="⚡ ALERTA",
                    font=("Roboto", 10, "bold"),
                    fg_color="#922B21",
                    text_color="#FADBD8",
                    corner_radius=6,
                    padx=6,
                    pady=2
                )
                lbl_alert_tag.pack(side="left", padx=(0, 4))

            # Exibe os badges dos erros (limite de 3 para manter o layout limpo)
            max_visible = 3
            for err in active_errors[:max_visible]:
                b = ctk.CTkLabel(
                    badge_container,
                    text=err.get("badge", err.get("label", "ERRO")),
                    font=("Roboto", 11, "bold"),
                    fg_color=err.get("fg_color", "#78281F"),
                    text_color=err.get("text_color", "#FFFFFF"),
                    corner_radius=8,
                    padx=8,
                    pady=2
                )
                b.pack(side="left", padx=3)

            if len(active_errors) > max_visible:
                remaining = len(active_errors) - max_visible
                b_more = ctk.CTkLabel(
                    badge_container,
                    text=f"+{remaining}",
                    font=("Roboto", 10, "bold"),
                    fg_color="#424949",
                    text_color="#E5E8E8",
                    corner_radius=8,
                    padx=6,
                    pady=2
                )
                b_more.pack(side="left", padx=2)

    def _send_config(self, robot_id, cb_text, val_str):
        if not val_str: return
        p_id = self.config_options[cb_text]
        try:
            val = float(val_str) if ('.' in val_str or val_str.isdigit()) else val_str
        except:
            val = val_str
        self.network.send_config_command(robot_id, p_id, val)

    def _toggle_kicker(self, robot_id):
        ui = self.robots_ui.get(robot_id)
        if not ui:
            return

        is_kicking = not ui.get("is_kicking", False)
        ui["is_kicking"] = is_kicking

        btn = ui.get("btn_kicker")
        if btn:
            if is_kicking:
                btn.configure(text="⚡ Kicker: ATIVO", fg_color="#c0392b", hover_color="#e74c3c")
            else:
                btn.configure(text="⚡ Kicker: OFF", fg_color="#3a3a3a", hover_color="#4a4a4a")

        self.focus_set()

        if is_kicking:
            if ui.get("is_driving", False):
                vx, vy, vw = ui.get("latest_motion", (0.0, 0.0, 0.0))
                self.network.send_motion_command(robot_id, vx, vy, vw, kick_h=1)
            else:
                self.network.send_motion_command(robot_id, 0, 0, 0, kick_h=1)
                if ui.get("drive_timer_id") is None:
                    self._drive_loop(robot_id)
        else:
            if ui.get("is_driving", False):
                vx, vy, vw = ui.get("latest_motion", (0.0, 0.0, 0.0))
                self.network.send_motion_command(robot_id, vx, vy, vw, kick_h=0)
            else:
                if ui.get("drive_timer_id") is not None:
                    self.after_cancel(ui["drive_timer_id"])
                    ui["drive_timer_id"] = None
                self.network.send_motion_command(robot_id, 0, 0, 0, kick_h=0)

    def _toggle_drive(self, robot_id):
        ui = self.robots_ui[robot_id]
        is_driving = ui["switch_drive"].get() == 1
        ui["is_driving"] = is_driving

        if ui["drive_timer_id"] is not None:
            self.after_cancel(ui["drive_timer_id"])
            ui["drive_timer_id"] = None

        if is_driving:
            self.focus_set()
            self._drive_loop(robot_id)
        else:
            is_kicking = ui.get("is_kicking", False)
            kick_val = 1 if is_kicking else 0
            self.network.send_motion_command(robot_id, 0, 0, 0, kick_h=kick_val)
            ui["lbl_current_v"].configure(text="Vx: 0.0 | Vy: 0.0 | Vw: 0.0")
            ui["lbl_expected_wheels"].configure(text="🎯 Rodas Exp: [M1: 0.0, M2: 0.0, M3: 0.0, M4: 0.0] rad/s")
            ui["latest_motion"] = (0.0, 0.0, 0.0)
            ui["latest_exp"] = [0.0, 0.0, 0.0, 0.0]
            ui["last_v_tuple"] = (0.0, 0.0, 0.0)
            if is_kicking:
                self._drive_loop(robot_id)

    def _drive_loop(self, robot_id):
        ui = self.robots_ui.get(robot_id)
        if not ui:
            return

        is_driving = ui.get("is_driving", False)
        is_kicking = ui.get("is_kicking", False)

        if not is_driving and not is_kicking:
            self.network.send_motion_command(robot_id, 0, 0, 0, kick_h=0)
            ui["lbl_current_v"].configure(text="Vx: 0.0 | Vy: 0.0 | Vw: 0.0")
            ui["lbl_expected_wheels"].configure(text="🎯 Rodas Exp: [M1: 0.0, M2: 0.0, M3: 0.0, M4: 0.0] rad/s")
            ui["latest_motion"] = (0.0, 0.0, 0.0)
            ui["latest_exp"] = [0.0, 0.0, 0.0, 0.0]
            ui["last_v_tuple"] = (0.0, 0.0, 0.0)
            ui["drive_timer_id"] = None
            return

        try:
            kick_val = 1 if is_kicking else 0
            if is_driving:
                max_s = float(ui["ent_speed"].get())
                vx, vy, vw = 0.0, 0.0, 0.0

                if self.keys_pressed["w"]:
                    vy = max_s
                elif self.keys_pressed["s"]:
                    vy = -max_s

                if self.keys_pressed["d"]:
                    vx = max_s
                elif self.keys_pressed["a"]:
                    vx = -max_s

                if self.keys_pressed["q"]:
                    vw = max_s
                elif self.keys_pressed["e"]:
                    vw = -max_s
            else:
                vx, vy, vw = 0.0, 0.0, 0.0

            self.network.send_motion_command(robot_id, vx, vy, vw, kick_h=kick_val)
            w_exp = calculate_omni4_inverse_kinematics(vx, vy, vw)

            # Evita reconfigurar os widgets CustomTkinter se a velocidade não se alterou
            current_v_tuple = (vx, vy, vw)
            if ui.get("last_v_tuple") != current_v_tuple:
                ui["last_v_tuple"] = current_v_tuple
                ui["lbl_current_v"].configure(text=f"Vx: {vx:.1f} | Vy: {vy:.1f} | Vw: {vw:.1f}")
                w_str = f"M1: {w_exp[0]:+.1f}, M2: {w_exp[1]:+.1f}, M3: {w_exp[2]:+.1f}, M4: {w_exp[3]:+.1f}"
                ui["lbl_expected_wheels"].configure(text=f"🎯 Rodas Exp: [{w_str}] rad/s")

            ui["latest_motion"] = (vx, vy, vw)
            ui["latest_exp"] = w_exp
            history = self.robots_history.setdefault(robot_id, RobotDataHistory())
            history.add_point(time.time(), ui["latest_bat"], ui["latest_kicker"], ui["latest_real"], ui["latest_exp"], ui["latest_motion"])
        except:
            pass

        ui["drive_timer_id"] = self.after(20, lambda: self._drive_loop(robot_id))

    def process_queue(self):
        robots_with_telemetry = set()
        while not self.msg_queue.empty():
            try:
                item = self.msg_queue.get_nowait()
                if isinstance(item, tuple) and item[0] == "ROBOT_DISCONNECTED":
                    self._handle_robot_disconnect(item[1])
                else:
                    ptype = item.WhichOneof("payload")
                    if ptype == 'telemetry':
                        self._process_telemetry_packet(item)
                        robots_with_telemetry.add(item.robot_id)
                    elif ptype == 'alert':
                        err_flags = int(item.alert)
                        self._update_robot_status_badge(item.robot_id, err_flags, is_alert=True)
                    elif ptype == 'response':
                        ui = self._get_or_create_robot_panel(item.robot_id)
                        r = item.response
                        v = r.text_value if r.text_value else str(getattr(r, 'value', ''))
                        ui["lbl_resp"].configure(text=f"Resp: {v}", text_color="green")
            except Exception:
                break

        # Renderiza os labels de texto da interface apenas UMA VEZ por ciclo por robô com o dado mais recente
        for robot_id in robots_with_telemetry:
            self._render_robot_telemetry_labels(robot_id)

        self._check_timeouts()
        self.after(50, self.process_queue)

    def _check_timeouts(self):
        """Verifica se algum robô previamente conectado excedeu o timeout de heartbeat."""
        now = time.time()
        for robot_id, ui in self.robots_ui.items():
            last_t = ui.get("last_packet_time")
            if last_t is not None and ui.get("is_connected", False):
                if now - last_t > ROBOT_TIMEOUT_SECONDS:
                    self._handle_robot_disconnect(robot_id)

    def _process_telemetry_packet(self, packet):
        ui = self._get_or_create_robot_panel(packet.robot_id)
        ui["last_packet_time"] = time.time()
        ui["is_connected"] = True

        t = packet.telemetry
        bat = float(getattr(t, 'battery_voltage', 0.0))
        ui["latest_bat"] = bat
        wheels = list(t.wheel_speeds)
        ui["latest_real"] = wheels if wheels else [0.0, 0.0, 0.0, 0.0]
        kicker_volts = list(t.kicker_voltages)
        ui["latest_kicker"] = float(kicker_volts[0]) if kicker_volts else 0.0

        # Grava todos os pontos no histórico para o gráfico sem perdas
        history = self.robots_history.setdefault(packet.robot_id, RobotDataHistory())
        history.add_point(time.time(), ui["latest_bat"], ui["latest_kicker"], ui["latest_real"], ui["latest_exp"], ui["latest_motion"])

        # Guarda a telemetria mais recente para renderização agregada
        ui["pending_telemetry_render"] = t

    def _render_robot_telemetry_labels(self, robot_id):
        ui = self.robots_ui.get(robot_id)
        if not ui or "pending_telemetry_render" not in ui:
            return
        t = ui.pop("pending_telemetry_render")

        bat = ui["latest_bat"]
        ui["lbl_bat"].configure(text=f"🔋 Bateria: {bat:.2f} V")
        ui["lbl_ball"].configure(text=f"⚽ Sensor: {'🟢 Sim' if getattr(t, 'ball_sensor', False) else '🔴 Não'}")

        wheels = ui["latest_real"]
        if wheels and any(w != 0.0 for w in wheels):
            wheels_str = ", ".join([f"{w:.2f}" for w in wheels])
            ui["lbl_wheels"].configure(text=f"⚙️ Rodas Real: [{wheels_str}]")
        else:
            ui["lbl_wheels"].configure(text="⚙️ Rodas Real: [--]")

        r = getattr(t, 'roll', 0.0)
        p = getattr(t, 'pitch', 0.0)
        y = getattr(t, 'yaw', 0.0)
        gz = getattr(t, 'gyro_z', 0.0)
        ui["lbl_imu"].configure(text=f"🧭 IMU: Roll: {r:.2f} | Pitch: {p:.2f} | Yaw: {y:.2f} | Gyro Z: {gz:.2f}")

        err_flags = int(getattr(t, 'error_flags', 0))
        self._update_robot_status_badge(robot_id, err_flags)


if __name__ == "__main__":
    app = RobotDashboard()

    def on_close():
        set_x11_keys_autorepeat(True)
        app.network.running = False
        for win in list(app.plot_windows.values()):
            try:
                win.destroy()
            except:
                pass
        app.destroy()

    app.protocol("WM_DELETE_WINDOW", on_close)
    app.mainloop()