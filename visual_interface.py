import customtkinter as ctk
import socket
import threading
import queue
import struct
import proto.robot_comm_pb2 as proto


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
                    self.msg_queue.put(packet)
                except:
                    break

    def broadcast_TCP_IP(self, ip: str):
        self.send_config_command(0xFF, 1, ip)


class RobotDashboard(ctk.CTk):
    def __init__(self):
        super().__init__()
        self.title("Controle de Frota - Naive Hermes")
        self.geometry("1100x900")
        ctk.set_appearance_mode("dark")

        self.keys_pressed = {"w": False, "a": False, "s": False, "d": False, "q": False, "e": False}
        self.bind("<KeyPress>", self._on_key_press)
        self.bind("<KeyRelease>", self._on_key_release)

        self.msg_queue = queue.Queue()
        self.network = RobotInterface(msg_queue=self.msg_queue)
        self.robots_ui = {}

        self.info_options = {"0 - IP da ESP32": 0, "1 - MAC da ESP32": 1, "2 - Solicitar Telemetria": 2}
        self.config_options = {"0 - Novo ID": 0, "1 - IP TCP": 1, "2 - Porta TCP": 2, "3 - Porta UDP": 3}

        self._build_header()
        self.scrollable_frame = ctk.CTkScrollableFrame(self, fg_color="transparent")
        self.scrollable_frame.pack(fill="both", expand=True, padx=20, pady=10)
        self.process_queue()

    def _on_key_press(self, event):
        key = event.keysym.lower()
        if key in self.keys_pressed: self.keys_pressed[key] = True

    def _on_key_release(self, event):
        key = event.keysym.lower()
        if key in self.keys_pressed: self.keys_pressed[key] = False

    def _build_header(self):
        header_frame = ctk.CTkFrame(self, corner_radius=10)
        header_frame.pack(fill="x", padx=20, pady=(20, 10))

        ctk.CTkLabel(header_frame, text="Adicionar Robô (ID):").pack(side="left", padx=15, pady=15)
        self.entry_new_robot = ctk.CTkEntry(header_frame, width=80)
        self.entry_new_robot.pack(side="left", padx=10)
        ctk.CTkButton(header_frame, text="Adicionar Painel", command=self._add_robot_manual).pack(side="left", padx=10)

        # Separador visual
        ctk.CTkLabel(header_frame, text="|", text_color="gray").pack(side="left", padx=10)

        # Campo de IP e botão de broadcast
        ctk.CTkLabel(header_frame, text="IP TCP:").pack(side="left", padx=(5, 5), pady=15)
        self.entry_tcp_ip = ctk.CTkEntry(header_frame, width=130, placeholder_text="192.168.0.100")
        self.entry_tcp_ip.pack(side="left", padx=5)
        ctk.CTkButton(header_frame, text="Broadcast IP", command=self._broadcast_tcp_ip).pack(side="left", padx=10)

    def _broadcast_tcp_ip(self):
        ip = self.entry_tcp_ip.get().strip()
        if ip:
            self.network.broadcast_TCP_IP(ip)

    def _add_robot_manual(self):
        robot_id = self.entry_new_robot.get()
        if robot_id.isdigit():
            self._get_or_create_robot_panel(int(robot_id))
            self.entry_new_robot.delete(0, 'end')

    def _get_or_create_robot_panel(self, robot_id):
        if robot_id in self.robots_ui:
            return self.robots_ui[robot_id]

        panel = ctk.CTkFrame(self.scrollable_frame, corner_radius=15)
        panel.pack(fill="x", pady=10)

        ctk.CTkLabel(panel, text=f"🤖 Robô ID: {robot_id}", font=("Roboto", 18, "bold"), text_color="#1f6aa5").pack(
            anchor="w", padx=20, pady=(15, 5))

        # --- TELEMETRIA ---
        tele_f = ctk.CTkFrame(panel, fg_color="transparent")
        tele_f.pack(fill="x", padx=20)

        lbl_bat = ctk.CTkLabel(tele_f, text="🔋 Bateria: -- V")
        lbl_bat.grid(row=0, column=0, sticky="w", padx=(0, 20))

        lbl_ball = ctk.CTkLabel(tele_f, text="⚽ Sensor: --")
        lbl_ball.grid(row=0, column=1, sticky="w", padx=(0, 20))

        lbl_wheels = ctk.CTkLabel(tele_f, text="⚙️ Rodas: [--]")
        lbl_wheels.grid(row=0, column=2, sticky="w")

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
        ctk.CTkButton(actions_f, text="Set Config", width=80,
                      command=lambda: self._send_config(robot_id, cb_conf.get(), ent_conf.get())).grid(row=1, column=2)

        # --- MODO CONDUÇÃO ---
        drive_f = ctk.CTkFrame(panel, fg_color="#2b2b2b", corner_radius=10)
        drive_f.pack(fill="x", padx=20, pady=(5, 15))

        ctk.CTkLabel(drive_f, text="🎮 CONDUÇÃO WASD", font=("Roboto", 11, "bold")).grid(row=0, column=0, padx=10)
        ctk.CTkLabel(drive_f, text="Vel Max:").grid(row=0, column=1, padx=5)
        ent_speed = ctk.CTkEntry(drive_f, width=50)
        ent_speed.insert(0, "1.5")
        ent_speed.grid(row=0, column=2, padx=5)

        lbl_curr = ctk.CTkLabel(drive_f, text="Vx: 0.0 | Vy: 0.0 | Vw: 0.0", text_color="#777777")
        lbl_curr.grid(row=0, column=3, padx=15)

        switch_drive = ctk.CTkSwitch(drive_f, text="Ativar Teclado", command=lambda: self._toggle_drive(robot_id))
        switch_drive.grid(row=0, column=4, padx=10)

        self.robots_ui[robot_id] = {
            "lbl_bat": lbl_bat, "lbl_ball": lbl_ball, "lbl_wheels": lbl_wheels, "lbl_resp": lbl_resp,
            "ent_speed": ent_speed, "lbl_current_v": lbl_curr, "switch_drive": switch_drive,
            "is_driving": False
        }
        return self.robots_ui[robot_id]

    def _send_config(self, robot_id, cb_text, val_str):
        if not val_str: return
        p_id = self.config_options[cb_text]
        try:
            val = float(val_str) if ('.' in val_str or val_str.isdigit()) else val_str
        except:
            val = val_str
        self.network.send_config_command(robot_id, p_id, val)

    def _toggle_drive(self, robot_id):
        ui = self.robots_ui[robot_id]
        ui["is_driving"] = ui["switch_drive"].get() == 1
        if ui["is_driving"]: self._drive_loop(robot_id)

    def _drive_loop(self, robot_id):
        ui = self.robots_ui.get(robot_id)
        if not ui or not ui["is_driving"]:
            self.network.send_motion_command(robot_id, 0, 0, 0)
            if ui: ui["lbl_current_v"].configure(text="Vx: 0.0 | Vy: 0.0 | Vw: 0.0")
            return

        try:
            max_s = float(ui["ent_speed"].get())
            vx, vy, vw = 0.0, 0.0, 0.0

            if self.keys_pressed["w"]:
                vx = max_s
            elif self.keys_pressed["s"]:
                vx = -max_s

            if self.keys_pressed["d"]:
                vy = max_s
            elif self.keys_pressed["a"]:
                vy = -max_s

            if self.keys_pressed["q"]:
                vw = max_s
            elif self.keys_pressed["e"]:
                vw = -max_s

            self.network.send_motion_command(robot_id, vx, vy, vw)
            ui["lbl_current_v"].configure(text=f"Vx: {vx:.1f} | Vy: {vy:.1f} | Vw: {vw:.1f}")
        except:
            pass

        self.after(20, lambda: self._drive_loop(robot_id))

    def process_queue(self):
        while not self.msg_queue.empty():
            try:
                pkt = self.msg_queue.get_nowait()
                self._update_ui_from_packet(pkt)
            except:
                break
        self.after(100, self.process_queue)

    def _update_ui_from_packet(self, packet):
        ui = self._get_or_create_robot_panel(packet.robot_id)
        ptype = packet.WhichOneof("payload")

        if ptype == 'telemetry':
            t = packet.telemetry
            ui["lbl_bat"].configure(text=f"🔋 Bateria: {getattr(t, 'battery_voltage', 0.0):.2f} V")
            ui["lbl_ball"].configure(text=f"⚽ Sensor: {'🟢 Sim' if getattr(t, 'ball_sensor', False) else '🔴 Não'}")

            wheels = list(t.wheel_speeds)
            if wheels:
                wheels_str = ", ".join([f"{w:.2f}" for w in wheels])
                ui["lbl_wheels"].configure(text=f"⚙️ Rodas: [{wheels_str}]")
            else:
                ui["lbl_wheels"].configure(text="⚙️ Rodas: [--]")

        elif ptype == 'response':
            r = packet.response
            v = r.text_value if r.text_value else str(getattr(r, 'value', ''))
            ui["lbl_resp"].configure(text=f"Resp: {v}", text_color="green")


if __name__ == "__main__":
    app = RobotDashboard()

    def on_close():
        app.network.running = False
        app.destroy()

    app.protocol("WM_DELETE_WINDOW", on_close)
    app.mainloop()