import socket
import struct
import threading
import time
import proto.robot_comm_pb2 as proto


class RobotInterface:
    def __init__(self, UDP_IP="192.168.142.255", UDP_PORT=5000, RX_UDP_IP="0.0.0.0", RX_UDP_PORT=5001):
        self.running = True
        self.UDP_IP = UDP_IP
        self.UDP_PORT = UDP_PORT
        self.RX_UDP_IP = RX_UDP_IP
        self.RX_UDP_PORT = RX_UDP_PORT
        self._tx_seq_counters = {}

        # Setup UDP (Sender)
        self.udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.udp_sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)

        # Iniciar Thread de Telemetria/Respostas/Alertas (UDP Receiver)
        self.rx_thread = threading.Thread(target=self._udp_rx_server_task, daemon=True)
        self.rx_thread.start()

    def _send_udp_packet(self, packet, target_ip=None):
        dest_ip = target_ip if target_ip else self.UDP_IP
        seq_key = (int(packet.robot_id), dest_ip)
        next_seq = (self._tx_seq_counters.get(seq_key, 0) + 1) & 0xFFFFFFFF
        self._tx_seq_counters[seq_key] = next_seq
        packet.sequence_id = next_seq

        msg = packet.SerializeToString()
        self.udp_sock.sendto(msg, (dest_ip, self.UDP_PORT))

    def send_motion_command(self, robot_id, vx, vy, vw, kick_h=0, kick_v=0, target_ip=None):
        """Envia comando de movimentação para um robô específico via UDP"""
        packet = proto.RobotPacket()
        packet.robot_id = int(robot_id)

        packet.motion.vel_x = float(vx)
        packet.motion.vel_y = float(vy)
        packet.motion.vel_w = float(vw)
        packet.motion.kick_h = int(kick_h)
        packet.motion.kick_v = int(kick_v)

        self._send_udp_packet(packet, target_ip=target_ip)
        print(f"[UDP] Comando de Movimento enviado para Robô {robot_id}")

    def send_info_request(self, robot_id, info_index, target_ip=None):
        """Solicita informações (0=IP, 1=MAC, 2=Telemetria, 3=Perda de pacotes) via UDP"""
        packet = proto.RobotPacket()
        packet.robot_id = int(robot_id)
        packet.request.info_index = int(info_index)

        self._send_udp_packet(packet, target_ip=target_ip)
        print(f"[UDP] Request (index {info_index}) enviado para Robô {robot_id}")

    def send_config_command(self, robot_id, param_id, value, target_ip=None):
        """
        Envia configurações para o robô (0=Novo ID, 4=PID Kp, 5=PID Ki, 6=PID Kd).
        """
        packet = proto.RobotPacket()
        packet.robot_id = int(robot_id)
        packet.config.param_id = int(param_id)

        if isinstance(value, str):
            packet.config.text_value = value
        else:
            packet.config.value = float(value)

        self._send_udp_packet(packet, target_ip=target_ip)
        print(f"[UDP] Config {param_id} enviada: {value}")

    def _udp_rx_server_task(self):
        """Receptor UDP que recebe telemetria, respostas e alertas dos robôs"""
        rx_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        rx_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        rx_sock.bind((self.RX_UDP_IP, self.RX_UDP_PORT))
        rx_sock.settimeout(1.0)
        print(f"[UDP RX] Escutando telemetria na porta {self.RX_UDP_PORT}...")

        with rx_sock:
            while self.running:
                try:
                    data, addr = rx_sock.recvfrom(4096)
                    if not data:
                        continue

                    packet = proto.RobotPacket()
                    if len(data) > 4 and struct.unpack('<I', data[:4])[0] == len(data) - 4:
                        try:
                            packet.ParseFromString(data[4:])
                            if packet.WhichOneof("payload") is None:
                                packet.ParseFromString(data)
                        except Exception:
                            packet.ParseFromString(data)
                    else:
                        packet.ParseFromString(data)

                    payload_type = packet.WhichOneof("payload")
                    if payload_type == 'telemetry':
                        pass  # Processar telemetria
                    elif payload_type == 'alert':
                        print(f"[ALERTA] Robô {packet.robot_id} ({addr[0]}): flags=0x{packet.alert:04X}")
                    elif payload_type == 'response':
                        r = packet.response
                        print(f"[RESPOSTA] Robô {packet.robot_id} ({addr[0]}) Índice {r.info_index}: {r.text_value or r.value}")

                except socket.timeout:
                    continue
                except Exception as e:
                    print(f"[ERRO UDP RX] Falha ao processar dados: {e}")


# --- Exemplo de Uso ---
if __name__ == "__main__":
    robot = RobotInterface()

    try:
        time.sleep(1)
        # Exemplo: ajustar ganhos PID globais do robô 0 (4=Kp, 5=Ki, 6=Kd)
        robot.send_config_command(robot_id=0, param_id=4, value=0.08)
        robot.send_config_command(robot_id=0, param_id=5, value=0.20)
        robot.send_config_command(robot_id=0, param_id=6, value=0.00)

        while True:
            robot.send_motion_command(robot_id=0, vx=0.2, vy=0.0, vw=0.0)
            time.sleep(2)
            robot.send_info_request(robot_id=0, info_index=1)
            time.sleep(2)

    except KeyboardInterrupt:
        robot.running = False
