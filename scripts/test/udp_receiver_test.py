import socket
import struct

# --- UDP 配置 (必须和发送端完全一致) ---
UDP_IP = "0.0.0.0"  # 监听所有IP
UDP_PORT = 20002

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))

print(f"📡 临时UDP接收器已启动，正在监听端口 {UDP_PORT}...")

while True:
    data, addr = sock.recvfrom(1024) # buffer size is 1024 bytes
    
    # '7f' 对应7个float, 7 * 4 = 28字节
    if len(data) == 28:
        unpacked_data = struct.unpack('7f', data)
        px, py, pz, ox, oy, oz, ow = unpacked_data
        print(f"接收到数据 -> POS:({px:.2f}, {py:.2f}, {pz:.2f}), ORI:({ox:.2f}, {oy:.2f}, {oz:.2f}, {ow:.2f})")