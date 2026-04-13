"""
KCL Underwater Glider — TinyML Gateway & Serial Bridge
======================================================
Listens to Physics on UDP 5005, Sends Commands on UDP 5006.
Forwards data to ESP32 Serial at 1Hz when surfaced.
"""
import socket
import json
import serial
import sys
import numpy as np
import collections
import time

# Networking & Serial
UDP_IP = "127.0.0.1"
RX_PORT = 5005
TX_PORT = 5006
SERIAL_PORT = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyUSB0"

sock_rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock_rx.bind((UDP_IP, RX_PORT))
sock_tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

ser = None
try:
    ser = serial.Serial(SERIAL_PORT, 115200, timeout=0.1)
    print(f"\n[OK] Serial connected: {SERIAL_PORT}")
except Exception as e:
    print(f"\n[ERR] Serial failed: {e}\nContinue without serial...")

# ML Params
WINDOW_SIZE = 64
alert_state = 0
consecutive_anomalies = 0
buffer = collections.deque(maxlen=WINDOW_SIZE)
last_tx_time = 0

print("\n" + "="*50)
print("  TINYML GATEWAY IS RUNNING")
print("  Waiting for data from Physics Sim...")
print("="*50 + "\n")

def run_ml(data_buf):
    # Mock logic based on pH/EC
    ph_vals = [d['ph'] for d in data_buf]
    if np.mean(ph_vals) < 7.2: return 3 # Anomaly
    return 0

while True:
    # 1. Receive Physics Data
    data, addr = sock_rx.recvfrom(1024)
    phys = json.loads(data.decode())
    depth = phys['depth']
    
    # 2. Generate Sensor Data (Depth-dependent)
    # If we want to simulate a leak, we can manually trigger it here or via another input
    # For now, let's keep it normal unless depth is extreme
    ph = 8.1 + np.random.normal(0, 0.02)
    ec = 50.0 + np.random.normal(0, 0.1)
    temp = 25.0 - 1.5 * depth
    
    buffer.append({'ph':ph, 'ec':ec, 'depth':depth})
    
    # 3. TinyML Logic
    if len(buffer) == WINDOW_SIZE:
        res = run_ml(buffer)
        if res != 0:
            consecutive_anomalies += 1
            if consecutive_anomalies >= 15 and alert_state != 3:
                alert_state = 3
                print("\a[!!!] TINYML EMERGENCY DETECTED! Sending SURFACING command...")
                sock_tx.sendto(json.dumps({'command':'EMERGENCY'}).encode(), (UDP_IP, TX_PORT))
        else:
            consecutive_anomalies = 0

    # 4. Serial Out (Surfaced & 1Hz)
    curr_time = time.time()
    if depth < 0.3 and (curr_time - last_tx_time) >= 1.0:
        msg = f"D:{depth:.2f},T:{temp:.2f},PH:{ph:.2f},EC:{ec:.3f},O2:8.1,VOLT:8.4\n"
        if ser:
            ser.write(msg.encode())
            print(f"\033[92m[TX >> ESP32]\033[0m {msg.strip()}")
        else:
            print(f"\033[90m[TX (No Serial)]\033[0m {msg.strip()}")
        last_tx_time = curr_time

    # Heartbeat print
    if int(phys['t']*10) % 50 == 0:
        print(f"[STATUS] t={phys['t']:.1f}s | Depth={depth:.2f}m | State={phys['state']} | ML={alert_state}")
