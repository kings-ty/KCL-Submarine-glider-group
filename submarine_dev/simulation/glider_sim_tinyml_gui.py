"""
KCL Underwater Glider — TinyML Monitor & Controller GUI (PRO)
============================================================
- Features: Anomaly Injection, ML Reset, Force Surface, Serial Bridge.
- Communication: UDP 5005 (Rx), UDP 5006 (Tx).
"""
import sys
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.gridspec import GridSpec
from matplotlib.widgets import Button
import socket
import json
import collections
import serial
import threading
import time

# Networking
UDP_IP = "127.0.0.1"
RX_PORT, TX_PORT = 5005, 5006
sock_rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock_rx.bind((UDP_IP, RX_PORT))
sock_rx.setblocking(False)
sock_tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# Serial
SERIAL_PORT = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyUSB0"
ser = None
try:
    ser = serial.Serial(SERIAL_PORT, 115200, timeout=0.01)
    print(f"[SERIAL] Connected to {SERIAL_PORT}")
except: pass

# ML State
STATE_NAMES = ["NORMAL", "WATCHING", "CONFIRMED", "EMERGENCY"]
alert_state = 0
consecutive_anomalies = 0
anomaly_active = False
anomaly_type = 'NONE'

# Data Buffers
history = collections.defaultdict(lambda: collections.deque(maxlen=100))
phys_state = {'depth': 0.0, 't': 0.0, 'state': -1}

def listen_phys():
    while True:
        try:
            data, _ = sock_rx.recvfrom(1024)
            msg = json.loads(data.decode())
            # 물리 엔진이 새로 출발(DEPLOY=0)하면 ML 상태 자동 초기화
            if phys_state['state'] != 0 and msg['state'] == 0:
                reset_ml(None)
            phys_state['depth'] = msg['depth']
            phys_state['t'] = msg['t']
            phys_state['state'] = msg['state']
        except: pass

threading.Thread(target=listen_phys, daemon=True).start()

# UI Setup
fig = plt.figure(figsize=(12, 8), facecolor='#121212')
fig.suptitle('AUV TinyML Monitor & Controller', color='white', fontsize=14, fontweight='bold')
gs = GridSpec(4, 3, figure=fig, hspace=0.6, wspace=0.3, left=0.08, right=0.95, top=0.9, bottom=0.15)

ax_ph = fig.add_subplot(gs[0, 0:2]); ax_ec = fig.add_subplot(gs[1, 0:2]); ax_do = fig.add_subplot(gs[2, 0:2])
ax_status = fig.add_subplot(gs[0:3, 2])

for a in [ax_ph, ax_ec, ax_do]:
    a.set_facecolor('#1e1e1e'); a.tick_params(colors='gray', labelsize=8)

l_ph, = ax_ph.plot([], [], '#ff5555', linewidth=1.5); ax_ph.set_title('pH Sensor', color='white', fontsize=10)
l_ec, = ax_ec.plot([], [], '#5555ff', linewidth=1.5); ax_ec.set_title('EC Sensor', color='white', fontsize=10)
l_do, = ax_do.plot([], [], '#55ff55', linewidth=1.5); ax_do.set_title('DO Sensor', color='white', fontsize=10)

ax_status.axis('off')
status_text = ax_status.text(0.1, 0.5, '', color='white', family='monospace', fontsize=11, verticalalignment='center')

# Buttons
ax_btn1 = fig.add_axes([0.1, 0.03, 0.15, 0.06])
ax_btn2 = fig.add_axes([0.27, 0.03, 0.15, 0.06])
ax_btn3 = fig.add_axes([0.44, 0.03, 0.22, 0.06])
ax_btn4 = fig.add_axes([0.68, 0.03, 0.22, 0.06])

btn_ph = Button(ax_btn1, 'Inject pH Leak', color='#c0392b', hovercolor='#e74c3c')
btn_fault = Button(ax_btn2, 'Inject Fault', color='#d35400', hovercolor='#f39c12')
btn_reset = Button(ax_btn3, 'Reset ML & Normal', color='#2980b9', hovercolor='#3498db')
btn_force = Button(ax_btn4, 'FORCE SURFACE', color='#8e44ad', hovercolor='#9b59b6')

for b in [btn_ph, btn_fault, btn_reset, btn_force]: b.label.set_color('white'); b.label.set_fontsize(9)

def inject_ph(e): global anomaly_active, anomaly_type; anomaly_active=True; anomaly_type='PH'
def inject_fault(e): global anomaly_active, anomaly_type; anomaly_active=True; anomaly_type='FAULT'
def reset_ml(e):
    global anomaly_active, anomaly_type, alert_state, consecutive_anomalies
    anomaly_active = False; anomaly_type = 'NONE'
    alert_state = 0; consecutive_anomalies = 0
    print("[ML] Reset to Normal")
def force_surface(e):
    sock_tx.sendto(json.dumps({'command':'EMERGENCY'}).encode(), (UDP_IP, TX_PORT))
    print("[CMD] Forced Emergency Ascent")

btn_ph.on_clicked(inject_ph); btn_fault.on_clicked(inject_fault)
btn_reset.on_clicked(reset_ml); btn_force.on_clicked(force_surface)

last_ser_time = 0

def update(frame):
    global alert_state, consecutive_anomalies, last_ser_time
    
    # 1. Generate Data
    d = phys_state['depth']
    ph = 8.1 + np.random.normal(0, 0.02)
    ec = 50.0 + np.random.normal(0, 0.1)
    do = 8.0 - 0.5 * d + np.random.normal(0, 0.05)
    
    if anomaly_active:
        if anomaly_type == 'PH': ph -= 2.0; do -= 1.5
        else: ec += 15.0; ph += 0.5
        
    history['t'].append(phys_state['t'])
    history['ph'].append(ph); history['ec'].append(ec); history['do'].append(do)
    
    # 2. ML Logic
    is_anomaly = (ph < 7.0 or ec > 55.0)
    if is_anomaly:
        consecutive_anomalies += 1
        if consecutive_anomalies >= 3 and alert_state == 0: alert_state = 1
        if consecutive_anomalies >= 8 and alert_state == 1: alert_state = 2
        if consecutive_anomalies >= 15 and alert_state == 2:
            alert_state = 3
            sock_tx.sendto(json.dumps({'command':'EMERGENCY'}).encode(), (UDP_IP, TX_PORT))
    else:
        if consecutive_anomalies > 0: consecutive_anomalies -= 1
        if alert_state < 3 and consecutive_anomalies == 0: alert_state = 0

    # 3. Serial
    ser_status = "STANDBY"
    if d < 0.3:
        ser_status = "SENDING..."
        if time.time() - last_ser_time > 1.0:
            msg = f"D:{d:.2f},T:{25-1.5*d:.2f},PH:{ph:.2f},EC:{ec:.3f},O2:{do:.2f},VOLT:8.4\n"
            if ser: ser.write(msg.encode())
            last_ser_time = time.time()

    # 4. Plots
    t_win = list(history['t']); ph_win = list(history['ph'])
    if t_win:
        l_ph.set_data(t_win, ph_win); ax_ph.set_xlim(t_win[0], t_win[-1]); ax_ph.set_ylim(min(ph_win)-0.5, max(ph_win)+0.5)
        l_ec.set_data(t_win, list(history['ec'])); ax_ec.relim(); ax_ec.autoscale_view()
        l_do.set_data(t_win, list(history['do'])); ax_do.relim(); ax_do.autoscale_view()
    
    info = (f"LEVEL : {STATE_NAMES[alert_state]}\nCOUNT : {consecutive_anomalies}\nTYPE  : {anomaly_type}\n\n"
            f"DEPTH : {d:.2f} m\nSTATE : {phys_state['state']}\n\n"
            f"PORT  : {SERIAL_PORT}\nSERIAL: {ser_status}")
    status_text.set_text(info)
    fig.patch.set_facecolor('#300' if alert_state == 3 else '#121212')
    return l_ph, l_ec, l_do

ani = FuncAnimation(fig, update, interval=100, cache_frame_data=False)
plt.show()
