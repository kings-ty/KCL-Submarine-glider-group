"""
KCL Underwater Glider — TinyML & Physics Integrated Dashboard (V2)
==================================================================
This dashboard combines the full Physical Actuator Simulation 
with the TinyML Anomaly Detection engine and Serial Communication.

Key Features:
- Physics: Buoyancy, Mass shift, Depth control logic.
- TinyML: 64-sample sliding window, Anomaly Escalation (NORM-WATCH-CONF-EMRG).
- Dashboard: All 10 plots integrated into one window.
- Serial Output: Sends sensor data to an external ESP32 when surfaced.

Usage: 
  python simulation/glider_sim_tinyml_v2.py [PORT]
  (e.g., python simulation/glider_sim_tinyml_v2.py /dev/ttyUSB0)
"""

import sys
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.gridspec import GridSpec
from matplotlib.widgets import Button
import collections
import serial
import time

# ==============================================================
# PARAMETERS — Physical & TinyML
# ==============================================================
RHO_WATER   = 1025.0
G           = 9.81
M_TOTAL     = 10.0
M_MASS      = 1.0
V_NEUTRAL   = M_TOTAL / RHO_WATER
DELTA_V     = 0.0005
L_BODY_HALF = 0.25
META_HEIGHT = 0.05
BUOY_RATE   = 1.0 / 8.0
MASS_RATE   = 1.0 / 8.0
MIN_DEPTH   = 0.6
MAX_DEPTH   = 4.0
SURFACE_TH  = 0.3    # Surfacing threshold for Serial Send

# TinyML Constants
WINDOW_SIZE = 64
WINDOW_STRIDE = 16
STATE_NORMAL, STATE_WATCHING, STATE_CONFIRMED, STATE_EMERGENCY = 0, 1, 2, 3
CLASS_NORMAL, CLASS_MOTION, CLASS_FAULT, CLASS_ANOMALY = 0, 1, 2, 3
STATE_NAMES = ["NORMAL", "WATCHING", "CONFIRMED", "EMERGENCY"]

# Serial Config
SERIAL_PORT = sys.argv[1] if len(sys.argv) > 1 else None
ser = None
if SERIAL_PORT:
    try:
        ser = serial.Serial(SERIAL_PORT, 115200, timeout=0.1)
        print(f"[SERIAL] Connected to {SERIAL_PORT}")
    except Exception as e:
        print(f"[SERIAL] Error: {e}")

# ==============================================================
# SIMULATION CLASSES
# ==============================================================

class GliderSensors:
    def __init__(self):
        self.anomaly_active = False
        self.anomaly_type = None 
        
    def get_readings(self, depth, t):
        ph = 8.1 + 0.05 * np.sin(t * 0.1) + np.random.normal(0, 0.01)
        ec = 50.0 + np.random.normal(0, 0.1)
        temp = 25.0 - 1.5 * depth + np.random.normal(0, 0.05)
        do = 8.0 - 0.5 * depth + np.random.normal(0, 0.05)
        
        if self.anomaly_active:
            if self.anomaly_type == 'PH_LEAK':
                ph -= 1.8 + 0.1 * np.random.random()
                do -= 1.5
            elif self.anomaly_type == 'SENSOR_FAULT':
                ec += 15.0 * np.sin(t * 2.0)
                ph += np.random.normal(0, 0.5)
        
        return {'ph': ph, 'ec': ec, 'do': do, 'temp': temp, 'depth': depth, 'volt': 8.4}

class TinyMLProcessor:
    def __init__(self):
        self.buffer = collections.deque(maxlen=WINDOW_SIZE)
        self.total_samples = 0
        self.alert_state = STATE_NORMAL
        self.consecutive_anomalies = 0
        self.last_ml_result = CLASS_NORMAL
        
    def process_sample(self, readings):
        self.buffer.append(readings)
        self.total_samples += 1
        if len(self.buffer) == WINDOW_SIZE and self.total_samples % WINDOW_STRIDE == 0:
            result = self.run_inference()
            self.update_state_machine(result)
            self.last_ml_result = result
            
    def run_inference(self):
        ph_vals = [s['ph'] for s in self.buffer]
        ec_vals = [s['ec'] for s in self.buffer]
        if np.mean(ph_vals) < 7.0 or np.std(ph_vals) > 0.3: return CLASS_ANOMALY
        if np.mean(ec_vals) > 58.0 or np.std(ec_vals) > 3.0: return CLASS_FAULT
        return CLASS_NORMAL

    def update_state_machine(self, result):
        if result == CLASS_NORMAL:
            if self.consecutive_anomalies > 0: self.consecutive_anomalies -= 1
            if self.alert_state == STATE_WATCHING and self.consecutive_anomalies == 0:
                self.alert_state = STATE_NORMAL
        else:
            self.consecutive_anomalies += 1
            if self.alert_state == STATE_NORMAL and self.consecutive_anomalies >= 3: self.alert_state = STATE_WATCHING
            elif self.alert_state == STATE_WATCHING and self.consecutive_anomalies >= 8: self.alert_state = STATE_CONFIRMED
            elif self.alert_state == STATE_CONFIRMED and self.consecutive_anomalies >= 15: self.alert_state = STATE_EMERGENCY

# ==============================================================
# SIMULATION CORE
# ==============================================================
MOT_IDLE, MOT_DEPLOY, MOT_DIVE, MOT_CLIMB, MOT_SURFACE = -1, 0, 1, 2, 3
sim = {'t': 0.0, 'depth': 0.0, 'x': 0.0, 'v': 0.0, 'buoy_pos': 1.0, 'mass_pos': 0.0, 'state': MOT_IDLE}
sensors = GliderSensors()
ml_proc = TinyMLProcessor()
history = collections.defaultdict(list)
DT = 0.05

def sim_step():
    s = sim
    readings = sensors.get_readings(s['depth'], s['t'])
    ml_proc.process_sample(readings)
    
    # TinyML Emergency Trigger
    if ml_proc.alert_state == STATE_EMERGENCY and s['state'] != MOT_SURFACE:
        s['state'] = MOT_SURFACE

    # Serial Send logic (Surfaced)
    if s['depth'] < SURFACE_TH and int(s['t']*20) % 20 == 0 and ser:
        msg = f"D:{s['depth']:.2f},T:{readings['temp']:.2f},PH:{readings['ph']:.2f},EC:{readings['ec']:.3f},O2:{readings['do']:.2f},VOLT:8.4\n"
        ser.write(msg.encode())

    # Physics State Machine
    F_b = (RHO_WATER * (V_NEUTRAL + (s['buoy_pos'] - 0.5) * 2.0 * DELTA_V) - M_TOTAL) * G
    pitch = np.arctan2(M_MASS * (s['mass_pos'] * L_BODY_HALF), M_TOTAL * META_HEIGHT)

    if s['state'] == MOT_IDLE: pass
    elif s['state'] == MOT_DEPLOY:
        s['buoy_pos'] = max(0.0, s['buoy_pos'] - BUOY_RATE * DT)
        if s['depth'] >= 0.3: s['state'] = MOT_DIVE
    elif s['state'] == MOT_DIVE:
        s['buoy_pos'] = max(0.0, s['buoy_pos'] - BUOY_RATE * DT)
        s['mass_pos'] = min(1.0, s['mass_pos'] + MASS_RATE * DT)
        if s['depth'] >= MAX_DEPTH: s['state'] = MOT_CLIMB
    elif s['state'] == MOT_CLIMB:
        s['buoy_pos'] = min(1.0, s['buoy_pos'] + BUOY_RATE * DT)
        s['mass_pos'] = max(-1.0, s['mass_pos'] - MASS_RATE * DT)
        if s['depth'] <= MIN_DEPTH: s['state'] = MOT_DIVE
    elif s['state'] == MOT_SURFACE:
        s['buoy_pos'] = min(1.0, s['buoy_pos'] + BUOY_RATE * DT)
        s['mass_pos'] = max(-0.1, min(0.1, s['mass_pos']))
        if s['depth'] <= 0.0: s['state'] = MOT_IDLE

    # Movement
    s['v'] = min(MAX_SPEED := 0.5, s['v'] + (abs(F_b)/M_TOTAL)*0.4*DT)
    if s['state'] in [MOT_DEPLOY, MOT_SURFACE]:
        s['depth'] = max(0.0, s['depth'] + (1 if s['state']==MOT_DEPLOY else -1) * s['v'] * DT)
    else:
        s['depth'] = max(0.0, s['depth'] + s['v'] * np.sin(abs(pitch)) * (1 if s['state']==MOT_DIVE else -1) * DT)
        s['x'] += s['v'] * np.cos(abs(pitch)) * DT

    s['t'] += DT
    for k, v in zip(['t','depth','x','ph','do','ec','alert','buoy','mass','fb'], 
                    [s['t'], s['depth'], s['x'], readings['ph'], readings['do'], readings['ec'], ml_proc.alert_state, s['buoy_pos'], s['mass_pos'], F_b]):
        history[k].append(v)

# ==============================================================
# UI DASHBOARD
# ==============================================================
fig = plt.figure(figsize=(16, 10), facecolor='#121212')
gs = GridSpec(6, 4, figure=fig, hspace=0.6, wspace=0.4, top=0.92, bottom=0.08)

# Panels
ax_traj  = fig.add_subplot(gs[0:3, 0])
ax_ph    = fig.add_subplot(gs[0, 1])
ax_do    = fig.add_subplot(gs[1, 1])
ax_ec    = fig.add_subplot(gs[2, 1])
ax_alert = fig.add_subplot(gs[3, 1])
ax_buoy  = fig.add_subplot(gs[0, 2:])
ax_mass  = fig.add_subplot(gs[1, 2:])
ax_depth = fig.add_subplot(gs[2, 2:])
ax_ml    = fig.add_subplot(gs[3:6, 2:])

for ax in [ax_traj, ax_ph, ax_do, ax_ec, ax_alert, ax_buoy, ax_mass, ax_depth, ax_ml]:
    ax.set_facecolor('#1e1e1e')
    ax.tick_params(colors='gray', labelsize=8)
    for spine in ax.spines.values(): spine.set_edgecolor('#333')

# Trajectory Setup
ax_traj.set_title('MISSION TRAJECTORY', color='white', fontsize=10)
line_traj, = ax_traj.plot([], [], color='#00d4ff', linewidth=1.5)
dot_glider, = ax_traj.plot([], [], 'o', color='white')
st_text = ax_traj.text(0.05, 0.05, '', transform=ax_traj.transAxes, color='cyan', weight='bold')

# Actuator Setup
bar_buoy = ax_buoy.barh(0, 0, height=0.6, color='#2ecc71')[0]
bar_mass = ax_mass.barh(0, 0, height=0.6, color='#e67e22')[0]
ax_buoy.set_title('BUOYANCY ACTUATOR (0:DIVE, 1:SURFACE)', color='white', fontsize=9)
ax_mass.set_title('MASS ACTUATOR (-1:TAIL, 1:NOSE)', color='white', fontsize=9)
ax_buoy.set_xlim(0, 1); ax_mass.set_xlim(-1, 1)

# Sensor Setup
l_ph, = ax_ph.plot([], [], '#ff5555'); ax_ph.set_title('pH', color='white', fontsize=9)
l_do, = ax_do.plot([], [], '#55ff55'); ax_do.set_title('DO', color='white', fontsize=9)
l_ec, = ax_ec.plot([], [], '#5555ff'); ax_ec.set_title('EC', color='white', fontsize=9)

# TinyML Setup
l_alert, = ax_alert.plot([], [], 'yellow', drawstyle='steps-post')
ax_alert.set_yticks([0,1,2,3]); ax_alert.set_yticklabels(['NORM','WATCH','CONF','EMRG'])
ax_ml.axis('off'); ml_text = ax_ml.text(0.1, 0.9, '', color='white', family='monospace', fontsize=10)

# Buttons
ax_btn_deploy = fig.add_axes([0.05, 0.03, 0.08, 0.03])
ax_btn_ph     = fig.add_axes([0.15, 0.03, 0.12, 0.03])
ax_btn_fault  = fig.add_axes([0.28, 0.03, 0.12, 0.03])
btn_deploy = Button(ax_btn_deploy, 'Deploy', color='#2ecc71')
btn_ph     = Button(ax_btn_ph, 'Inject pH Leak', color='#e74c3c')
btn_fault  = Button(ax_btn_fault, 'Sensor Fault', color='#f39c12')

def on_deploy(e): sim['state'] = MOT_DEPLOY
def on_ph(e): sensors.anomaly_active = True; sensors.anomaly_type = 'PH_LEAK'
def on_fault(e): sensors.anomaly_active = True; sensors.anomaly_type = 'SENSOR_FAULT'
btn_deploy.on_clicked(on_deploy); btn_ph.on_clicked(on_ph); btn_fault.on_clicked(on_fault)

def update(frame):
    for _ in range(4): sim_step()
    line_traj.set_data(history['x'], history['depth'])
    if history['x']: dot_glider.set_data([history['x'][-1]], [history['depth'][-1]])
    ax_traj.set_xlim(0, max(10, history['x'][-1]+2) if history['x'] else 10); ax_traj.set_ylim(6, -0.5)
    
    st_names = {MOT_IDLE:"IDLE", MOT_DEPLOY:"DEPLOY", MOT_DIVE:"DIVE", MOT_CLIMB:"CLIMB", MOT_SURFACE:"SURFACE"}
    st_text.set_text(st_names[sim['state']])
    
    # Actuators
    bar_buoy.set_width(sim['buoy_pos']); bar_buoy.set_color('#2ecc71' if sim['buoy_pos']>0.5 else '#e74c3c')
    mp = sim['mass_pos']; bar_mass.set_width(abs(mp)); bar_mass.set_x(min(mp, 0))
    
    # Sensors (Window 10s)
    tw = history['t'][-200:]
    l_ph.set_data(tw, history['ph'][-200:]); ax_ph.set_xlim(tw[0], tw[-1]); ax_ph.set_ylim(min(history['ph'][-200:])*0.9, max(history['ph'][-200:])*1.1)
    l_do.set_data(tw, history['do'][-200:]); ax_do.set_xlim(tw[0], tw[-1]); ax_do.set_ylim(min(history['do'][-200:])*0.9, max(history['do'][-200:])*1.1)
    l_ec.set_data(tw, history['ec'][-200:]); ax_ec.set_xlim(tw[0], tw[-1]); ax_ec.set_ylim(min(history['ec'][-200:])*0.9, max(history['ec'][-200:])*1.1)
    
    l_alert.set_data(history['t'], history['alert']); ax_alert.set_xlim(history['t'][0], history['t'][-1])

    info = (
        f"--- TINYML ENGINE DASHBOARD ---\n"
        f"Alert State: {STATE_NAMES[ml_proc.alert_state]}\n"
        f"Consecutive Anomalies: {ml_proc.consecutive_anomalies}\n"
        f"Last ML Classification: {['NORMAL','MOTION','FAULT','ANOMALY'][ml_proc.last_ml_result]}\n\n"
        f"--- PHYSICS ENGINE ---\n"
        f"Depth: {sim['depth']:.2f} m\n"
        f"Vertical Velocity: {sim['v']:.2f} m/s\n"
        f"Buoyancy Force: {history['fb'][-1]:+.2f} N\n\n"
        f"--- EXTERNAL COMM ---\n"
        f"Serial Port: {SERIAL_PORT if SERIAL_PORT else 'NONE'}\n"
        f"Status: {'SENDING...' if sim['depth'] < SURFACE_TH else 'STANDBY'}"
    )
    ml_text.set_text(info)
    ax_ml.set_facecolor('#411' if ml_proc.alert_state == STATE_EMERGENCY else '#1e1e1e')
    return line_traj, dot_glider, bar_buoy, bar_mass, l_ph, l_do, l_ec, l_alert

ani = FuncAnimation(fig, update, interval=50, blit=False)
plt.show()
