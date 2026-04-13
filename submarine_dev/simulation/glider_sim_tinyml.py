"""
KCL Underwater Glider — TinyML Integrated Simulation
====================================================
This simulation integrates the Glider Physics with the TinyML logic 
from the sensor_ph.ino firmware. 

Features:
- Real-time simulated sensors: pH, EC, DO, Temperature
- TinyML Feature Extraction (Mean, Std, Slope, RMS)
- State Machine: NORMAL -> WATCHING -> CONFIRMED -> EMERGENCY
- Closed-loop control: ML-triggered emergency surfacing
"""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.gridspec import GridSpec
from matplotlib.widgets import Button
import collections

# ==============================================================
# GLIDER PARAMETERS (from glider_sim_closed_loop.py)
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
DEPLOY_DEPTH = 0.3
DEPTH_HYST  = 0.05
DT          = 0.05     # Sim timestep
ANIM_STEPS  = 4
TRAIL_LEN   = 300
MAX_SPEED   = 0.5
ACCEL_GAIN  = 0.4

# ==============================================================
# TINYML PARAMETERS (from sensor_types.h)
# ==============================================================
WINDOW_SIZE = 64
WINDOW_STRIDE = 16
NUM_CHANNELS = 6  # pH, EC, DO, O2, DEPTH, TEMP

# ML Classes
CLASS_NORMAL = 0
CLASS_MOTION_ARTIFACT = 1
CLASS_SENSOR_FAULT = 2
CLASS_ENV_ANOMALY = 3

# Alert States
STATE_NORMAL = 0
STATE_WATCHING = 1
STATE_CONFIRMED = 2
STATE_EMERGENCY = 3

STATE_NAMES = {
    STATE_NORMAL: "NORMAL",
    STATE_WATCHING: "WATCHING",
    STATE_CONFIRMED: "CONFIRMED",
    STATE_EMERGENCY: "EMERGENCY"
}

# ==============================================================
# SIMULATED SENSORS
# ==============================================================
class GliderSensors:
    def __init__(self):
        self.anomaly_active = False
        self.anomaly_type = None  # 'PH_LEAK' or 'SENSOR_FAULT'
        
    def get_readings(self, depth, time):
        # Base values (Seawater typical)
        ph = 8.1 + 0.05 * np.sin(time * 0.1) + np.random.normal(0, 0.01)
        ec = 50.0 + np.random.normal(0, 0.1) # mS/cm
        temp = 25.0 - 1.5 * depth + np.random.normal(0, 0.05)
        # DO decreases with depth
        do = 8.0 - 0.5 * depth + np.random.normal(0, 0.05)
        
        # Inject Anomaly
        if self.anomaly_active:
            if self.anomaly_type == 'PH_LEAK':
                # Rapid pH drop
                ph -= 1.5 + 0.1 * np.random.random()
                do -= 1.0
            elif self.anomaly_type == 'SENSOR_FAULT':
                # High noise and drift in EC
                ec += 10.0 * np.sin(time * 2.0) + np.random.normal(0, 5.0)
                ph += np.random.normal(0, 0.5)
        
        return {
            'ph': ph,
            'ec': ec,
            'do': do,
            'temp': temp,
            'depth': depth
        }

# ==============================================================
# TINYML PROCESSOR (Python implementation of Arduino logic)
# ==============================================================
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
        
        # Run inference every STRIDE samples
        if len(self.buffer) == WINDOW_SIZE and self.total_samples % WINDOW_STRIDE == 0:
            result = self.run_inference()
            self.update_state_machine(result)
            self.last_ml_result = result
            
    def run_inference(self):
        # Ported Feature Extraction
        ph_data = [s['ph'] for s in self.buffer]
        ec_data = [s['ec'] for s in self.buffer]
        
        ph_mean = np.mean(ph_data)
        ph_std = np.std(ph_data)
        ec_mean = np.mean(ec_data)
        
        # Mock ML Model Logic (in real life this uses the .tflite model)
        if ph_mean < 7.2 or ph_std > 0.2:
            return CLASS_ENV_ANOMALY
        if ec_mean > 55.0 or np.std(ec_data) > 2.0:
            return CLASS_SENSOR_FAULT
        return CLASS_NORMAL

    def update_state_machine(self, result):
        if result == CLASS_NORMAL:
            if self.consecutive_anomalies > 0:
                self.consecutive_anomalies -= 1
            if self.alert_state == STATE_WATCHING and self.consecutive_anomalies == 0:
                self.alert_state = STATE_NORMAL
        else:
            self.consecutive_anomalies += 1
            
            if self.alert_state == STATE_NORMAL:
                if self.consecutive_anomalies >= 3:
                    self.alert_state = STATE_WATCHING
            elif self.alert_state == STATE_WATCHING:
                if self.consecutive_anomalies >= 8:
                    self.alert_state = STATE_CONFIRMED
            elif self.alert_state == STATE_CONFIRMED:
                if self.consecutive_anomalies >= 12:
                    self.alert_state = STATE_EMERGENCY

# ==============================================================
# SIMULATION STATE
# ==============================================================
MOT_IDLE    = -1
MOT_DEPLOY  =  0
MOT_DIVE    =  1
MOT_CLIMB   =  2
MOT_SURFACE =  3

sim = {
    't': 0.0,
    'depth': 0.0,
    'x': 0.0,
    'v': 0.0,
    'buoy_pos': 1.0,
    'mass_pos': 0.0,
    'state': MOT_IDLE,
    'state_timer': 0.0,
}

sensors = GliderSensors()
ml_proc = TinyMLProcessor()
paused = False

history = collections.defaultdict(list)

def buoyancy_force(buoy_pos):
    V = V_NEUTRAL + (buoy_pos - 0.5) * 2.0 * DELTA_V
    return (RHO_WATER * V - M_TOTAL) * G

def pitch_from_mass(mass_pos):
    x_m = mass_pos * L_BODY_HALF
    return np.arctan2(M_MASS * x_m, M_TOTAL * META_HEIGHT)

def set_state(new_state):
    sim['state'] = new_state
    sim['state_timer'] = 0.0

def sim_step():
    s = sim
    
    # 1. Update Sensors & TinyML
    readings = sensors.get_readings(s['depth'], s['t'])
    ml_proc.process_sample(readings)
    
    # TinyML override: if Emergency detected, force Surface
    if ml_proc.alert_state == STATE_EMERGENCY and s['state'] != MOT_SURFACE:
        set_state(MOT_SURFACE)

    if s['state'] == MOT_IDLE:
        pass
    elif s['state'] == MOT_DEPLOY:
        s['buoy_pos'] = max(0.0, s['buoy_pos'] - BUOY_RATE * DT)
        if s['depth'] >= DEPLOY_DEPTH: set_state(MOT_DIVE)
    elif s['state'] == MOT_DIVE:
        s['buoy_pos'] = max(0.0, s['buoy_pos'] - BUOY_RATE * DT)
        s['mass_pos'] = min(1.0, s['mass_pos'] + MASS_RATE * DT)
        if s['depth'] >= MAX_DEPTH: set_state(MOT_CLIMB)
    elif s['state'] == MOT_CLIMB:
        s['buoy_pos'] = min(1.0, s['buoy_pos'] + BUOY_RATE * DT)
        s['mass_pos'] = max(-1.0, s['mass_pos'] - MASS_RATE * DT)
        if s['depth'] <= MIN_DEPTH: set_state(MOT_DIVE)
    elif s['state'] == MOT_SURFACE:
        s['buoy_pos'] = min(1.0, s['buoy_pos'] + BUOY_RATE * DT)
        s['mass_pos'] = 0.0
        if s['depth'] <= 0.0: set_state(MOT_IDLE)

    # Physics Integration
    F_b = buoyancy_force(s['buoy_pos'])
    pitch = pitch_from_mass(s['mass_pos'])

    if s['state'] in [MOT_DEPLOY, MOT_SURFACE]:
        s['v'] += (abs(F_b) / M_TOTAL) * ACCEL_GAIN * DT
        s['v'] = min(s['v'], MAX_SPEED)
        dir = 1.0 if s['state'] == MOT_DEPLOY else -1.0
        s['depth'] = max(0.0, s['depth'] + dir * s['v'] * DT)
    else:
        dive_dir = 1.0 if s['state'] == MOT_DIVE else -1.0
        vz = s['v'] * np.sin(abs(pitch)) * dive_dir
        vx = s['v'] * np.cos(abs(pitch))
        s['v'] += (abs(F_b) / M_TOTAL) * ACCEL_GAIN * DT
        s['v'] = min(s['v'], MAX_SPEED)
        s['depth'] = max(0.0, s['depth'] + vz * DT)
        s['x'] += vx * DT

    s['t'] += DT
    s['state_timer'] += DT
    
    # Save History
    history['t'].append(s['t'])
    history['depth'].append(s['depth'])
    history['x'].append(s['x'])
    history['ph'].append(readings['ph'])
    history['do'].append(readings['do'])
    history['ec'].append(readings['ec'])
    history['alert'].append(ml_proc.alert_state)

# ==============================================================
# UI SETUP
# ==============================================================
fig = plt.figure(figsize=(15, 9), facecolor='#1a1a2e')
fig.suptitle('AUV TinyML Integrated Simulation (Closed-Loop)', color='white', fontsize=14, fontweight='bold')

gs = GridSpec(4, 3, figure=fig, hspace=0.5, wspace=0.3)

ax_traj  = fig.add_subplot(gs[0:2, 0])
ax_ph    = fig.add_subplot(gs[0, 1])
ax_do    = fig.add_subplot(gs[1, 1])
ax_ec    = fig.add_subplot(gs[2, 1])
ax_alert = fig.add_subplot(gs[2, 0])
ax_ml    = fig.add_subplot(gs[0:2, 2])

for ax in [ax_traj, ax_ph, ax_do, ax_ec, ax_alert, ax_ml]:
    ax.set_facecolor('#16213e')
    ax.tick_params(colors='#aaaaaa', labelsize=8)
    for spine in ax.spines.values(): spine.set_edgecolor('#444466')

# --- Trajectory ---
ax_traj.set_title('Mission Trajectory', color='white', fontsize=10)
line_traj, = ax_traj.plot([], [], color='#00d4ff', linewidth=1.5)
dot_glider, = ax_traj.plot([], [], 'o', color='white', markersize=8)
state_text = ax_traj.text(0.05, 0.05, '', transform=ax_traj.transAxes, fontsize=12, fontweight='bold')

# --- Sensor Plots ---
ax_ph.set_title('pH Sensor', color='white', fontsize=9)
line_ph, = ax_ph.plot([], [], color='#ff5555')
ax_do.set_title('DO (Dissolved Oxygen)', color='white', fontsize=9)
line_do, = ax_do.plot([], [], color='#55ff55')
ax_ec.set_title('EC (Conductivity)', color='white', fontsize=9)
line_ec, = ax_ec.plot([], [], color='#5555ff')

# --- TinyML Status ---
ax_alert.set_title('TinyML Alert State', color='white', fontsize=10)
line_alert, = ax_alert.plot([], [], color='yellow', drawstyle='steps-post')
ax_alert.set_yticks([0, 1, 2, 3])
ax_alert.set_yticklabels(['NORM', 'WATCH', 'CONF', 'EMRG'])

# --- ML Result Info ---
ax_ml.axis('off')
ml_info_text = ax_ml.text(0.1, 0.8, '', color='white', fontsize=11, family='monospace')

# --- Buttons ---
ax_btn_deploy = fig.add_axes([0.1, 0.05, 0.1, 0.04])
ax_btn_ph     = fig.add_axes([0.25, 0.05, 0.15, 0.04])
ax_btn_fault  = fig.add_axes([0.45, 0.05, 0.15, 0.04])
ax_btn_reset  = fig.add_axes([0.65, 0.05, 0.1, 0.04])

btn_deploy = Button(ax_btn_deploy, 'Deploy Mission', color='#2ecc71', hovercolor='#27ae60')
btn_ph     = Button(ax_btn_ph, 'Inject pH Leak', color='#e74c3c', hovercolor='#c0392b')
btn_fault  = Button(ax_btn_fault, 'Inject Sensor Fault', color='#f39c12', hovercolor='#d35400')
btn_reset  = Button(ax_btn_reset, 'Reset System', color='#3498db', hovercolor='#2980b9')

def on_deploy(event): 
    if sim['state'] == MOT_IDLE: set_state(MOT_DEPLOY)
def on_ph(event):
    sensors.anomaly_active = True
    sensors.anomaly_type = 'PH_LEAK'
def on_fault(event):
    sensors.anomaly_active = True
    sensors.anomaly_type = 'SENSOR_FAULT'
def on_reset(event):
    sensors.anomaly_active = False
    ml_proc.__init__()
    if sim['state'] == MOT_IDLE: 
        sim.update({'t': 0.0, 'depth': 0.0, 'x': 0.0, 'v': 0.0, 'state': MOT_IDLE})
        for k in history: history[k].clear()

btn_deploy.on_clicked(on_deploy)
btn_ph.on_clicked(on_ph)
btn_fault.on_clicked(on_fault)
btn_reset.on_clicked(on_reset)

# ==============================================================
# ANIMATION
# ==============================================================
def update(frame):
    if not paused:
        for _ in range(ANIM_STEPS): sim_step()

    # Trajectory
    line_traj.set_data(history['x'], history['depth'])
    if history['x']: dot_glider.set_data([history['x'][-1]], [history['depth'][-1]])
    ax_traj.set_xlim(0, max(10, max(history['x']) + 2) if history['x'] else 10)
    ax_traj.set_ylim(6, -0.5)
    
    # State Text
    st = sim['state']
    st_names = {MOT_IDLE: "READY", MOT_DEPLOY: "DEPLOY", MOT_DIVE: "DIVE", MOT_CLIMB: "CLIMB", MOT_SURFACE: "SURFACING"}
    state_text.set_text(st_names[st])
    state_text.set_color('#2ecc71' if st in [MOT_DIVE, MOT_CLIMB] else 'white')

    # Sensors
    t_window = history['t'][-100:]
    line_ph.set_data(t_window, history['ph'][-100:])
    line_do.set_data(t_window, history['do'][-100:])
    line_ec.set_data(t_window, history['ec'][-100:])
    for ax, key in zip([ax_ph, ax_do, ax_ec], ['ph', 'do', 'ec']):
        if history[key]:
            ax.set_xlim(t_window[0], t_window[-1])
            ax.set_ylim(min(history[key][-100:])*0.9, max(history[key][-100:])*1.1)

    # Alert State
    line_alert.set_data(history['t'], history['alert'])
    ax_alert.set_xlim(history['t'][0], history['t'][-1])

    # ML Info Text
    info = (
        f"--- TINYML ENGINE ---\n"
        f"Status: {STATE_NAMES[ml_proc.alert_state]}\n"
        f"Cons. Anomalies: {ml_proc.consecutive_anomalies}\n"
        f"Last ML Class: {['NORMAL', 'MOTION', 'FAULT', 'ANOMALY'][ml_proc.last_ml_result]}\n\n"
        f"Samples: {ml_proc.total_samples}\n"
        f"Window: {len(ml_proc.buffer)}/64\n"
        f"--- PHYSICAL ---\n"
        f"Depth: {sim['depth']:.2f} m\n"
        f"Pitch: {np.degrees(pitch_from_mass(sim['mass_pos'])):.1f} deg"
    )
    ml_info_text.set_text(info)
    
    # Emergency Color Flash
    if ml_proc.alert_state == STATE_EMERGENCY:
        ax_ml.set_facecolor('#441111')
    else:
        ax_ml.set_facecolor('#16213e')

    return line_traj, dot_glider, line_ph, line_do, line_ec, line_alert

ani = FuncAnimation(fig, update, interval=50, blit=False)
plt.show()
