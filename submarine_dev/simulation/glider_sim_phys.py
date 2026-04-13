"""
KCL Underwater Glider — Physics Engine (Full Dashboard Integration)
==================================================================
- Full UI: Trajectory, Buoyancy, Mass, and Depth Control plots.
- Networking: Broadcasts state to UDP 5005, Listens for Commands on UDP 5006.
- Interactive: Integrated buttons and real-time physics simulation.
"""
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.gridspec import GridSpec
from matplotlib.widgets import Button
import socket
import json
import threading

# ==============================================================
# PARAMETERS & NETWORKING
# ==============================================================
RHO_WATER, G, M_TOTAL, M_MASS = 1025.0, 9.81, 10.0, 1.0
V_NEUTRAL, DELTA_V, L_BODY_HALF, META_HEIGHT = M_TOTAL/1025.0, 0.0005, 0.25, 0.05
BUOY_RATE, MASS_RATE = 1.0/8.0, 1.0/8.0
MIN_DEPTH, MAX_DEPTH, DT = 0.6, 3.0, 0.05
DEPLOY_DEPTH = 0.3

UDP_IP = "127.0.0.1"
TX_PORT, RX_PORT = 5005, 5006
sock_tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock_rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock_rx.bind((UDP_IP, RX_PORT))
sock_rx.setblocking(False)

# ==============================================================
# STATE MACHINE & SIMULATION
# ==============================================================
MOT_IDLE, MOT_DEPLOY, MOT_DIVE, MOT_CLIMB, MOT_SURFACE = -1, 0, 1, 2, 3
sim = {'t':0.0, 'depth':0.0, 'x':0.0, 'v':0.0, 'buoy_pos':1.0, 'mass_pos':0.0, 'state':MOT_IDLE, 'state_timer':0.0}
history = {'t':[], 'depth':[], 'x':[], 'buoy':[], 'mass':[], 'fb':[]}
paused = False

def listen_commands():
    while True:
        try:
            data, _ = sock_rx.recvfrom(1024)
            if json.loads(data.decode()).get('command') == 'EMERGENCY':
                sim['state'] = MOT_SURFACE
                print("\n[PHYSICS] ⚠️ EMERGENCY SIGNAL RECEIVED FROM TINYML!\n")
        except: pass

threading.Thread(target=listen_commands, daemon=True).start()

def sim_step():
    s = sim
    if s['state'] == MOT_IDLE: return
    
    # Actuator Logic
    if s['state'] == MOT_DEPLOY:
        s['buoy_pos'] = max(0.0, s['buoy_pos'] - BUOY_RATE * DT)
        if s['depth'] >= DEPLOY_DEPTH: s['state'] = MOT_DIVE
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
        s['mass_pos'] = 0.0
        if s['depth'] <= 0.0: s['state'] = MOT_IDLE

    # Physics Integration
    F_b = (RHO_WATER * (V_NEUTRAL + (s['buoy_pos']-0.5)*2.0*DELTA_V) - M_TOTAL)*G
    pitch = np.arctan2(M_MASS*(s['mass_pos']*L_BODY_HALF), M_TOTAL*META_HEIGHT)
    s['v'] = min(0.5, s['v'] + (abs(F_b)/M_TOTAL)*0.4*DT)
    
    if s['state'] in [MOT_DEPLOY, MOT_SURFACE]:
        s['depth'] = max(0.0, s['depth'] + (1 if s['state']==MOT_DEPLOY else -1)*s['v']*DT)
    else:
        s['depth'] = max(0.0, s['depth'] + s['v']*np.sin(abs(pitch))*(1 if s['state']==MOT_DIVE else -1)*DT)
        s['x'] += s['v']*np.cos(abs(pitch))*DT
    
    s['t'] += DT
    for k, v in zip(['t','depth','x','buoy','mass','fb'], [s['t'], s['depth'], s['x'], s['buoy_pos'], s['mass_pos'], F_b]):
        history[k].append(v)
    
    # Broadcast to Gateway
    msg = json.dumps({'t':s['t'], 'depth':s['depth'], 'x':s['x'], 'state':s['state']})
    sock_tx.sendto(msg.encode(), (UDP_IP, TX_PORT))

# ==============================================================
# UI FIGURE SETUP
# ==============================================================
fig = plt.figure(figsize=(14, 8), facecolor='#1a1a2e')
fig.suptitle('KCL Underwater Glider — Physics Simulation', color='white', fontsize=12, fontweight='bold')

gs = GridSpec(3, 3, figure=fig, hspace=0.45, wspace=0.38, left=0.07, right=0.97, top=0.93, bottom=0.14)

ax_traj = fig.add_subplot(gs[:, 0])
ax_buoy = fig.add_subplot(gs[0, 1:])
ax_mass = fig.add_subplot(gs[1, 1:])
ax_depth = fig.add_subplot(gs[2, 1:])

for ax in [ax_traj, ax_buoy, ax_mass, ax_depth]:
    ax.set_facecolor('#16213e')
    ax.tick_params(colors='#aaaaaa', labelsize=8)
    for spine in ax.spines.values(): spine.set_edgecolor('#444466')

# --- Elements ---
line_traj, = ax_traj.plot([], [], '#00d4ff', linewidth=1.2, alpha=0.7)
dot_glider, = ax_traj.plot([], [], 'o', color='white', markersize=7, zorder=5)
ax_traj.set_title('Mission Trajectory', color='white', fontsize=9)

bar_buoy = ax_buoy.barh(0, 0, height=0.6, color='#3498db', alpha=0.85)[0]
ax_buoy.set_title('Buoyancy Actuator (0:Dive, 1:Surface)', color='white', fontsize=9)

bar_mass = ax_mass.barh(0, 0, height=0.6, color='#e67e22', alpha=0.85)[0]
ax_mass.set_title('Mass Actuator (-1:Tail, 1:Nose)', color='white', fontsize=9)

line_depth, = ax_depth.plot([], [], '#f1c40f', linewidth=1.6)
ax_depth.set_title('Depth Trend', color='white', fontsize=9)

# --- Buttons ---
ax_btn_deploy = fig.add_axes([0.1, 0.03, 0.15, 0.05])
btn_deploy = Button(ax_btn_deploy, 'Deploy Mission', color='#16213e', hovercolor='#2a2a5e')
btn_deploy.label.set_color('white')

def on_deploy(e):
    if sim['state'] == MOT_IDLE: sim['state'] = MOT_DEPLOY

btn_deploy.on_clicked(on_deploy)

def update(frame):
    if not paused:
        for _ in range(5): sim_step()
    
    if not history['t']: return line_traj, 
    
    t_arr = np.array(history['t']); d_arr = np.array(history['depth']); x_arr = np.array(history['x'])
    tw = t_arr[-300:]; dw = d_arr[-300:]; xw = x_arr[-300:]
    
    line_traj.set_data(xw, dw)
    dot_glider.set_data([xw[-1]], [dw[-1]])
    ax_traj.set_xlim(max(0, xw[-1]-3), xw[-1]+0.5); ax_traj.set_ylim(max(max(d_arr), 3.5), -0.3)
    
    bar_buoy.set_width(sim['buoy_pos']); bar_buoy.set_color('#2ecc71' if sim['buoy_pos']>0.5 else '#e74c3c')
    ax_buoy.set_xlim(0, 1)
    
    mp = sim['mass_pos']; bar_mass.set_width(abs(mp)); bar_mass.set_x(min(mp, 0))
    ax_mass.set_xlim(-1, 1)
    
    line_depth.set_data(tw, dw)
    ax_depth.set_xlim(tw[0], max(tw[-1], tw[0]+5)); ax_depth.set_ylim(max(max(d_arr), 3.5), -0.1)
    
    return line_traj, dot_glider, bar_buoy, bar_mass, line_depth

ani = FuncAnimation(fig, update, interval=50, cache_frame_data=False)
plt.show()
