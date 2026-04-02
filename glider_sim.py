"""
KCL Underwater Glider — Real-Time Simulation
=============================================
Live animation showing how buoyancy and mass actuators are manipulated
and the resulting pitch angle, angle of attack, and glider trajectory.

Usage:
    pip install numpy matplotlib
    python glider_sim.py

Controls:
    Deploy  — begin deployment and start sawtooth mission
    Pause   — freeze / resume animation
    Surface — abort mission, extend buoy, rise to surface
    Restart — reset simulation to initial state
    Close window to stop.
"""

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib.animation import FuncAnimation
from matplotlib.gridspec import GridSpec
from matplotlib.widgets import Button

# ==============================================================
# PARAMETERS — tune to match the physical glider
# ==============================================================

RHO_WATER   = 1025.0   # kg/m³  seawater density
G           = 9.81     # m/s²
M_TOTAL     = 10.0     # kg     total glider mass
M_MASS      = 1.0      # kg     movable mass block
V_NEUTRAL   = M_TOTAL / RHO_WATER
DELTA_V     = 0.0005   # m³     max syringe volume change (~500 ml)
L_BODY_HALF = 0.25     # m      mass travels ±this from centre
META_HEIGHT = 0.05     # m      metacentric height (pitch stiffness)
C_L         = 0.8      # lift coefficient
C_D         = 0.12     # drag coefficient

# Actuator speeds — normalised rates (0→1 per second)
# NOTE: BUOY_RATE here is tuned for visual simulation only — at 1/12 the buoyancy
# takes 6s just to pass neutral, meaning the glider stays in negative buoyancy
# for most of the CLIMB leg. 1/3 gives a clean sawtooth visually.
# Hardware spec is unknown — update when measured on the real LD20.
BUOY_RATE   = 1.0 / 8.0   # sim-tuned (real hardware: measure and update)
MASS_RATE   = 1.0 / 8.0  # sim-tuned — fast crossing so pitch rarely sits near zero

# State machine timing (must match motion.c)
DIVE_MS     = 8000    # ms
CLIMB_MS    = 8000    # ms

# Simulation
DT          = 0.02     # s  timestep (20ms = App_Tick)
ANIM_STEPS  = 5        # sim steps per animation frame — lower = slower playback
TRAIL_LEN   = 300      # trajectory points to keep on screen
DEPLOY_DEPTH = 0.3     # m  descend to this depth before starting sawtooth

# ==============================================================
# STATE MACHINE
# ==============================================================
MOT_IDLE    = -1
MOT_DEPLOY  =  0
MOT_DIVE    =  1
MOT_CLIMB   =  2
MOT_SURFACE =  3
STATE_NAMES  = {MOT_IDLE: "READY", MOT_DEPLOY: "DEPLOY", MOT_DIVE: "DIVE",
                MOT_CLIMB: "CLIMB", MOT_SURFACE: "SURFACING"}
STATE_COLORS = {MOT_IDLE: "#888888", MOT_DEPLOY: "#3498db", MOT_DIVE: "#e74c3c",
                MOT_CLIMB: "#2ecc71", MOT_SURFACE: "#f39c12"}

# ==============================================================
# PHYSICS
# ==============================================================

def buoyancy_force(buoy_pos):
    """F_b = (rho * V - m_total) * g"""
    V = V_NEUTRAL + (buoy_pos - 0.5) * 2.0 * DELTA_V
    return (RHO_WATER * V - M_TOTAL) * G

def pitch_from_mass(mass_pos):
    """tau = m_mass * g * x_m * cos(theta)  →  theta ~ arctan(x_m / h)"""
    x_m = mass_pos * L_BODY_HALF
    return np.arctan2(M_MASS * x_m, M_TOTAL * META_HEIGHT)

def angle_of_attack(pitch_rad, glide_angle_rad):
    """
    alpha = theta - gamma
    In steady glide: gamma = arctan(C_D / C_L)
    During transitions alpha deviates from this equilibrium.
    """
    return pitch_rad - glide_angle_rad

# ==============================================================
# SIMULATION STATE
# ==============================================================

sim = {
    't': 0.0,
    'depth': 0.0,
    'x': 0.0,
    'v': 0.0,
    'buoy_pos': 1.0,   # fully extended — glider is floating at surface
    'mass_pos': 0.0,
    'state': MOT_IDLE,
    'state_timer': 0.0,
}

paused = False

history = {
    't': [],
    'depth': [],
    'x': [],
    'pitch': [],
    'aoa': [],
    'buoy': [],
    'mass': [],
    'fb': [],
    'state': [],
}

def sim_step():
    s = sim

    if s['state'] == MOT_IDLE:
        # Waiting for Deploy button — record current state, don't move
        F_b   = buoyancy_force(s['buoy_pos'])
        pitch = pitch_from_mass(s['mass_pos'])
        history['t'].append(s['t'])
        history['depth'].append(s['depth'])
        history['x'].append(s['x'])
        history['pitch'].append(np.degrees(pitch))
        history['aoa'].append(0.0)
        history['buoy'].append(s['buoy_pos'])
        history['mass'].append(s['mass_pos'])
        history['fb'].append(F_b)
        history['state'].append(s['state'])
        return

    if s['state'] == MOT_DEPLOY:
        s['buoy_pos'] = max(0.0, s['buoy_pos'] - BUOY_RATE * DT)
        s['mass_pos'] = 0.0
        if s['depth'] >= DEPLOY_DEPTH:
            s['state'] = MOT_DIVE
            s['state_timer'] = 0.0
    elif s['state'] == MOT_DIVE:
        s['buoy_pos'] = max(0.0, s['buoy_pos'] - BUOY_RATE * DT)
        s['mass_pos'] = min(1.0, s['mass_pos'] + MASS_RATE * DT)
        if s['state_timer'] >= DIVE_MS / 1000.0:
            s['state'] = MOT_CLIMB
            s['state_timer'] = 0.0
    elif s['state'] == MOT_CLIMB:
        s['buoy_pos'] = min(1.0, s['buoy_pos'] + BUOY_RATE * DT)
        s['mass_pos'] = max(-1.0, s['mass_pos'] - MASS_RATE * DT)
        if s['state_timer'] >= CLIMB_MS / 1000.0:
            s['state'] = MOT_DIVE
            s['state_timer'] = 0.0
    elif s['state'] == MOT_SURFACE:
        # Extend buoy fully, centre mass, rise to surface
        s['buoy_pos'] = min(1.0, s['buoy_pos'] + BUOY_RATE * DT)
        if s['mass_pos'] > 0:
            s['mass_pos'] = max(0.0, s['mass_pos'] - MASS_RATE * DT)
        elif s['mass_pos'] < 0:
            s['mass_pos'] = min(0.0, s['mass_pos'] + MASS_RATE * DT)
        if s['depth'] <= 0.0:
            s['v'] = 0.0

    F_b   = buoyancy_force(s['buoy_pos'])
    pitch = pitch_from_mass(s['mass_pos'])

    if s['state'] == MOT_DEPLOY:
        # Descend directly — no pitch yet
        s['v'] += (abs(F_b) / M_TOTAL) * 0.4 * DT
        s['v'] = min(s['v'], 0.5)
        s['depth'] += s['v'] * DT
        vx = 0.0
        glide_angle = 0.0
        aoa = 0.0
    elif s['state'] == MOT_SURFACE:
        # Rise directly based on buoyancy — pitch-independent so mass centering
        # doesn't stall the ascent when pitch → 0
        s['v'] += (abs(F_b) / M_TOTAL) * 0.4 * DT
        s['v'] = min(s['v'], 0.5)
        s['depth'] = max(0.0, s['depth'] - s['v'] * DT)
        vx = 0.0
        glide_angle = 0.0
        aoa = 0.0
        if s['depth'] <= 0.0:
            s['v'] = 0.0
            s['state'] = MOT_IDLE
    else:
        # DIVE goes down, CLIMB goes up — pitch-driven
        dive_dir = 1.0 if s['state'] == MOT_DIVE else -1.0
        vz = s['v'] * np.sin(abs(pitch)) * dive_dir if s['v'] > 0.001 else 0.0
        vx = s['v'] * np.cos(abs(pitch)) if s['v'] > 0.001 else 0.0
        glide_angle = np.arctan2(abs(vz), max(vx, 0.001)) * (-dive_dir)
        aoa = angle_of_attack(pitch, glide_angle)

        s['v'] += (abs(F_b) / M_TOTAL) * 0.4 * DT
        s['v'] = min(s['v'], 0.5)

        s['depth'] = max(0.0, s['depth'] + vz * DT)
        s['x'] += vx * DT

    s['t'] += DT
    s['state_timer'] += DT

    history['t'].append(s['t'])
    history['depth'].append(s['depth'])
    history['x'].append(s['x'])
    history['pitch'].append(np.degrees(pitch))
    history['aoa'].append(np.degrees(aoa))
    history['buoy'].append(s['buoy_pos'])
    history['mass'].append(s['mass_pos'])
    history['fb'].append(F_b)
    history['state'].append(s['state'])

# Pre-run a few steps so plots have initial data
for _ in range(5):
    sim_step()

# ==============================================================
# FIGURE LAYOUT
# ==============================================================

fig = plt.figure(figsize=(14, 8), facecolor='#1a1a2e')
fig.suptitle('KCL Underwater Glider — Real-Time Simulation', color='white',
             fontsize=13, fontweight='bold', y=0.98)

gs = GridSpec(3, 3, figure=fig, hspace=0.45, wspace=0.38,
              left=0.07, right=0.97, top=0.93, bottom=0.14)

ax_traj  = fig.add_subplot(gs[:, 0])    # Trajectory (full left column)
ax_buoy  = fig.add_subplot(gs[0, 1:])   # Buoyancy actuator
ax_mass  = fig.add_subplot(gs[1, 1:])   # Mass actuator
ax_angle = fig.add_subplot(gs[2, 1:])   # Pitch + AoA time series

for ax in [ax_traj, ax_buoy, ax_mass, ax_angle]:
    ax.set_facecolor('#16213e')
    ax.tick_params(colors='#aaaaaa', labelsize=8)
    for spine in ax.spines.values():
        spine.set_edgecolor('#444466')

# --- Trajectory ---
ax_traj.set_title('Trajectory', color='white', fontsize=9)
ax_traj.set_xlabel('Horizontal (m)', color='#aaaaaa', fontsize=8)
ax_traj.set_ylabel('Depth (m,  ↓ = deeper)', color='#aaaaaa', fontsize=8)
ax_traj.axhline(0, color='#4488ff', linewidth=1.2, linestyle='--', label='Surface')
ax_traj.grid(True, alpha=0.15)

line_traj,   = ax_traj.plot([], [], color='#00d4ff', linewidth=1.2, alpha=0.7)
dot_glider,  = ax_traj.plot([], [], 'o', color='white', markersize=7, zorder=5)
state_text    = ax_traj.text(0.05, 0.05, '', transform=ax_traj.transAxes,
                              fontsize=11, fontweight='bold', color='white')

# --- Buoyancy gauge ---
ax_buoy.set_title('Buoyancy Actuator / LD20  (0 = retracted, 1 = extended)', color='white', fontsize=9)
ax_buoy.set_xlim(0, 1)
ax_buoy.set_ylim(-0.5, 0.5)
ax_buoy.set_yticks([])
ax_buoy.axvline(0.5, color='#888888', linewidth=1, linestyle=':', alpha=0.6)
ax_buoy.text(0.5, 0.38, 'neutral', ha='center', color='#888888', fontsize=7,
             transform=ax_buoy.transAxes)
ax_buoy.text(0.02, 0.55, '− buoyancy\n(dive)', color='#e74c3c', fontsize=7,
             transform=ax_buoy.transAxes)
ax_buoy.text(0.72, 0.55, '+ buoyancy\n(surface/climb)', color='#2ecc71', fontsize=7,
             transform=ax_buoy.transAxes)

bar_buoy = ax_buoy.barh(0, 0, height=0.6, color='#3498db', alpha=0.85)[0]
dot_buoy, = ax_buoy.plot([], [], '|', color='white', markersize=18, markeredgewidth=2.5)
val_buoy  = ax_buoy.text(0.98, -0.4, '', ha='right', color='white', fontsize=8)

# --- Mass gauge ---
ax_mass.set_title('Mass Actuator / L16-P  (−1 = toward tail, +1 = toward nose)', color='white', fontsize=9)
ax_mass.set_xlim(-1, 1)
ax_mass.set_ylim(-0.5, 0.5)
ax_mass.set_yticks([])
ax_mass.axvline(0, color='#888888', linewidth=1, linestyle=':', alpha=0.6)
ax_mass.text(0.5, 0.38, 'centred', ha='center', color='#888888', fontsize=7,
             transform=ax_mass.transAxes)
ax_mass.text(0.01, 0.55, 'nose up\n(climb)', color='#2ecc71', fontsize=7,
             transform=ax_mass.transAxes)
ax_mass.text(0.78, 0.55, 'nose down\n(dive)', color='#e74c3c', fontsize=7,
             transform=ax_mass.transAxes)

bar_mass = ax_mass.barh(0, 0, height=0.6, color='#e67e22', alpha=0.85)[0]
dot_mass, = ax_mass.plot([], [], '|', color='white', markersize=18, markeredgewidth=2.5)
val_mass  = ax_mass.text(0.98, -0.4, '', ha='right', color='white', fontsize=8,
                          transform=ax_mass.transAxes)

# --- Pitch + AoA ---
ax_angle.set_title('Pitch angle  θ  and  Angle of Attack  α', color='white', fontsize=9)
ax_angle.set_xlabel('Time (s)', color='#aaaaaa', fontsize=8)
ax_angle.set_ylabel('Degrees (°)', color='#aaaaaa', fontsize=8)
ax_angle.axhline(0, color='#555577', linewidth=0.8)
ax_angle.grid(True, alpha=0.15)

line_pitch, = ax_angle.plot([], [], color='#f39c12', linewidth=1.4, label='Pitch θ')
line_aoa,   = ax_angle.plot([], [], color='#9b59b6', linewidth=1.4, label='AoA α', linestyle='--')
ax_angle.legend(loc='upper right', fontsize=7, facecolor='#1a1a2e', edgecolor='#444466',
                labelcolor='white')

# ==============================================================
# BUTTONS
# ==============================================================

btn_style = dict(color='#16213e', hovercolor='#2a2a5e')

ax_btn_deploy  = fig.add_axes([0.05, 0.03, 0.18, 0.05])
ax_btn_pause   = fig.add_axes([0.27, 0.03, 0.18, 0.05])
ax_btn_surface = fig.add_axes([0.50, 0.03, 0.18, 0.05])
ax_btn_restart = fig.add_axes([0.73, 0.03, 0.18, 0.05])

btn_deploy  = Button(ax_btn_deploy,  'Deploy',  **btn_style)
btn_pause   = Button(ax_btn_pause,   'Pause',   **btn_style)
btn_surface = Button(ax_btn_surface, 'Surface', **btn_style)
btn_restart = Button(ax_btn_restart, 'Restart', **btn_style)

for btn in [btn_deploy, btn_pause, btn_surface, btn_restart]:
    btn.label.set_color('white')
    btn.label.set_fontsize(10)

def on_deploy(event):
    if sim['state'] == MOT_IDLE:
        sim['state'] = MOT_DEPLOY
        sim['state_timer'] = 0.0
        sim['v'] = 0.0

def on_pause(event):
    global paused
    paused = not paused
    btn_pause.label.set_text('Play' if paused else 'Pause')
    fig.canvas.draw_idle()

def on_surface(event):
    if sim['state'] not in (MOT_IDLE, MOT_SURFACE):
        sim['state'] = MOT_SURFACE
        sim['state_timer'] = 0.0
        sim['v'] = 0.0

def on_restart(event):
    global paused
    paused = False
    btn_pause.label.set_text('Pause')
    sim.update({'t': 0.0, 'depth': 0.0, 'x': 0.0, 'v': 0.0,
                'buoy_pos': 1.0, 'mass_pos': 0.0,
                'state': MOT_IDLE, 'state_timer': 0.0})
    for key in history:
        history[key].clear()
    for _ in range(5):
        sim_step()

btn_deploy.on_clicked(on_deploy)
btn_pause.on_clicked(on_pause)
btn_surface.on_clicked(on_surface)
btn_restart.on_clicked(on_restart)

# ==============================================================
# ANIMATION UPDATE
# ==============================================================

def update(frame):
    if not paused:
        for _ in range(ANIM_STEPS):
            sim_step()

    t_arr    = np.array(history['t'])
    d_arr    = np.array(history['depth'])
    x_arr    = np.array(history['x'])
    pitch_arr = np.array(history['pitch'])
    aoa_arr  = np.array(history['aoa'])
    buoy_arr = np.array(history['buoy'])
    mass_arr = np.array(history['mass'])

    # Trim to trail length
    n = min(len(t_arr), TRAIL_LEN)
    t_w    = t_arr[-n:]
    d_w    = d_arr[-n:]
    x_w    = x_arr[-n:]
    p_w    = pitch_arr[-n:]
    a_w    = aoa_arr[-n:]
    bu_w   = buoy_arr[-n:]
    ma_w   = mass_arr[-n:]

    # Trajectory
    line_traj.set_data(x_w, d_w)
    dot_glider.set_data([x_w[-1]], [d_w[-1]])
    ax_traj.set_xlim(max(0, x_w[-1] - 3), x_w[-1] + 0.5)
    ax_traj.set_ylim(max(d_arr) * 1.15 + 0.1, -0.3)

    cur_state = sim['state']
    state_text.set_text(STATE_NAMES[cur_state])
    state_text.set_color(STATE_COLORS[cur_state])

    # Buoyancy bar
    bp = buoy_arr[-1]
    bar_buoy.set_width(bp)
    bar_buoy.set_color('#2ecc71' if bp > 0.5 else '#e74c3c')
    dot_buoy.set_data([bp], [0])
    val_buoy.set_text(f'pos: {bp:.2f}  F_b: {history["fb"][-1]:+.2f} N')

    # Mass bar
    mp = mass_arr[-1]
    bar_mass.set_width(abs(mp))
    bar_mass.set_x(min(mp, 0))
    bar_mass.set_color('#e74c3c' if mp > 0 else '#2ecc71')
    dot_mass.set_data([mp], [0])
    val_mass.set_text(f'pos: {mp:+.2f}')

    # Angles
    line_pitch.set_data(t_w, p_w)
    line_aoa.set_data(t_w, a_w)
    ax_angle.set_xlim(t_w[0], max(t_w[-1], t_w[0] + 5))
    all_angles = np.concatenate([p_w, a_w])
    pad = max(abs(all_angles).max() * 0.3, 2)
    ax_angle.set_ylim(all_angles.min() - pad, all_angles.max() + pad)

    return line_traj, dot_glider, bar_buoy, dot_buoy, bar_mass, dot_mass, \
           line_pitch, line_aoa, state_text, val_buoy, val_mass

# ==============================================================
# RUN
# ==============================================================

ani = FuncAnimation(fig, update, interval=50, blit=False, cache_frame_data=False)
plt.show()
