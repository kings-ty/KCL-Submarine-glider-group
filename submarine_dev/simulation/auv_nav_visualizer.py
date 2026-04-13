"""
============================================================================
 AUV Navigation Visualizer — Real-time IMU Data Display
============================================================================

 STM32 시리얼 출력 기반 항법 시각화
 실제 데이터 형식: Roll: -1.85° Pitch: 26.47° Heading: 152.73° GPIO: ...

 사용법:
   python auv_nav_visualizer.py              (더미 데이터로 데모)
   python auv_nav_visualizer.py --port COM5  (실제 STM32 연결)

============================================================================
"""

import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.gridspec import GridSpec
import numpy as np
import time
import argparse
import math
import sys
from collections import deque

# ============================================================
# 실제 캡처된 데이터 기반 더미 생성
# ============================================================
# 이미지에서 읽은 실제 STM32 출력값
REAL_CAPTURED_DATA = [
    {"roll": -1.85, "pitch": 26.47, "heading": 152.73, "pa4": 1, "pa5": 0, "pa6": 0, "pa7_pwm": 1, "pa8_pwm": 0},
    {"roll": -1.99, "pitch": 26.44, "heading": 162.54, "pa4": 1, "pa5": 0, "pa6": 0, "pa7_pwm": 0, "pa8_pwm": 0},
    {"roll": -2.64, "pitch": 26.83, "heading": 169.68, "pa4": 1, "pa5": 0, "pa6": 0, "pa7_pwm": 1, "pa8_pwm": 0},
    {"roll": -18.85, "pitch": 38.53, "heading": 216.64, "pa4": 1, "pa5": 0, "pa6": 0, "pa7_pwm": 0, "pa8_pwm": 0},
    {"roll": -21.31, "pitch": 20.73, "heading": 268.33, "pa4": 1, "pa5": 0, "pa6": 0, "pa7_pwm": 1, "pa8_pwm": 0},
    {"roll": -16.36, "pitch": 18.05, "heading": 302.65, "pa4": 1, "pa5": 0, "pa6": 0, "pa7_pwm": 0, "pa8_pwm": 0},
    {"roll": 56.40, "pitch": 12.14, "heading": 14.23, "pa4": 1, "pa5": 0, "pa6": 0, "pa7_pwm": 1, "pa8_pwm": 0},
    {"roll": 74.98, "pitch": -3.38, "heading": 43.13, "pa4": 1, "pa5": 0, "pa6": 1, "pa7_pwm": 0, "pa8_pwm": 1},
    {"roll": -9.80, "pitch": 7.41, "heading": 297.75, "pa4": 0, "pa5": 0, "pa6": 0, "pa7_pwm": 0, "pa8_pwm": 0},
    {"roll": -50.03, "pitch": -5.55, "heading": 26.76, "pa4": 0, "pa5": 0, "pa6": 0, "pa7_pwm": 0, "pa8_pwm": 1},
    {"roll": -85.21, "pitch": -2.29, "heading": 49.06, "pa4": 1, "pa5": 0, "pa6": 0, "pa7_pwm": 0, "pa8_pwm": 0},
    {"roll": -59.69, "pitch": -5.75, "heading": 56.32, "pa4": 1, "pa5": 0, "pa6": 1, "pa7_pwm": 0, "pa8_pwm": 0},
    {"roll": -34.77, "pitch": -65.30, "heading": 213.51, "pa4": 1, "pa5": 0, "pa6": 1, "pa7_pwm": 1, "pa8_pwm": 0},
    {"roll": 43.54, "pitch": -84.01, "heading": 80.01, "pa4": 1, "pa5": 0, "pa6": 1, "pa7_pwm": 0, "pa8_pwm": 0},
    {"roll": 14.26, "pitch": -24.45, "heading": 86.09, "pa4": 1, "pa5": 0, "pa6": 1, "pa7_pwm": 1, "pa8_pwm": 1},
    {"roll": -4.18, "pitch": 44.96, "heading": 154.37, "pa4": 0, "pa5": 0, "pa6": 1, "pa7_pwm": 0, "pa8_pwm": 0},
    {"roll": -4.25, "pitch": 60.69, "heading": 35.87, "pa4": 0, "pa5": 0, "pa6": 0, "pa7_pwm": 0, "pa8_pwm": 1},
    {"roll": -3.28, "pitch": 60.97, "heading": 45.86, "pa4": 1, "pa5": 0, "pa6": 0, "pa7_pwm": 0, "pa8_pwm": 0},
    {"roll": -3.09, "pitch": 42.80, "heading": 43.17, "pa4": 1, "pa5": 0, "pa6": 0, "pa7_pwm": 1, "pa8_pwm": 0},
    {"roll": -4.55, "pitch": 25.68, "heading": 51.99, "pa4": 1, "pa5": 0, "pa6": 0, "pa7_pwm": 0, "pa8_pwm": 0},
]


def generate_mission_data(n_points=300):
    """실제 캡처 데이터를 기반으로 현실적인 미션 시뮬레이션 생성"""
    data = []
    t = 0
    
    # Phase 1: Surface (안정)
    for i in range(60):
        data.append({
            "t": t, "phase": "SURFACE",
            "roll": np.random.normal(-2, 1.5),
            "pitch": np.random.normal(2, 2),
            "heading": 152 + np.random.normal(0, 3),
            "depth": 0.1 + np.random.normal(0, 0.05),
            "motor": "IDLE"
        })
        t += 0.5
    
    # Phase 2: Diving (하강 — pitch 증가, depth 증가)
    for i in range(60):
        progress = i / 60
        data.append({
            "t": t, "phase": "DIVING",
            "roll": np.random.normal(-5, 8) + 10 * np.sin(progress * np.pi),
            "pitch": 25 + 35 * progress + np.random.normal(0, 3),
            "heading": 152 + 60 * progress + np.random.normal(0, 5),
            "depth": 0.1 + 8 * progress + np.random.normal(0, 0.2),
            "motor": "MASS_SHIFT" if i % 3 == 0 else "BUOYANCY"
        })
        t += 0.5
    
    # Phase 3: Gliding (수중 글라이딩 — 안정적)
    for i in range(80):
        data.append({
            "t": t, "phase": "GLIDING",
            "roll": np.random.normal(-3, 2),
            "pitch": np.random.normal(5, 3),
            "heading": 210 + np.random.normal(0, 4),
            "depth": 8 + np.random.normal(0, 0.3),
            "motor": "IDLE"
        })
        t += 0.5
    
    # Phase 4: Emergency Ascent (비상 상승)
    for i in range(40):
        progress = i / 40
        data.append({
            "t": t, "phase": "EMERGENCY",
            "roll": np.random.normal(0, 15),
            "pitch": -30 - 20 * progress + np.random.normal(0, 5),
            "heading": 210 + np.random.normal(0, 10),
            "depth": 8 * (1 - progress) + np.random.normal(0, 0.3),
            "motor": "EMERGENCY_ASCENT"
        })
        t += 0.5
    
    # Phase 5: Surface recovery
    for i in range(60):
        data.append({
            "t": t, "phase": "SURFACE",
            "roll": np.random.normal(-1, 2),
            "pitch": np.random.normal(0, 2),
            "heading": 210 + np.random.normal(0, 5),
            "depth": 0.1 + np.random.normal(0, 0.05),
            "motor": "IDLE"
        })
        t += 0.5
    
    return data


def draw_attitude_indicator(ax, roll_deg, pitch_deg):
    """Artificial horizon (자세 지시계)"""
    ax.clear()
    ax.set_xlim(-1.2, 1.2)
    ax.set_ylim(-1.2, 1.2)
    ax.set_aspect('equal')
    ax.axis('off')
    
    # Clip circle
    circle = plt.Circle((0, 0), 1, fill=False, color='#30363d', linewidth=2)
    ax.add_patch(circle)
    ax.set_clip_path(circle)
    
    # Sky and ground
    pitch_offset = pitch_deg / 90
    roll_rad = math.radians(roll_deg)
    
    # Rotated horizon
    cos_r = math.cos(roll_rad)
    sin_r = math.sin(roll_rad)
    
    # Ground (brown) and sky (blue)
    sky_pts = np.array([[-2, pitch_offset], [2, pitch_offset], [2, 3], [-2, 3]])
    gnd_pts = np.array([[-2, -3], [2, -3], [2, pitch_offset], [-2, pitch_offset]])
    
    # Rotate
    def rotate(pts, c, s):
        return np.column_stack([pts[:,0]*c - pts[:,1]*s, pts[:,0]*s + pts[:,1]*c])
    
    sky_rot = rotate(sky_pts, cos_r, sin_r)
    gnd_rot = rotate(gnd_pts, cos_r, sin_r)
    
    ax.fill(sky_rot[:,0], sky_rot[:,1], color='#1e3a5f', alpha=0.8)
    ax.fill(gnd_rot[:,0], gnd_rot[:,1], color='#5c3a1e', alpha=0.8)
    
    # Horizon line
    hl = rotate(np.array([[-1.5, pitch_offset], [1.5, pitch_offset]]), cos_r, sin_r)
    ax.plot(hl[:,0], hl[:,1], 'w-', linewidth=1.5, alpha=0.8)
    
    # Aircraft symbol (fixed)
    ax.plot([-0.4, -0.15], [0, 0], '-', color='#f59e0b', linewidth=3)
    ax.plot([0.15, 0.4], [0, 0], '-', color='#f59e0b', linewidth=3)
    ax.plot(0, 0, 'o', color='#f59e0b', markersize=6)
    
    # Roll indicator arc
    angles = np.linspace(math.radians(210), math.radians(330), 50)
    ax.plot(1.05*np.cos(angles), 1.05*np.sin(angles), '-', color='#8b949e', linewidth=1)
    
    # Roll pointer
    ptr_angle = math.radians(270 - roll_deg)
    ax.plot([0.95*math.cos(ptr_angle)], [0.95*math.sin(ptr_angle)], 'v', color='#f59e0b', markersize=8)
    
    ax.set_title(f'Roll: {roll_deg:+.1f}°  Pitch: {pitch_deg:+.1f}°', 
                 color='#c9d1d9', fontsize=11, fontfamily='monospace', pad=8)


def draw_compass(ax, heading_deg):
    """Heading compass (나침반)"""
    ax.clear()
    ax.set_xlim(-1.3, 1.3)
    ax.set_ylim(-1.3, 1.3)
    ax.set_aspect('equal')
    ax.axis('off')
    
    # Outer ring
    circle = plt.Circle((0, 0), 1, fill=False, color='#30363d', linewidth=2)
    ax.add_patch(circle)
    inner = plt.Circle((0, 0), 0.95, fill=True, color='#0d1117', alpha=0.9)
    ax.add_patch(inner)
    
    # Cardinal directions (rotated by heading)
    cardinals = {'N': 0, 'E': 90, 'S': 180, 'W': 270}
    for label, angle in cardinals.items():
        rad = math.radians(90 - (angle - heading_deg))
        x, y = 0.82 * math.cos(rad), 0.82 * math.sin(rad)
        color = '#ef4444' if label == 'N' else '#8b949e'
        ax.text(x, y, label, ha='center', va='center', fontsize=12, 
                fontweight='bold', color=color, fontfamily='monospace')
    
    # Tick marks
    for i in range(36):
        angle = math.radians(90 - (i * 10 - heading_deg))
        r1 = 0.92 if i % 9 == 0 else 0.95
        r2 = 1.0
        ax.plot([r1*math.cos(angle), r2*math.cos(angle)],
                [r1*math.sin(angle), r2*math.sin(angle)],
                '-', color='#8b949e', linewidth=1 if i % 9 == 0 else 0.5)
    
    # Heading pointer (top, fixed)
    ax.plot([0], [1.08], 'v', color='#f59e0b', markersize=10)
    
    # Aircraft icon in center
    ax.plot([0, 0], [0.15, 0.3], '-', color='#3fb950', linewidth=2)
    ax.plot([-0.12, 0.12], [0.05, 0.05], '-', color='#3fb950', linewidth=2)
    ax.plot([-0.06, 0.06], [-0.1, -0.1], '-', color='#3fb950', linewidth=1.5)
    
    ax.set_title(f'Heading: {heading_deg:.1f}°', 
                 color='#c9d1d9', fontsize=11, fontfamily='monospace', pad=8)


def draw_depth_gauge(ax, depth, max_depth=12):
    """Depth profile (수심 게이지)"""
    ax.clear()
    ax.set_facecolor('#0d1117')
    
    # Water gradient (shallow = light blue, deep = dark blue)
    for i in range(20):
        d = i / 20 * max_depth
        color_val = 0.15 + 0.4 * (1 - i/20)
        ax.axhspan(d, d + max_depth/20, color=(0.05, color_val * 0.6, color_val), alpha=0.5)
    
    # Current depth marker
    ax.axhline(y=depth, color='#f59e0b', linewidth=2, linestyle='--', alpha=0.8)
    ax.plot(0.5, depth, '>', color='#f59e0b', markersize=15, transform=ax.get_yaxis_transform())
    
    ax.set_ylim(max_depth, 0)
    ax.set_xlim(0, 1)
    ax.set_xticks([])
    ax.set_ylabel('Depth (m)', color='#8b949e', fontsize=10)
    ax.tick_params(colors='#8b949e', labelsize=9)
    ax.spines['top'].set_visible(False)
    ax.spines['right'].set_visible(False)
    ax.spines['bottom'].set_visible(False)
    ax.spines['left'].set_color('#30363d')
    
    ax.text(0.5, depth + 0.3, f'{depth:.1f}m', ha='center', va='top',
            color='#f59e0b', fontsize=14, fontweight='bold', fontfamily='monospace')


def run_visualizer(use_serial=False, port='COM5', baud=115200):
    """메인 시각화 루프"""
    
    # Figure 설정
    plt.style.use('dark_background')
    fig = plt.figure(figsize=(14, 7), facecolor='#0d1117')
    fig.suptitle('AUV Navigation Monitor', color='#c9d1d9', fontsize=16, 
                 fontfamily='monospace', fontweight='bold', y=0.98)
    
    gs = GridSpec(2, 4, figure=fig, hspace=0.35, wspace=0.3,
                  left=0.05, right=0.95, top=0.92, bottom=0.08)
    
    ax_attitude = fig.add_subplot(gs[0, 0:2])   # Attitude indicator
    ax_compass = fig.add_subplot(gs[0, 2:4])     # Compass
    ax_depth = fig.add_subplot(gs[1, 0])          # Depth gauge
    ax_timeline = fig.add_subplot(gs[1, 1:3])     # Roll/Pitch/Heading history
    ax_status = fig.add_subplot(gs[1, 3])          # Status panel
    
    # History buffers
    history_len = 80
    t_history = deque(maxlen=history_len)
    roll_history = deque(maxlen=history_len)
    pitch_history = deque(maxlen=history_len)
    heading_history = deque(maxlen=history_len)
    depth_history = deque(maxlen=history_len)
    
    # Generate mission data
    mission_data = generate_mission_data()
    
    plt.ion()
    plt.show()
    
    # Serial or simulation
    ser = None
    if use_serial:
        try:
            import serial
            ser = serial.Serial(port, baud, timeout=0.1)
            time.sleep(2)
            print(f"Connected to {port}")
        except Exception as e:
            print(f"Serial failed: {e}, using simulation")
            use_serial = False
    
    frame = 0
    try:
        while True:
            if use_serial and ser:
                # Real serial data
                line = ser.readline().decode('utf-8', errors='replace').strip()
                if not line or 'Roll:' not in line:
                    continue
                try:
                    parts = line.split()
                    roll = float(parts[1].replace('°', ''))
                    pitch = float(parts[3].replace('°', ''))
                    heading = float(parts[5].replace('°', ''))
                    depth = 0.0  # STM32 doesn't send depth yet
                    phase = "LIVE"
                    motor = "UNKNOWN"
                except (IndexError, ValueError):
                    continue
            else:
                # Simulation
                if frame >= len(mission_data):
                    frame = 0
                d = mission_data[frame]
                roll = d['roll']
                pitch = d['pitch']
                heading = d['heading'] % 360
                depth = max(0, d['depth'])
                phase = d['phase']
                motor = d['motor']
                frame += 1
            
            # Update histories
            t_val = frame * 0.5
            t_history.append(t_val)
            roll_history.append(roll)
            pitch_history.append(pitch)
            heading_history.append(heading)
            depth_history.append(depth)
            
            # Draw attitude indicator
            draw_attitude_indicator(ax_attitude, roll, pitch)
            
            # Draw compass
            draw_compass(ax_compass, heading)
            
            # Draw depth gauge
            draw_depth_gauge(ax_depth, depth)
            
            # Draw timeline
            ax_timeline.clear()
            ax_timeline.set_facecolor('#0d1117')
            if len(t_history) > 1:
                t_arr = list(t_history)
                ax_timeline.plot(t_arr, list(roll_history), '-', color='#ef4444', linewidth=1.2, label='Roll', alpha=0.8)
                ax_timeline.plot(t_arr, list(pitch_history), '-', color='#3b82f6', linewidth=1.2, label='Pitch', alpha=0.8)
                ax_timeline.axhline(y=0, color='#30363d', linewidth=0.5, linestyle='--')
            ax_timeline.set_ylabel('Degrees', color='#8b949e', fontsize=9)
            ax_timeline.set_xlabel('Time (s)', color='#8b949e', fontsize=9)
            ax_timeline.tick_params(colors='#8b949e', labelsize=8)
            ax_timeline.legend(loc='upper right', fontsize=8, 
                             facecolor='#161b22', edgecolor='#30363d',
                             labelcolor='#c9d1d9')
            ax_timeline.spines['top'].set_visible(False)
            ax_timeline.spines['right'].set_visible(False)
            ax_timeline.spines['bottom'].set_color('#30363d')
            ax_timeline.spines['left'].set_color('#30363d')
            ax_timeline.set_title('Attitude history', color='#8b949e', fontsize=10, pad=5)
            
            # Draw status panel
            ax_status.clear()
            ax_status.set_xlim(0, 1)
            ax_status.set_ylim(0, 1)
            ax_status.axis('off')
            ax_status.set_facecolor('#0d1117')
            
            # Phase color
            phase_colors = {
                'SURFACE': '#3fb950', 'DIVING': '#58a6ff',
                'GLIDING': '#a78bfa', 'EMERGENCY': '#f85149', 'LIVE': '#f59e0b'
            }
            pc = phase_colors.get(phase, '#8b949e')
            
            # Status card background
            rect = patches.FancyBboxPatch((0.02, 0.02), 0.96, 0.96,
                                          boxstyle="round,pad=0.03",
                                          facecolor='#161b22', edgecolor='#30363d')
            ax_status.add_patch(rect)
            
            ax_status.text(0.5, 0.92, 'STATUS', ha='center', va='top',
                          color='#8b949e', fontsize=9, fontfamily='monospace')
            
            # Phase indicator
            phase_rect = patches.FancyBboxPatch((0.1, 0.72), 0.8, 0.14,
                                                 boxstyle="round,pad=0.02",
                                                 facecolor=pc, alpha=0.2, edgecolor=pc)
            ax_status.add_patch(phase_rect)
            ax_status.text(0.5, 0.79, phase, ha='center', va='center',
                          color=pc, fontsize=13, fontweight='bold', fontfamily='monospace')
            
            ax_status.text(0.08, 0.60, f'Motor:', color='#8b949e', fontsize=8, fontfamily='monospace')
            ax_status.text(0.08, 0.52, f'{motor}', color='#c9d1d9', fontsize=9, fontfamily='monospace')
            
            ax_status.text(0.08, 0.40, f'Depth:', color='#8b949e', fontsize=8, fontfamily='monospace')
            ax_status.text(0.08, 0.32, f'{depth:.1f} m', color='#58a6ff', fontsize=12, 
                          fontweight='bold', fontfamily='monospace')
            
            ax_status.text(0.08, 0.20, f'Heading:', color='#8b949e', fontsize=8, fontfamily='monospace')
            ax_status.text(0.08, 0.12, f'{heading:.1f}', color='#f59e0b', fontsize=12, 
                          fontweight='bold', fontfamily='monospace')
            
            # Update
            fig.canvas.draw_idle()
            fig.canvas.flush_events()
            plt.pause(0.08)
    
    except KeyboardInterrupt:
        print("\nVisualization stopped.")
    finally:
        if ser:
            ser.close()
        plt.ioff()
        plt.close()


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='AUV Navigation Visualizer')
    parser.add_argument('--port', type=str, default='COM5', help='Serial port')
    parser.add_argument('--baud', type=int, default=115200, help='Baud rate')
    parser.add_argument('--live', action='store_true', help='Use real serial data')
    args = parser.parse_args()
    
    print("=" * 50)
    print("  AUV Navigation Visualizer")
    print("  Press Ctrl+C to stop")
    print("=" * 50)
    
    run_visualizer(use_serial=args.live, port=args.port, baud=args.baud)
