#!/usr/bin/env python3

import yaml
import matplotlib.pyplot as plt
import numpy as np
import math

def quaternion_to_yaw(q):
    """Convert quaternion (x,y,z,w) to yaw (2D heading)."""
    siny_cosp = 2 * (q['w'] * q['z'] + q['x'] * q['y'])
    cosy_cosp = 1 - 2 * (q['y']**2 + q['z']**2)
    return math.atan2(siny_cosp, cosy_cosp)

def yaw_to_quaternion(yaw):
    """Convert yaw (heading) to quaternion dict."""
    return {
        'x': 0.0,
        'y': 0.0,
        'z': math.sin(yaw / 2.0),
        'w': math.cos(yaw / 2.0)
    }

WAYPOINT_FILE = '/home/mbiason/waypoints.yaml'
OUTPUT_FILE = '/home/mbiason/waypoints_normalized.yaml'

# --- Load waypoints ---
with open(WAYPOINT_FILE) as f:
    data = yaml.safe_load(f)

wps = data['waypoints']
xs = np.array([wp['pose']['position']['x'] for wp in wps])
ys = np.array([wp['pose']['position']['y'] for wp in wps])
yaws = [quaternion_to_yaw(wp['pose']['orientation']) for wp in wps]

# --- Compute rotation so first segment aligns with X axis ---
if len(xs) >= 2:
    dx = xs[1] - xs[0]
    dy = ys[1] - ys[0]
    angle = -np.arctan2(dy, dx)  # rotate into X axis

    R = np.array([[np.cos(angle), -np.sin(angle)],
                  [np.sin(angle),  np.cos(angle)]])

    coords = np.vstack((xs - xs[0], ys - ys[0]))
    rotated = R @ coords
    xs, ys = rotated[0], rotated[1]

    # Rotate headings as well
    yaws = [yaw + angle for yaw in yaws]

# --- Plot ---
plt.figure()
plt.plot(xs, ys, 'go-', label="Waypoints (normalized)")  # green dots + line
plt.xlabel("X (m)")
plt.ylabel("Y (m)")
plt.axis('equal')
plt.title("Waypoints (aligned with X-axis)")

# Draw orientation arrows
arrow_len = 0.2  # meters
for x, y, yaw in zip(xs, ys, yaws):
    plt.arrow(x, y,
              arrow_len * np.cos(yaw),
              arrow_len * np.sin(yaw),
              head_width=0.1, head_length=0.1, fc='r', ec='r')

plt.legend()
plt.show()

# --- Prompt to save ---
ans = input(f"\nSave normalized waypoints to {OUTPUT_FILE}? [y/N]: ").strip().lower()
if ans == 'y':
    normalized_wps = []
    for x, y, yaw in zip(xs, ys, yaws):
        normalized_wps.append({
            'pose': {
                'position': {'x': float(x), 'y': float(y), 'z': 0.0},
                'orientation': yaw_to_quaternion(yaw)
            }
        })

    # Preserve any other keys in the original file
    out_data = {k: v for k, v in data.items() if k != 'waypoints'}
    out_data['waypoints'] = normalized_wps

    with open(OUTPUT_FILE, 'w') as f:
        yaml.safe_dump(out_data, f, default_flow_style=False)

    print(f"✅ Saved {len(normalized_wps)} waypoints to {OUTPUT_FILE}")
else:
    print("Skipped saving.")
