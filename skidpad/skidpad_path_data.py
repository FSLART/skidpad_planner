import numpy as np
from scipy.interpolate import splprep, splev
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

from matplotlib.animation import PillowWriter

# -------------------------------------------------------------------
# SETTINGS
use_spline = False  # Set to False to skip interpolation
rotate_cones = 0   # Angle of rotation
random_point = np.array([-20, -20]) # or None
animation_speed = 5 # lower number = faster animation speed

# -------------------------------------------------------------------
# PRESET PATH
# Hard-coded midle cones placement
example_cones = np.array([
    [0.0, -1.5],   # cone 1
    [0.0,  1.5],   # cone 2
    [3.0, -1.5],   # cone 3
    [3.0,  1.5]    # cone 4
])

cones_center_pos = example_cones.mean(axis=0)
translation_cones = example_cones - cones_center_pos

theta_cones = np.deg2rad(-rotate_cones)
cones_rotation_matrix = np.array([
    [np.cos(theta_cones), -np.sin(theta_cones)],
    [np.sin(theta_cones),  np.cos(theta_cones)]
])

apply_rotation_cones = translation_cones @ cones_rotation_matrix.T + cones_center_pos

# -------------------------------------------------------------------
# CONES and TRACK
CONES_DATA = np.load('./CONES_DATA.npy', allow_pickle=True)
BASE_SKIDPAD_PATH = np.load('./BASE_SKIDPAD.npy', allow_pickle=True)

# -------------------------------------------------------------------
# TRACK SETUP
mid12 = (apply_rotation_cones[0] + apply_rotation_cones[1]) / 2
mid34 = (apply_rotation_cones[2] + apply_rotation_cones[3]) / 2
middle_line = mid34 - mid12
rotation = np.arctan2(middle_line[1], middle_line[0])  # radians

ZERO_POS = [cones_center_pos[0], cones_center_pos[1], rotation]
step: int = 10
BASE_SKIDPAD_STEP = BASE_SKIDPAD_PATH[::step]

theta = ZERO_POS[2]
rotaion_matrix = np.array([
    [np.cos(theta), -np.sin(theta)],
    [np.sin(theta),  np.cos(theta)]
])
rotated_skidpad = BASE_SKIDPAD_PATH @ rotaion_matrix.T
rotated_skidpad[:,0] += ZERO_POS[0]
rotated_skidpad[:,1] += ZERO_POS[1]

# ——————————————————————————————————————————————
# MIDDLE CONES (already rotated earlier)
cones_middle = apply_rotation_cones

# ——————————————————————————————————————————————
# FUNCTION: produce a pair of cones at track start/end
def make_cone_pair(center_point, spacing=1.5, width=3.0, theta=0):
    R = np.array([
        [np.cos(theta), -np.sin(theta)],
        [np.sin(theta),  np.cos(theta)]
    ])

    # relative cone positions before rotation
    local = np.array([
        [0, -spacing],
        [0,  spacing],
        [width, -spacing],
        [width,  spacing]
    ])

    rotated = local @ R.T + center_point
    return rotated

# ——————————————————————————————————————————————
# START & END CONES (correct orientation)
theta_track = ZERO_POS[2]      # rotation angle of the skidpad
start_pt    = rotated_skidpad[0]
end_pt      = rotated_skidpad[-1]

cones_start = make_cone_pair(start_pt, spacing=1.5, width=3.0, theta=theta_track)
cones_end   = make_cone_pair(end_pt,   spacing=1.5, width=3.0, theta=theta_track)

# ——————————————————————————————————————————————
# COMBINE: start + middle + end
ALL_CONES = np.vstack([cones_start, cones_middle, cones_end])

# -------------------------------------------------------------------
# CLOSEST POINT INDEX
def closest_index_to(point, path):
    diff = path - point
    dists = np.hypot(diff[:, 0], diff[:, 1])
    return np.argmin(dists)

# -------------------------------------------------------------------
# RANDOM START POSITION
# set current_path to rotated_slidpad
current_path = rotated_skidpad[::step]
if random_point is not None:
    idx = closest_index_to(random_point, current_path)
    connector_count = 20  # number of connecting points
    connector = np.linspace(random_point, current_path[idx], connector_count)

    # Inject connector before the real path
    current_path = np.vstack([connector, current_path[idx:]])

start_index = 0

# -------------------------------------------------------------------
# OPTIONAL SPLINE
if use_spline:
    # Remove duplicate points
    mask = np.any(np.diff(current_path, axis=0) != 0, axis=1)
    mask = np.insert(mask, 0, True)
    current_path = current_path[mask]

    # Spline order
    k = 3
    if current_path.shape[0] <= k:
        k = current_path.shape[0] - 1

    points = current_path.T
    tck, u = splprep(points, s=0.5, k=k)
    smooth_count = 2000
    u_fine = np.linspace(0, 1, smooth_count)
    x_spline, y_spline = splev(u_fine, tck)
    current_path = np.vstack([x_spline, y_spline]).T

# -------------------------------------------------------------------
# ANIMATION
diffs = np.diff(current_path, axis=0)
distances = np.cumsum(np.hypot(diffs[:, 0], diffs[:, 1]))
distances = np.insert(distances, 0, 0)
window_length = 20

fig, ax = plt.subplots(figsize=(6, 6))
ax.set_aspect('equal')
ax.grid(True)
ax.set_xlim(rotated_skidpad[:, 0].min() - 5, rotated_skidpad[:, 0].max() + 5)
ax.set_ylim(rotated_skidpad[:, 1].min() - 5, rotated_skidpad[:, 1].max() + 5)

ax.scatter(ALL_CONES[:,0], ALL_CONES[:,1], color="orange", s=80)
# ax.scatter(apply_rotation_cones[:, 0], apply_rotation_cones[:, 1], color="orange", s=80, label="Cones")
if random_point is not None:
    ax.scatter(random_point[0], random_point[1], color="red", s=100, label="Start Point")

(line,) = ax.plot([], [], ".", color="blue", label="Skidpad")
heading_line, = ax.plot([], [], color="black", linewidth=1, label="Vector direction")
ax.legend()

def update(frame):
    idx = start_index + frame
    idx = min(idx, len(current_path) - 1)  # clamp to avoid IndexError
    start_idx = np.searchsorted(distances, max(distances[idx] - window_length, 0))

    line.set_data(current_path[start_idx:idx, 0], current_path[start_idx:idx, 1])

    if idx > 0:
        p, p_prev = current_path[idx], current_path[idx - 1]
        direction = p - p_prev
        length = np.linalg.norm(direction)
        if length > 0:
            direction = direction / length * 5
        heading_line.set_data([p[0], p[0] + direction[0]], [p[1], p[1] + direction[1]])
    else:
        heading_line.set_data([], [])

    return line, heading_line

ani = FuncAnimation(fig, update, frames=len(current_path), interval=animation_speed, blit=True, repeat=True)

# ——————————————————————————————————————————————
# SAVE AS A GIF
# ani = FuncAnimation(
#     fig,
#     update,
#     frames=len(current_path),
#     interval=animation_speed,
#     blit=True,
#     repeat=True
# )
# ani.save("skidpad_animation_random_point.gif", writer=PillowWriter(fps=30))

# ——————————————————————————————————————————————
# SHOW PLOT, 
plt.show()