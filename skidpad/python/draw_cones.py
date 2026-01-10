import csv
import numpy as np
from pathlib import Path
import matplotlib.pyplot as plt

SCRIPT_DIR = Path(__file__).resolve().parent
CONES_PATH = SCRIPT_DIR.parent / "cpp" / "cones.csv"

def load_cones_numpy(filename):
    blue, yellow, orange = [], [], []

    with open(filename, newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            x = float(row["x"])
            y = float(row["y"])
            color = row["color"].strip()

            pt = [x, y]

            if color == "blue":
                blue.append(pt)
            elif color == "yellow":
                yellow.append(pt)
            elif color.startswith("orange"):
                orange.append(pt)

    return {
        "blue": np.array(blue),
        "yellow": np.array(yellow),
        "orange": np.array(orange)
    }

# ---- Load cones ----
cones = load_cones_numpy(CONES_PATH)

print("Blue cones shape:", cones["blue"].shape)
print("Yellow cones shape:", cones["yellow"].shape)
print("Orange cones shape:", cones["orange"].shape)

# ---- Plot ----
blue = cones["blue"]
yellow = cones["yellow"]
orange = cones["orange"]

plt.figure(figsize=(6, 6))

if blue.size:
    plt.scatter(blue[:, 0], blue[:, 1], c="blue", s=8, label="Blue cones")
if yellow.size:
    plt.scatter(yellow[:, 0], yellow[:, 1], c="gold", s=8, label="Yellow cones")
if orange.size:
    plt.scatter(orange[:, 0], orange[:, 1], c="orange", s=40, marker="s", label="Orange cones")

# Circle centers (for sanity check)
plt.scatter([-10, 10], [0, 0], c="red", s=80, marker="x", label="Circle centers")

plt.axis("equal")
plt.grid(True)
plt.legend()
plt.title("Skidpad Cones")
plt.xlabel("x [m]")
plt.ylabel("y [m]")

plt.show()
