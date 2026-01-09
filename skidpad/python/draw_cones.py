import csv
import numpy as np
from pathlib import Path

SCRIPT_DIR = Path(__file__).resolve().parent
CONES_PATH = SCRIPT_DIR.parent / "cpp" / "cones.csv"

def load_cones_numpy(filename):
    blue, yellow, orange = [], [], []

    with open(filename, newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            pt = [float(row["x"]), float(row["y"])]
            if row["color"] == "blue":
                blue.append(pt)
            elif row["color"] == "yellow":
                yellow.append(pt)
            elif row["color"] == "orange":
                orange.append(pt)

    return {
        "blue": np.array(blue),
        "yellow": np.array(yellow),
        "orange": np.array(orange)
    }

cones = load_cones_numpy(CONES_PATH)
print("Blue cones shape:", cones["blue"].shape)

import matplotlib.pyplot as plt

blue = cones["blue"]
yellow = cones["yellow"]
orange = cones["orange"]

plt.figure(figsize=(6,6))

if blue.size:
    plt.scatter(blue[:,0], blue[:,1], c="blue", s=10, label="Blue")
if yellow.size:
    plt.scatter(yellow[:,0], yellow[:,1], c="gold", s=10, label="Yellow")
if orange.size:
    plt.scatter(orange[:,0], orange[:,1], c="orange", s=10, label="Orange")

plt.axis("equal")
plt.grid(True)
plt.legend()
plt.title("Skidpad Cones")
plt.xlabel("x [m]")
plt.ylabel("y [m]")

plt.show()
