import matplotlib.pyplot as plt
import pandas as pd

sample_nums = []
speeds = []
targets = []
k_d = 0.04
k_p = 0.04

# Read data
with open(".\\data\\Lab7_data7.csv", "r") as data_file:
    for line in data_file:
        cells = [float(c) for c in line.strip().split(',')]
        sample_nums.append(int(cells[0]))
        speeds.append(float(cells[1]))
        targets.append(float(cells[2]))

speeds = speeds[10:]
targets = targets[10:]
errors = [targets[i] - speeds[i] for i in range(len(speeds))]

de_idx = max(enumerate(errors),key=lambda x: x[1])[0]
de = errors[de_idx]
print(f"Max error: {de}")

u_d = de * k_d
print(f"Control effort (u_d): {u_d}")

u_p = errors[de_idx + 1] * k_p
print(f"Proportional control effort (u_p): {u_p}")
