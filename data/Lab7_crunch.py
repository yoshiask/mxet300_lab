import matplotlib.pyplot as plt
import pandas as pd

sample_nums = []
speeds = []
targets = []

# Read data
with open(".\\data\\Lab7_data8.csv", "r") as data_file:
    #data_file.readline()
    for line in data_file:
        cells = [float(c) for c in line.strip().split(',')]
        sample_nums.append(cells[0])
        speeds.append(cells[1])
        targets.append(cells[2])

plt.title("Target & Current Speed (Left Wheel), k_p = k_i = k_d = 0.04")
plt.xlabel("Sample Number")
plt.ylabel("Speed (rad/s)")
plt.plot(sample_nums, speeds, label="Speed")
plt.plot(sample_nums, targets, label="Target")
plt.legend(loc="lower center")
plt.show()