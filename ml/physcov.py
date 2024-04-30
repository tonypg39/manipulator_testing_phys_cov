"""
This script should find a way to analyze the information stored in the runs inside of dev/eval_data
"""
import os
import numpy as np
import json
import matplotlib.pyplot as plt

#FIXCONFIG: Add the path to a config file
data_path = "/root/UR5-Pick-and-Place-Simulation/ml/dev/eval_data/distance/"
#SE-FIX: Add the function to a common place at least for ML folder
def read_json_file(file_path):
	with open(file_path, 'r') as file:
		data = json.load(file)
	return data

te_data = []
lf = os.listdir(data_path)
for f in lf:
	d = read_json_file(data_path + f)
	te_data.append(d['time_elapsed'])

print(te_data)
plt.scatter(range(len(te_data)), te_data)
plt.xticks([2,4,6,8,10,12,14,16,18,20])
plt.xlabel("Test Case #")
plt.ylabel("Time Elapsed (seconds)")
plt.grid()
plt.show()