"""
This script should find a way to analyze the information stored in the runs inside of dev/eval_data
"""
import os
import numpy as np
import json
import matplotlib.pyplot as plt

#FIXCONFIG: Add the path to a config file
data_path = "/root/UR5-Pick-and-Place-Simulation/ml/dev/example_data/"

# FIXCONFIG: Params for the PhysCov computation
# FIXME: Fix the ranges for the dimensions
inf_velocity = 1e8
W,L,H = 10,10,3
x_range = [-10,10]
y_range = [-5,5]
z_range = [0,4]

#SE-FIX: Add the function to a common place at least for ML folder
def read_json_file(file_path):
    with open(file_path, 'r') as file:
        data = json.load(file)
    return data



def get_vel_index(record):
    # returns vi form (0 to V-1)
    thresholds = [2.0,4.0,inf_velocity] #FIXME: Compute the actual velocity ranges
    v = record[3]
    for i,t in enumerate(thresholds):
        if v < t:
            return i

def get_pos_index(record):
    global W,L,H
    x,y,z = record[:3]
    w_pct = (x-x_range[0])/(x_range[1] - x_range[0])
    wi = int(w_pct * W)
    l_pct = (y-y_range[0])/(y_range[1] - y_range[0])
    li = int(l_pct * L)
    h_pct = (z-z_range[0])/(z_range[1] - z_range[0])
    hi = int(h_pct * H)
    return wi,li,hi
    

def get_time_elapsed(record):
    return record['time_elapsed']


def run_evol_matrix(records):
    # Record evolution metrics and return the final matrix
    M = np.zeros((W,L,H),dtype='int8')
    num_covered_cells = []
    te_data = []
    for record in records:
        cov_cells = 0
        for r in record['data']:
            v = get_vel_index(r)
            pos = get_pos_index(r)
            M[pos] += 1<<(v)
            # The velocities are stored in a binary signature: 100 => only the second highest (fastest) velocity range was covered
            # Potential metric: how many cells have non zero velocity: sum(M>0)
            cov_cells += sum(M>0)
        num_covered_cells.append(cov_cells)
        te_data.append(record['time_elapsed'])
        # TODO: Add more Metrics to be plot

    d = {
        "matrix": M,
        "Covered Cells": num_covered_cells,
        "Time Elapsed": te_data
    }
    return d

def generate_plots(D):
    # plot time elapsed
    te_data = D["Covered Cells"]
    print(D)
    plt.scatter(range(len(te_data)), te_data)
    # plt.xticks([2,4,6,8,10,12,14,16,18,20])
    plt.xlabel("Test Case #")
    plt.ylabel("Time Elapsed (seconds)")
    plt.grid()
    plt.show()

if __name__ == "__main__":
    te_data = []
    lf = os.listdir(data_path)
    records = []
    for f in lf:
        d = read_json_file(data_path + f)
        records.append(d)

    D = run_evol_matrix(records)
    generate_plots(D)


# print(te_data)
