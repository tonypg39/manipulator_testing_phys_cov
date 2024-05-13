"""
This script should find a way to analyze the information stored in the runs inside of dev/eval_data
"""
import os
import numpy as np
import json
import matplotlib.pyplot as plt
import math
from utils import get_dev_path, read_json_file

dev_path = get_dev_path()
data_path =  f"{dev_path}example_data/"


inf = 1e8
W,L,H = 10,10,3
x_range = [-0.75, 0.83]
y_range = [0.02, 0.86]
z_range = [0.10, 0.53]

def get_vel_index(record):
    # returns vi form (0 to V-1)
    thresholds = [0.33,0.67,inf]
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

## Exploratory Methods

def get_ranges(records):
    
    x_r, y_r, z_r, v_r = [inf,-inf],[inf,-inf],[inf,-inf],[inf,-inf]
    for r in records:
        d = np.array(r['data'])
        x_r = [min(x_r[0], np.min(d[:,0])),max(x_r[1], np.max(d[:,0]))]
        y_r = [min(y_r[0], np.min(d[:,1])),max(y_r[1], np.max(d[:,1]))]        
        z_r = [min(z_r[0], np.min(d[:,2])),max(z_r[1], np.max(d[:,2]))]        
        v_r = [min(v_r[0], np.min(d[:,3])),max(v_r[1], np.max(d[:,3]))]   
    return {
        'x': x_r,
        'y': y_r,
        'z': z_r,
        'v': v_r
    }

#########################


def run_evol_matrix(records):
    # Record evolution metrics and return the final matrix
    M = np.zeros((W+2,L+2,H+2),dtype='int8')
    num_covered_cells = []
    te_data = []
    cov_cells = 0
    for record in records:
        for r in record['data']:
            v = get_vel_index(r)
            pos = get_pos_index(r)
#             print(f"The record {r[:4]} got the cell {pos} | vel {v}")
#             print(f"Vel {v} | Marking with {1<<v}")
            M[pos] |= 1<<(v)
            # The velocities are stored in a binary signature: 100 => only the second highest (fastest) velocity range was covered
            # Potential metric: how many cells have non zero velocity: sum(M>0)
            cov_cells = np.sum(M>0)
        num_covered_cells.append(cov_cells)
        te_data.append(record['time_elapsed'])
        # TODO-DEV: Add more Metrics to be plot

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
