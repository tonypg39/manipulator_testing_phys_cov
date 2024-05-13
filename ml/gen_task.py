# Process
# Generate random parameters:
# * task_params = {} # example_dict
# * use np.random.randint() and 
# np.random.rand()*2 -1 to generate the pos_x and pos_y
# * export new dict into task_params.json in the desired location in the ros project. 
import numpy as np
import pandas as pd
import json
import os
import math
from datetime import datetime
from utils import get_dev_path
task_params = {
 "task_id": "01",
 "brick_type": 2, # index 0 -> 10 defining which type from the brickDict
 "posX": -0.5, # value from -1 to 1, px is calculated as px = posX * (total_x/2)
 "posY": 1, # value from -1 to 1, py is calculated as py = posy * (total_y/2)
 "rotationZ": 45, #in degrees (-180 to 180)
 "color": 2 # index of ColorList (0 -> 9  = len(colorList))
}

dev_path = get_dev_path()
file_path = dev_path 
csv_path = "data/runs.csv"

def gen_csv(task_params,file_path):
    headers = list(task_params.keys())
    df = pd.DataFrame(columns=headers)
    df.to_csv(file_path,index=False)

def generate():
    ## Checking if the csv file exists already
    csv_file = file_path + csv_path
    if not os.path.exists(csv_file):
        gen_csv(task_params, csv_file)

    t = task_params.copy()
    t['task_id'] = datetime.now().strftime('%Y%m%dT%H%M%S')
    t['posX'] = round(np.random.rand()*2 - 1, 2)
    t['posY'] = round(np.random.rand()*2 - 1, 2)
    t['rotationZ'] = round(np.random.randint(360) - 180)
    t['color'] = np.random.randint(10)
    t['brick_type'] = np.random.randint(10)
    ts = json.dumps(t)

    with open(file_path  + "task_params.json", 'w') as f:
        f.write(ts)


def generate_tests_json_file(n, filename):
    tests = {"tests": []}
    for i in range(n):
        t = {}
        t['task_id'] = str(i)
        t['posX'] = round(np.random.rand()*2 - 1, 2)
        t['posY'] = round(np.random.rand()*2 - 1, 2)
        t['rotationZ'] = round(np.random.randint(360) - 180)
        t['color'] = np.random.randint(9)
        t['brick_type'] = np.random.randint(10)
        tests["tests"].append(t)
    
    with open(filename, 'w') as f:
        f.write(json.dumps(tests))




if __name__ == "__main__":
    # generate()
    generate_tests_json_file(20,"./ml/dev/eval_tests/tests01.json")



    