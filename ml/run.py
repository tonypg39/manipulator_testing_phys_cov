# Do the loop around 1,2,3

from gen_task import generate
from simulation import main_run, main_kill
import json
import time

# FIXCONFIG: Add connection for the max iterations of the training
max_iter = 10
dev_file_path = "/root/UR5-Pick-and-Place-Simulation/ml/dev"


#SE-FIX Add to a utils file
def read_json_file(file_path):
    with open(file_path, 'r') as file:
        data = json.load(file)
    return data


it = 0

while it < max_iter:
    generate()
    ts = main_run()
    #FIXCONFIG: add path to the config in utils file

    file_path = "/root/UR5-Pick-and-Place-Simulation/ml/dev/"
    while True:
        d  = read_json_file(file_path + "status.json")
        if d['state'] == "finished":
            break
        time.sleep(1.5)
    
    time.sleep(1)
    main_kill(ts)
