# Do the loop around 1,2,3

from gen_task import generate
from simulation import main_run, main_kill
from datetime import datetime
import json
import time

# FIXCONFIG: Add connection for the max iterations of the training
tests_file_path = "/root/UR5-Pick-and-Place-Simulation/ml/dev/eval_tests/"
task_params_file_path = "/root/UR5-Pick-and-Place-Simulation/ml/dev/"


# Settings
TEST_NAME = "distance.json"

#SE-FIX Add to a utils file
def read_json_file(file_path):
    with open(file_path, 'r') as file:
        data = json.load(file)
    return data


def load_test(test):
    test['task_id'] = test['task_id'] + datetime.now().strftime('%Y%m%dT%H%M%S')
    ts = json.dumps(test)
    with open(task_params_file_path  + "task_params.json", 'w') as f:
        f.write(ts)


if __name__ == "__main__":
    tests = read_json_file(tests_file_path + "distance.json")
    for t in tests["tests"]:
        # generate()
        load_test(t)
        threads = main_run()
        #FIXCONFIG: add path to the config in utils file

        file_path = "/root/UR5-Pick-and-Place-Simulation/ml/dev/"
        while True:
            d  = read_json_file(file_path + "status.json")
            if d['state'] == "finished":
                break
            time.sleep(1.5)
        
        time.sleep(1)
        main_kill(threads)