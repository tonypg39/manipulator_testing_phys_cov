# Do the loop around 1,2,3
import argparse
from gen_task import generate
from simulation import main_run, main_kill
from datetime import datetime
import json
import time
from utils import read_json_file, get_dev_path


dev_path = get_dev_path()
tests_file_path = f"{dev_path}eval_tests/"
task_params_file_path = dev_path


def load_test(test):
    test['task_id'] = test['task_id'] + datetime.now().strftime('%Y%m%dT%H%M%S')
    ts = json.dumps(test)
    with open(task_params_file_path  + "task_params.json", 'w') as f:
        f.write(ts)


if __name__ == "__main__":
    if __name__ == "__main__":
        cmdline = argparse.ArgumentParser(formatter_class=argparse.ArgumentDefaultsHelpFormatter)
        cmdline.add_argument('--test_file', default="distance.json")
        # cmdline.add_argument('--id', default="")
        flags, unk_args = cmdline.parse_known_args()
        f_test = flags.test_file
    
    tests = read_json_file(tests_file_path + f_test)
    for t in tests["tests"]:
        # generate()
        load_test(t)
        threads = main_run()
        file_path = dev_path
        while True:
            d  = read_json_file(file_path + "status.json")
            if d['state'] == "finished":
                break
            time.sleep(1.5)
        
        time.sleep(1)
        main_kill(threads)