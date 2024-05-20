# Do the loop around 1,2,3
import argparse
from gen_task import generate
from simulation import main_run, main_kill
from datetime import datetime
import json
import time
from utils import read_json_file, get_dev_path, get_config


dev_path = get_dev_path()
tests_file_path = f"{dev_path}eval_tests/"
task_params_file_path = dev_path


limits = {
    "color": [0,8]
}

def load_test(test):
    test['task_id'] = test['task_id'] + "_" + datetime.now().strftime('%Y%m%dT%H%M%S')
    # check limits
    for k,l in limits.items():
        if not(l[0] <= test[k] <= l[1]):
            test[k] = l[1]
    ts = json.dumps(test)
    with open(task_params_file_path  + "task_params.json", 'w') as f:
        f.write(ts)


if __name__ == "__main__":
    config = get_config()
    cmdline = argparse.ArgumentParser(formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    cmdline.add_argument('--test_file', default="distance.json")
    # cmdline.add_argument('--id', default="")
    flags, unk_args = cmdline.parse_known_args()
    f_test = flags.test_file

    timeout = config["test_timeout"]

    tests = read_json_file(tests_file_path + f_test)
    for i,t in enumerate(tests["tests"]):
        # generate()
        n = len(tests["tests"])
        print(f"\n\n---\nRunning test {i}/{n}")
        start_time = time.time()
        load_test(t)
        threads = main_run()
        file_path = dev_path
        while True and (time.time() - start_time < timeout):
            # d = read_json_file(file_path + "status.json")
            while True:
                try:
                    d = read_json_file(file_path + "status.json")
                    break  # Exit loop if file is read successfully
                except json.JSONDecodeError as e:
                    print(f"ERROR reading JSON: {e}. Retrying...")
                    time.sleep(1.5)  # Wait for 2 seconds before retrying
            
            if d['state'] == "finished":
                break
            time.sleep(1.5)
        
        # TODO-DEV: Add a condition to check for the generated log file
        time.sleep(2)
        main_kill(threads)