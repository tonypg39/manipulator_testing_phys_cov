import os
import threading
import subprocess
import time
from utils import get_dev_path, read_json_file, update_json_file
import argparse
import psutil


# Process
# - Run all the roslaunch and the rosrun commands
# (using the xterm wrapper)

cmds_run = [
    {"run": "xterm", "cmd": "roslaunch levelManager lego_world.launch" , "wait": 5},
    {"run": "xterm", "cmd": "rosservice call /gazebo/unpause_physics '{}'", "wait": 0},
    {"run": "xterm", "cmd": "rosrun levelManager gen_lego_world.py", "wait": 3},
    {"run": "xterm", "cmd": "rosrun motion_planning get_ef.py", "wait": 0.5},
    {"run": "xterm", "cmd": "rosrun motion_planning tracker.py", "wait":1},
    {"run": "xterm", "cmd": "rosrun motion_planning motion_planning.py", "wait":1}
]

cmds_kill = [
    {"cmd": "killall gzserver" , "wait": 1},
    # {"cmd": "rosnode kill -a" , "wait": 1}

    # {"cmd": "killall gzclient" , "wait": 0.5},
    # {"cmd": "killall xterm" , "wait": 0.5},
]

kill_ids = [None]*len(cmds_run)

def xterm_wrapper(cmd,k_idx):
    c = f'xterm -fa "Monospace" -fs 12 -e "{cmd}; bash" &'
    proc = subprocess.Popen(cmd.split())
    pid = proc.pid
    kill_ids[k_idx] = pid
    return pid

def subprocess_wrapper(cmd, k_idx):
    c = f'{cmd} &'
    pid = os.popen(c).read()
    kill_ids[k_idx] = pid
    return pid

def subproc_wrapper(cmd, k_idx):
    # c = f'{cmd} &'
    proc = subprocess.Popen(cmd.split())
    pid = proc.pid
    kill_ids[k_idx] = pid
    return pid

def main_run():
    """Returns the threads for all the processes"""
    file_path = get_dev_path()

    update_json_file(file_path+"status.json",{"state":"running"})
    threads = []
    for i,c in enumerate(cmds_run):
        # if c["run"] == "xterm":
        #     k = xterm_wrapper(c["cmd"])
        #     kill_ids.append(k)
        # elif c["run"] == "os":
        #     os.popen(c["cmd"])
        if c["run"] == "xterm":
            # th = threading.Thread(target=xterm_wrapper, args=(c["cmd"],i),name=f"t_{i}")
            th = threading.Thread(target=subprocess_wrapper, args=(c["cmd"],i),name=f"t_{i}")
            threads.append(th)
            th.start()
            if c["wait"] > 0:
                time.sleep(c["wait"])
        elif c["run"] == "os":
            os.popen(c["cmd"])
    
    # Give the start command
    
    tp = read_json_file(file_path+"task_params.json")
    task_id = tp["task_id"]
    # os.popen(f"rostopic pub /start_solver std_msgs/String --once \"{task_id}\"")
    print(f"Task Id: {task_id} || Type: {type(task_id)}")
    os.popen(f"rostopic pub /start_solver std_msgs/String --once '{task_id}' ")
    return threads

def kill_processes():
    # Discover the processes
    kill_keys = ["python3", "gzserver", "rosout"]
    for proc in psutil.process_iter(['pid', 'name', 'cmdline']):
        try:
            # Match the command to find the correct process
            if proc.info['cmdline'] and ' '.join(proc.info['cmdline']).find('python3') > -1:
                pid = proc.info['pid']
                name = proc.info['name']
                cmdl = proc.info['cmdline']
                print(f'Killing Process =>  Name: {name} |ID: {pid}')
                os.popen(f"kill {pid}")
        except (psutil.NoSuchProcess, psutil.AccessDenied, psutil.ZombieProcess):
            print("Could not kill the processes...")
        

    # Kill them with kill -9

def main_kill(threads):
    #TODO-DEV: Look into a cleaner kill of gazebo that goes through the processes
    # Kill the gazebo processes
    print("\n\n\n\n***\n\n\nThe IDs of the process are: ", kill_ids)
    for c in cmds_kill:
        os.popen(c["cmd"])
        if c["wait"] > 0:
            time.sleep(c["wait"])

    # for t in threads:
    #     t.join()
    

if __name__=="__main__":
    cmdline = argparse.ArgumentParser(formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    cmdline.add_argument('--hold', action='store_true', help='Hold the simulation after')
    flags, unk_args = cmdline.parse_known_args()

    ts = main_run()
    file_path = get_dev_path()
    while True:
        d  = read_json_file(file_path + "status.json")
        if d['state'] == "finished":
            break
        time.sleep(1.5)
    
    time.sleep(1)
    kill_processes()
    if not flags.hold:
        main_kill(ts)
