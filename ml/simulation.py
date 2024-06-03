import os
import threading
import time
from utils import get_dev_path, read_json_file, update_json_file
import argparse



# Process
# - Run all the roslaunch and the rosrun commands
# (using the xterm wrapper)

cmds_run = [
    {"run": "xterm", "cmd": "roslaunch levelManager lego_world.launch" , "wait": 5},
    {"run": "os", "cmd": "rosservice call /gazebo/unpause_physics '{}'", "wait": 0},
    {"run": "xterm", "cmd": "rosrun levelManager gen_lego_world.py", "wait": 3},
    {"run": "xterm", "cmd": "rosrun motion_planning get_ef.py", "wait": 0.5},
    {"run": "xterm", "cmd": "rosrun motion_planning tracker.py", "wait":1},
    {"run": "xterm", "cmd": "rosrun motion_planning motion_planning.py", "wait":1}
]

cmds_kill = [
    {"cmd": "killall gzserver" , "wait": 1},
    {"cmd": "killall gzclient" , "wait": 0.5},
    {"cmd": "killall xterm" , "wait": 0.5},
]

kill_ids = [None]*len(cmds_run)

def xterm_wrapper(cmd,k_idx):
    c = f'xterm -fa "Monospace" -fs 12 -e "{cmd}; bash" &'
    pid = os.popen(c).read()
    kill_ids[k_idx] = pid
    return pid


def tmux_wrapper(cmd, session, count):
    c = f'tmux new-window -t {session}:{count} -n "window-{count}" "{cmd}"'
    os.popen(c)



def main_run(task_id):
    """Returns the threads for all the processes"""
    file_path = get_dev_path()    

    # if needed (headless mode), start the tmux session
    # FIXCONFIG: set headless mode variable
    session = f"session-{task_id}"
    os.popen(f"tmux new-session -d -s {session}")  
    time.sleep(1.5)

    update_json_file(file_path+"status.json",{"state":"running"})
    threads = []
    
    for i,c in enumerate(cmds_run):
        if c["run"] == "xterm":
            print(f"\n\n--\nRunning command #{i}: {c['cmd']}")
            th = threading.Thread(target=tmux_wrapper, args=(c["cmd"],session,i+2),name=f"t_{i}")
            threads.append(th)
            th.start()
            if c["wait"] > 0:
                time.sleep(c["wait"])
        elif c["run"] == "os":
            os.popen(c["cmd"])
    
    # Give the start command
    
    
    # os.popen(f"rostopic pub /start_solver std_msgs/String --once \"{task_id}\"")
    print(f"Task Id: {task_id} || Type: {type(task_id)}")
    os.popen(f"rostopic pub /start_solver std_msgs/String --once '{task_id}' ")
    return threads

def main_kill(threads, task_id):
    #TODO-DEV: Look into a cleaner kill of gazebo that goes through the processes
    # Kill the gazebo processes
    # for c in cmds_kill:
    #     os.popen(c["cmd"])
    #     if c["wait"] > 0:
    #         time.sleep(c["wait"])
    os.popen(f"tmux kill-session -t session-{task_id}")  
    # for t in threads:
    #     t.join()
    
    

    

if __name__=="__main__":
    cmdline = argparse.ArgumentParser(formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    cmdline.add_argument('--hold', action='store_true', help='Hold the simulation after')
    flags, unk_args = cmdline.parse_known_args()
    
    file_path = get_dev_path()
    tp = read_json_file(file_path+"task_params.json")
    task_id = tp["task_id"]

    ts = main_run(task_id)
    file_path = get_dev_path()
    while True:
        d  = read_json_file(file_path + "status.json")
        if d['state'] == "finished":
            break
        time.sleep(1.5)
    
    time.sleep(1)
    if not flags.hold:
        main_kill(ts,task_id)
