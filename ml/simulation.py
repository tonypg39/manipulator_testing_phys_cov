import os
import threading
import time
import json
# Process
# - Run all the roslaunch and the rosrun commands
# (using the xterm wrapper)

cmds_run = [
    {"run": "xterm", "cmd": "roslaunch levelManager lego_world.launch" , "wait": 5},
    {"run": "os", "cmd": "rosservice call /gazebo/unpause_physics '{}'", "wait": 0},
    {"run": "xterm", "cmd": "rosrun levelManager gen_lego_world.py", "wait": 3},
    {"run": "xterm", "cmd": "rosrun motion_planning tracker.py", "wait":1},
    {"run": "xterm", "cmd": "rosrun motion_planning motion_planning.py", "wait":1}
]

cmds_kill = [
    {"cmd": "killall gzserver" , "wait": 1},
    {"cmd": "killall gzclient" , "wait": 0.5},
    {"cmd": "killall xterm" , "wait": 0.5},
]

kill_ids = [None]*len(cmds_run)

# SE-FIX Add to a utils file
def read_json_file(file_path):
	with open(file_path, 'r') as file:
		data = json.load(file)
	return data

# SE-FIX add to a utils file
def update_json_file(file_path,new_d):
    with open(file_path, 'r') as f:
        data = json.load(f)
    data.update(new_d)
    with open(file_path, 'w') as f:
        json.dump(data, f)



def xterm_wrapper(cmd,k_idx):
    c = f'xterm -fa "Monospace" -fs 12 -e "{cmd}; bash" &'
    pid = os.popen(c).read()
    kill_ids[k_idx] = pid
    return pid


def main_run():
    """Returns the threads for all the processes"""
    #FIXCONFIG: add path to the config in utils file
    file_path = "/root/UR5-Pick-and-Place-Simulation/ml/dev/"

    update_json_file(file_path+"status.json",{"state":"running"})
    threads = []
    for i,c in enumerate(cmds_run):
        # if c["run"] == "xterm":
        #     k = xterm_wrapper(c["cmd"])
        #     kill_ids.append(k)
        # elif c["run"] == "os":
        #     os.popen(c["cmd"])
        if c["run"] == "xterm":
            th = threading.Thread(target=xterm_wrapper, args=(c["cmd"],i),name=f"t_{i}")
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

def main_kill(threads):
    #FIXME: Look into a cleaner kill of gazebo that goes through the processes
    # Kill the gazebo processes
    for c in cmds_kill:
        os.popen(c["cmd"])
        if c["wait"] > 0:
            time.sleep(c["wait"])

    for t in threads:
        t.join()
    

if __name__=="__main__":
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
