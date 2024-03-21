import os
# Process
# - Run all the roslaunch and the rosrun commands
# (using the xterm wrapper)

def xterm_wrapper(cmd):
    c = f'xterm -fa "Monospace" -fs 14 -e "{cmd}; bash" &'
    pid = os.popen(c).read()
    return pid

cmds_run = [
    "roslaunch levelManager lego_world.launch",
    "rosrun levelManager gen_lego_world.py",
    "rosrun motion_planning tracker.py",
    "rosrun motion_planning motion_planning.py"
]

kill_ids = []

def main_run():
    for c in cmds_run:
        k = xterm_wrapper(c)
        kill_ids.append(k)

def main_kill():
    for k in kill_ids:
        os.popen(f"kill {k}")
    


