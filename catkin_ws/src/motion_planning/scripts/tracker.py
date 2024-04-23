#!/usr/bin/python3
import rospy
import tf2_ros
from sensor_msgs.msg import JointState
from std_msgs.msg import String
import numpy as np
import pandas as pd
import json

# def read_task_params_json():
# 	# FIXCONFIG: replace with the path from settings
#     # SE-FIX: move the method to the one place
# 	file_path = "/root/UR5-Pick-and-Place-Simulation/ml/dev/task_params.json"
# 	with open(file_path, 'r') as file:
# 		data = json.load(file)
# 	return data

#SE-FIX Add to a utils file
def read_json_file(file_path):
	with open(file_path, 'r') as file:
		data = json.load(file)
	return data

# FIXCONFIG Set this path in the config
EXPORT_PATH = "/root/UR5-Pick-and-Place-Simulation/ml/dev/eval_data/"
CSV_PATH = "/root/UR5-Pick-and-Place-Simulation/ml/dev/data/runs.csv"

class Sampler():

    def __init__(self, sample_period = 100) -> None:
        
        self.D = [] # list with all the data from a single experiment
        self.sample_period = sample_period
        # recording variables
        self.count = 0
        self.sampling_count = self.get_sampling_count(self.sample_period)
        # self.js_subscriber = rospy.Subscriber("/joint_states", JointState, self.js_handler) 
        self.ef_subscriber = rospy.Subscriber("/end_effector_pos", String, self.ef_handler)
        self.tfBuffer = tf2_ros.Buffer()
        self.recording = False
        self.task_id = None


    def get_sampling_count(self, frequency):
        sample_freq = 1 / self.sample_period
        signal_freq = 50 # FIXCONFIG: Replace this with the actual value
        # c = max(1, int(sample_freq/signal_freq))
        c = 50
        return c
    
    def start_record(self, task_id):
        self.task_id = task_id
        self.recording = True
    
    def stop_record(self):
        self.recording = False

    def ef_handler(self, msg):
        if not self.recording:
            return
        data = msg.data.split('|')
        p = [float(data[0]), float(data[1]), float(data[2])]   
        print(p)
        self.D.append(p)
    
    def js_handler(self, data):
        if not self.recording:
            return
        if self.count == self.sampling_count:
            self.count = 0 
            # Structure of the data is: [EFx,EFy,EFz,(Joint_Angles{6}), (Joint_Velocities{6})]
            p =  data.position + data.velocity
            self.D.append(p)
            print(p)
        else:
            self.count += 1
    
    def export_data(self):
        # Implement the export data functionality
        ndata = np.array(self.D)
        
        rospy.loginfo(f"The shape of the D is :{self.D} \n {len(self.D)} || {len(self.D[0])}")
        rospy.loginfo(f"The shape of the data is :{ndata.shape}")
        assert ndata.shape[-1] == 3 and len(ndata.shape) == 2
        
        #FIXCONFIG: add path to the config in utils file
        file_path = "/root/UR5-Pick-and-Place-Simulation/ml/dev/"
        tp = read_json_file(file_path+"task_params.json")
        assert self.task_id == tp['task_id']
        # save the motion data Y
        np.save(EXPORT_PATH+f"mov_{self.task_id}.npy",ndata)
        # save the task parameters X in a csv
        self.export_csv(tp)
    
    def export_csv(self, task_p=None):
        if task_p is None:
            task_p = {
                "task_id": "01",
                "brick_type": 2, # index 0 -> 10 defining which type from the brickDict
                "posX": -0.5, # value from -1 to 1, px is calculated as px = posX * (total_x/2)
                "posY": 1, # value from -1 to 1, py is calculated as py = posy * (total_y/2)
                "rotationZ": 45, #in degrees (-180 to 180)
                "color": 2 # index of ColorList (0 -> 9  = len(colorList))
            }
        d = pd.read_csv(CSV_PATH)
        print(d.values.shape)
        d.loc[len(d.index)] =  list(task_p.values())
        d.to_csv(CSV_PATH, index=False)



if __name__ == "__main__":

    s = Sampler()
    rospy.init_node("Tracker")
    start_msg = rospy.wait_for_message("start_solver", String, timeout=None)
    task_id = start_msg.data 

    print("Starting the tracker...")
    s.start_record(task_id)
    # print("Got here... after start")
    #FIXCONFIG: add path to the config in utils file

    file_path = "/root/UR5-Pick-and-Place-Simulation/ml/dev/"
    while True:
        d  = read_json_file(file_path + "status.json")
        if d['state'] == "finished":
            break
        rospy.sleep(1.5)
    s.stop_record()
    s.export_data()

    

    
