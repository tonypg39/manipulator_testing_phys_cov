import rospy


class Sampler():

    def __init__(self, sample_period = 100) -> None:
        
        self.D = [] # list with all the data from a single experiment
        self.sample_period = sample_period
        # recording variables
        self.count = 0
        self.sampling_count = self.get_sampling_count(self.sample_period)
        self.js_subscriber = rospy.Subscriber("/joint_states",DataType,self.js_handler) # FIXME: Fill in the correct value for the joint_state datatype
        self.recording = False


    def get_sampling_count(self, frequency):
        sample_freq = 1 / self.sample_period
        signal_freq = 50 # TODO: Replace this with the actual value
        c = max(1, int(sample_freq/signal_freq))
        return c
    
    def start_record(self):
        self.recording = True
    
    def stop_record(self):
        self.recording = False

    def js_handler(self, data):
        if not self.recording:
            return
        if self.count == self.sampling_count:
            self.count = 0 
            self.D.append(data)
        else:
            self.count += 1
    
    def export_data(self):
        # Implement the export data functionality
        pass


if __name__ == "__main__":
    s = Sampler()
    rospy.init_node("Tracker")

    # FIXME: Fix with the real method for waiting for messages
    while not rospy.wait_for_topic("/start_solver"):
        pass
    
    s.start_record()
    # FIXME: Fix with the real method for waiting for messages
    while not rospy.wait_for_topic("/finished_task"):
        pass

    s.stop_record()

    

    
