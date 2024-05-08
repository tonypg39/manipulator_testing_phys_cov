#!/usr/bin/env python3
import rospy
import tf2_ros
import math
import geometry_msgs.msg
from std_msgs.msg import String

def publish_pose():
    rospy.init_node('ur5_pose_listener')

    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)
    pub = rospy.Publisher('end_effector_pos', String, queue_size=10)

    # FIXCONFIG : Set the sampling rate to be a parameter
    sampling_freq = 10.0 
    rate = rospy.Rate(sampling_freq)
    last_pos = None
    vel = 0.0
    rospy.loginfo("Starting to publish the EF position...")
    while not rospy.is_shutdown():
        while not tfBuffer.can_transform('base_link', 'robotiq_85_right_inner_knuckle_link', rospy.Time(0)):
            pass
        trans = tfBuffer.lookup_transform('base_link', 'robotiq_85_right_inner_knuckle_link', rospy.Time(0))
        position = trans.transform.translation
        if last_pos is not None:
            dpos = math.sqrt((position.x - last_pos.x)**2 + (position.y - last_pos.y)**2 + (position.z - last_pos.z)**2)
            vel = dpos / (1/sampling_freq)
        # compute ==> vel = (pos - last_pos) / (1/sampling_freq)
        pub.publish(f"{position.x}|{position.y}|{position.z}|{vel}")
        rate.sleep()


if __name__ == '__main__':
    publish_pose()
