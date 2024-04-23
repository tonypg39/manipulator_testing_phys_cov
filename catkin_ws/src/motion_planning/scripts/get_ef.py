#!/usr/bin/env python3
import rospy
import tf2_ros
import geometry_msgs.msg

def print_pose():
    rospy.init_node('ur5_pose_listener')

    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        while not tfBuffer.can_transform('base_link', 'robotiq_85_right_inner_knuckle_link', rospy.Time(0)):
            pass
        trans = tfBuffer.lookup_transform('base_link', 'robotiq_85_right_inner_knuckle_link', rospy.Time(0))
        position = trans.transform.translation

        print("\n\n---\nPosition: ", trans.transform.translation)
        print("\n\n---\nPositionx: ", position.x)
        # try:
        #     trans = tfBuffer.lookup_transform('base_link', 'robotiq_85_right_inner_knuckle_link', rospy.Time())
        #     print("\n\n---\nPosition: ", trans.transform.translation)
        #     # print("Orientation: ", trans.transform.rotation)
        # except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        #     continue
        rate.sleep()

if __name__ == '__main__':
    print_pose()
