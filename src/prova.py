#!/usr/bin/python3
import numpy as np

import rospy
from geometry_msgs.msg import Pose, Point, Quaternion

from GripperController import GripperController

'''
initial_pos = [0, 0, 1.261]
initial_orient = [[1, 0, 0],
                  [0, 1, 0],
                  [0, 0, 1]]
'''

def interpolate_point(p0, p1, t):
    return p1 * t + p0 * (1 - t)



if __name__ == "__main__":

    rospy.init_node('main_node')

    # Get topic names from paremeter server
    joint_command_topic = rospy.get_param("joint_command_topic")
    joint_state_topic = rospy.get_param("joint_state_topic")
    desired_pose_topic = rospy.get_param("desired_pose_topic")
    gripper_command_topic = rospy.get_param("gripper_command_topic")

    # Initialize publisher for desired pose
    pub = rospy.Publisher(desired_pose_topic, Pose, queue_size=1)

    # Driver to control the gripper
    gripper = GripperController(gripper_command_topic)

    # Sleep to make sure controllers are setup
    rospy.sleep(5)


    pose = Pose(position = Point(0.7, 0.2, 0.7),
                orientation = Quaternion(0, 1, 0, 0)
    )
    pub.publish(pose)

    nsteps = 10
    p0 = np.array([0.7, 0.2, 0.7])      # Initial position
    p1 = np.array([0.5, -0.5, 0.19])    # Final desired position
    
    #vel = (p1 - p0) / 5
    #print(vel)

    rate = rospy.Rate(1.9)
    for t in range(nsteps):
        p = interpolate_point(p0, p1, (t+1)/nsteps)

        pose = Pose(position = Point(*tuple(p)),
                    orientation = Quaternion(0, 1, 0, 0)
        )
        pub.publish(pose)

        rate.sleep()
    
    rospy.sleep(2)

    gripper.close()

    rospy.sleep(2)

    '''
    p0 = np.array([0.5, -0.5, 0.19])      # Initial position
    p1 = np.array([0.2, -0.5, 0.5])     # Final desired position

    for t in range(nsteps):
        p = interpolate_point(p0, p1, (t+1)/nsteps)

        pose = Pose(position = Point(*tuple(p)),
                    orientation = Quaternion(0, 1, 0, 0)
        )
        pub.publish(pose)
    '''