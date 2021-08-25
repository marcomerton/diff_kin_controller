#!/usr/bin/python3
import numpy as np

import rospy
from geometry_msgs.msg import Pose, Point, Quaternion, Twist, Vector3

from GripperController import GripperController

'''
initial_pos = [0, 0, 1.261]
initial_orient = [[1, 0, 0],
                  [0, 1, 0],
                  [0, 0, 1]]
'''

def interpolate_point(p0, p1, t):
    ''''''
    return p1 * t + p0 * (1 - t)

def slerp(t, P, Q):
    ''''''
    diff = Q - P
    theta = np.arccos(diff.angle)

    res = ( np.sin(theta/2*(1-t)) * P + np.sin(theta/2*t) * Q ) / np.sin(theta/2)
    return res


def move(start, end, nsteps, time):

    # Compute (constant) velocity
    d = start - end
    tw = Twist(linear=Vector3(d[0], d[1], d[2]), angular=Vector3(0, 0, 0))

    # Start following trajectory
    rate = rospy.Rate(nsteps/time * 0.8)
    for t in range(nsteps):
        p = interpolate_point(start, end, (t+1)/nsteps)

        pose = Pose(position = Point(*tuple(p)), orientation = Quaternion(0, 1, 0, 0))

        pub_pose.publish(pose)
        pub_vel.publish(tw)

        rate.sleep()
    
    # Reset desider velocity to zero
    pub_vel.publish(Twist())

    # Wait for the arm to stabilize
    rospy.sleep(1)


if __name__ == "__main__":

    rospy.init_node('main_node')

    # Get topic names from paremeter server
    joint_command_topic = rospy.get_param("joint_command_topic")
    joint_state_topic = rospy.get_param("joint_state_topic")
    desired_pose_topic = rospy.get_param("desired_pose_topic")
    desired_vel_topic = rospy.get_param("desired_vel_topic")
    gripper_command_topic = rospy.get_param("gripper_command_topic")

    # Initialize publisher for desired pose
    pub_pose = rospy.Publisher(desired_pose_topic, Pose, queue_size=1)

    # Initialize publisher for desired velocity
    pub_vel = rospy.Publisher(desired_vel_topic, Twist, queue_size=1)

    # Driver to control the gripper
    gripper = GripperController(gripper_command_topic)

    # Sleep to make sure controllers are setup
    rospy.sleep(3)


    pub_pose.publish(Pose(position = Point(0.7, 0.2, 0.7),
                          orientation = Quaternion(0, 1, 0, 0)))



    p0 = np.array([0.7, 0.2, 0.7])      # Initial position
    p1 = np.array([0.5, -0.5, 0.4])     # Pick position
    p2 = np.array([-0.7, -0.2, 0.3])    # Place position


    move(p0, p1, 20, 5)


    # == Pick up the cube == #
    # This is made in 3 steps:
    #   1. lower the arm so that the cube is inside the gripper
    #   2. close the gripper
    #   3. bring the arm up again in the original position

    pub_pose.publish(Pose(position = Point(p1[0], p1[1], 0.19),
                          orientation = Quaternion(0, 1, 0, 0))
    )
    rospy.sleep(2)

    gripper.close()
    rospy.sleep(2)

    pub_pose.publish(Pose(position = Point(*tuple(p1)),
                          orientation = Quaternion(0, 1, 0, 0))
    )
    rospy.sleep(2)


    move(p1, p2, 20, 10)
