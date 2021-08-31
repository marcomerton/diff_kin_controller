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

def interpolate_point(t, p0, p1):
    ''''''
    return p1 * t + p0 * (1 - t)


def slerp(t, P, Q):
    ''''''
    cos_half_theta = P.w * Q.w + P.x * Q.x + P.y * Q.y + P.z * Q.z
    if abs(cos_half_theta) >= 1:
        return P

    half_theta = np.arccos(cos_half_theta)

    sin_half_theta = np.sqrt(1 - cos_half_theta * cos_half_theta)
    ratioP = np.sin((1 - t) * half_theta) / sin_half_theta
    ratioQ = np.sin(t * half_theta) / sin_half_theta

    res = Quaternion()
    res.w = P.w * ratioP + Q.w * ratioQ
    res.x = P.x * ratioP + Q.x * ratioQ
    res.y = P.y * ratioP + Q.y * ratioQ
    res.z = P.z * ratioP + Q.z * ratioQ

    return res


def move(start_p, start_o, end_p, end_o, nsteps, time):

    # Compute (constant) velocity
    d = start_p - end_p
    tw = Twist(linear=Vector3(d[0], d[1], d[2]), angular=Vector3(0, 0, 0))

    # Start following trajectory
    rate = rospy.Rate(nsteps/time * 0.8)
    for step in range(1, nsteps+1):
        t = step/nsteps

        p = interpolate_point(t, start_p, end_p)
        q = slerp(t, start_o, end_o)

        pose = Pose(position = Point(*tuple(p)), orientation = q)

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

    # Initialize publishers
    pub_pose = rospy.Publisher(desired_pose_topic, Pose, queue_size=1)
    pub_vel = rospy.Publisher(desired_vel_topic, Twist, queue_size=1)

    # Driver to control the gripper
    gripper = GripperController(gripper_command_topic)

    # Sleep to make sure controllers are setup
    rospy.sleep(3)


    # Initial position and orientation
    p0 = np.array([0.7, 0.2, 0.7])
    q0 = Quaternion(0, 0, 0, 1)

    # Pick position and orientation
    p1 = np.array([0.5, -0.505, 0.4])
    q1 = Quaternion(0, 1, 0, 0)

    # Place position and orientation
    p2 = np.array([-0.7, -0.2, 0.3])
    q2 = Quaternion(1, 0, 0, 0)


    pub_pose.publish(Pose(position = Point(*tuple(p0)), orientation = q0))
    rospy.sleep(1)


    move(p0, q0, p1, q1, 30, 5)


    # == Pick up the cube == #
    # This is made in 3 steps:
    #   1. lower the arm so that the cube is inside the gripper
    #   2. close the gripper
    #   3. bring the arm up again in the original position
    pick_position = np.array([p1[0], p1[1], 0.195])

    move(p1, q1, pick_position, q1, 5, 1)
    rospy.sleep(1)

    gripper.close()
    rospy.sleep(1)

    move(pick_position, q1, p1, q1, 5, 1)
    rospy.sleep(2)


    move(p1, q1, p2, q2, 40, 5)
