#!/usr/bin/python3
import numpy as np

import rospy
from geometry_msgs.msg import Pose, Point, Quaternion, Twist, Vector3
from utils.quaternion import *

from GripperController import GripperController


def interpolate_point(t, p0, p1):
    ''' Linear interpolation between two points '''
    return p1 * t + p0 * (1 - t)


def slerp(t, P, Q):
    ''' Spherical Linear Interpolation between two quaternions '''
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


def move(start_p, start_o, end_p, end_o, time, nsteps=10):
    ''' Move the arm from pose (start_p, start_o) to pose (end_p, end_o) in
    'time' seconds. The movement is executed linearly interpolating between
    the two poses 'nsteps' points. The generated profiles follow a sinuosidal
    curve. '''
    rate = rospy.Rate(nsteps/time)
    delta_t = time/nsteps

    p_old = start_p
    o_old = start_o
    for step in range(1, nsteps+1):
        t = (1 + np.sin(np.pi * step/nsteps - np.pi/2)) / 2

        # Compute pose
        p = interpolate_point(t, start_p, end_p)
        o = slerp(t, start_o, end_o)
        pose = Pose(position = Point(*tuple(p)), orientation = o)

        # Compute velocity
        vel = (p - p_old) / delta_t
        omega = get_angular_velocity(o_old, o, delta_t)
        tw = Twist(linear=Vector3(vel[0], vel[1], vel[2]), angular=Vector3(*omega))

        # Publish commands
        pub_pose.publish(pose)
        pub_vel.publish(tw)


        p_old = p
        o_old = o

        rate.sleep()
    
    # Reset desider velocity to zero
    pub_vel.publish(Twist())

    # Wait for the arm to stabilize
    rospy.sleep(0.5)


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
    rospy.sleep(2)


    p_start = np.array([0, 0, 1.126])
    o_start = Quaternion(0, 0, 0, 1)

    pick_trajectory = rospy.get_param('pick_trajectory')
    place_trajectory = rospy.get_param('place_trajectory')

    # Reach pick position
    for v in pick_trajectory:
        p_dest = np.array(v['position'])
        o_dest = Quaternion(*tuple(v['orientation']))

        move(p_start, o_start, p_dest, o_dest, v['time'], v['nsteps'])

        p_start = p_dest
        o_start = o_dest

    # Pick the cube
    gripper.close()
    rospy.sleep(1)

    # Reach place positon
    for v in place_trajectory:
        p_dest = np.array(v['position'])
        o_dest = Quaternion(*tuple(v['orientation']))

        move(p_start, o_start, p_dest, o_dest, v['time'], v['nsteps'])

        p_start = p_dest
        o_start = o_dest

    # Place the cube
    gripper.open()
    rospy.sleep(1)
