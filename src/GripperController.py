#!/usr/bin/python3
import rospy

from std_msgs.msg import Float64MultiArray


class GripperController:
    ''' Simple driver to control the gripper '''

    def __init__(self, command_topic):
        self._command_publisher = rospy.Publisher(command_topic, Float64MultiArray, queue_size=10)
   

    def close(self, verbose=False):
        ''' Close the gripper fingers '''
        command = Float64MultiArray()
        command.data = [0.06, 0.06]

        if verbose:
            rospy.loginfo(f"Sending: {command.data}")

        self._command_publisher.publish(command)


    def open(self, verbose=False):
        ''' Open the gripper fingers '''
        command = Float64MultiArray()
        command.data = [0, 0]

        if verbose:
            rospy.loginfo(f"Sending: {command.data}")

        self._command_publisher.publish(command)
