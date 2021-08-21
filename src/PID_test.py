#!/usr/bin/python3
import sys
import rospy

from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

from random import uniform


joint_command_topic = "/iiwa/iiwa_pos_effort_controller/command"
joint_state_topic = "/iiwa/joint_states"


def run(n_runs, eps):
    # Setup ros node
    rospy.init_node('PID_tester')

    # Setup joints state listener
    joints_pos = None
    def joints_pos_callback(state):
        nonlocal joints_pos
        joints_pos = state.position[2:]

    rospy.Subscriber(joint_state_topic, JointState, joints_pos_callback)

    # Setup command publisher
    pub = rospy.Publisher(joint_command_topic, Float64MultiArray, queue_size=10)


    rate = rospy.Rate(0.5)
    for i in range(n_runs):
        # Generate new position
        command = Float64MultiArray()
        command.data = [uniform(-1, 1) for _ in range(7)]
        rospy.loginfo(f"({i}) Sending: {command.data}\n")

        # Send command
        pub.publish(command)

        # Sleep to wait convergence
        rate.sleep()

        # Check error
        err = [abs(d-e) for (d,e) in zip(command.data, joints_pos)]
        rospy.loginfo(f"Error: {err}\n")

        if max(err) > eps:
            rospy.logerr("Max error is too high!")


if __name__ == '__main__':

    n_runs = 20
    eps = 5e-2

    if len(sys.argv) > 1:
        if sys.argv[1] in ['-h', '--help']:
            print(f"Use: {sys.argv[0]} [n_runs] [epsilon]\n")
            print(f"\tn_runs   -- Number of iterations")
            print(f"\tepsilon  -- Precision to check")

            sys.exit(0)

        else:
            n_runs = int(sys.argv[1])

    if len(sys.argv) > 2:
        eps = float(sys.argv[2])

    run(n_runs, eps)
