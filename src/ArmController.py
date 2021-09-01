#!/usr/bin/python3
import sys
import numpy as np

import rospy
import PyKDL as kdl
import kdl_parser_py.urdf as kdl_parser

from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Pose, Quaternion, Twist

from kdl_utils import *
from quaternion_utils import orientation_error


class ArmController:
    ''' Class implementing a differential inverse kinematic controller
    for an arm like the kuka iiwa.
    The control algorithm is the one presented in: Bruno Siciliano et al,
    "Robotics: Modelling, Planning and Control", pp 132-134.
    '''

    def __init__(self,
                arm_chain,          # kdl.Chain representing the arm
                desired_pose_topic, # topic name where the desided pose is published
                desired_vel_topic,  # topic name where the desired velocity is published
                state_topic,        # topic name where the joints position is published
                command_topic,      # topic name where to publish commands for the arm
                rate = 200,         # rate at which the controller is executed
                eps = 6e-2          # tolerance for the termination
        ):

        # Setup forward kinematics and jacobian solvers
        self._fk_solver_pos = kdl.ChainFkSolverPos_recursive(arm_chain)
        self._jac_solver = kdl.ChainJntToJacSolver(arm_chain)

        # Initialize state variables and utility variables
        self._joint_position = None
        self._rate = rate

        # Setup current joint position subscriber
        def jnt_pos_callback(state):
            self._joint_position = new_JntArray(state.position[2:])
        rospy.Subscriber(state_topic, JointState, jnt_pos_callback)

        # Setup the command publisher
        self._command_publisher = rospy.Publisher(command_topic, Float64MultiArray, queue_size=10)

        # Wait to receive initial position
        while(self._joint_position == None):
            pass

        # Set desired pose to current pose
        x_e = self._compute_fk(self._joint_position)
        self._x_p = x_e.p
        self._x_o = Quaternion(*x_e.M.GetQuaternion())

        # Set desired velocity to zero
        self._xdot = kdl.Twist()

        # Setup desired ee pose listener
        def des_pose_callback(pose):
            self._x_p = point_to_kdl_vector(pose.position)
            self._x_o = pose.orientation
        rospy.Subscriber(desired_pose_topic, Pose, des_pose_callback)

        # Setup desired velocity listener
        def des_vel_callback(twist):
            self._xdot = twist_to_kdl_twist(twist)
        rospy.Subscriber(desired_vel_topic, Twist, des_vel_callback)


    def run(self, verbose=False):
        ''' Starts the arm controller '''

        rate = rospy.Rate(self._rate)
        while True:

            # == Fix joints position ==
            curr_jnt_pos = kdl.JntArray(self._joint_position)

            # == Compute end effector pose with forward kinematics ==
            x_e = self._compute_fk(curr_jnt_pos)

            ### Should publish this somewhere


            # == Compute position and orientation error ==
            e = self._compute_error(x_e)

            if verbose:
                rospy.loginfo(f"Current error:{e}\n")

            # == Compute joint velocities ==
            err = self._xdot + e * (self._rate/4)        # v_e = K * e
            q_dot = self._compute_ik(curr_jnt_pos, err)

            # == Integrate current position with velocity and send new position ==
            kdl.Multiply(q_dot, 1/self._rate, q_dot)      # q_dot *= 1/Ts
            kdl.Add(curr_jnt_pos, q_dot, curr_jnt_pos)    # curr_jnt_pos += q_dot

            self._send_command(curr_jnt_pos)


            rate.sleep()


    def _compute_fk(self, jnt_pos):
        ''' Compute the forward kinematics for a given joints position '''

        ee_frame = kdl.Frame()
        if self._fk_solver_pos.JntToCart(jnt_pos, ee_frame) != 0:
            rospy.logerr("Forward kinematics returned with error")

        return ee_frame


    def _compute_error(self, ee_frame):
        ''' Compute the error between the desired ee pose and the current ee pose '''

        e_p = self._x_p - ee_frame.p

        Qe = Quaternion(*ee_frame.M.GetQuaternion())
        e_o = kdl.Vector(*orientation_error(self._x_o, Qe))

        return kdl.Twist(e_p, e_o)


    def _compute_ik(self, jnt_pos, v_e, rho=0.001):
        ''' Compute the inverse differential kinematics using the
        SVD decomposition of the Jacobian with damped singular values

        J = USV' => pinv(J) = V * [diag(S) / (diag(S) ** 2 + rho)] * U'
        '''

        # Compute the jacobian
        jac = kdl.Jacobian(jnt_pos.rows())
        if self._jac_solver.JntToJac(jnt_pos, jac) != 0:
            rospy.logerr("Jacobian solver returned with error")

        # Convert everything to numpy
        jac = Jac_to_numpy(jac)
        v_e = Twist_to_numpy(v_e)

        # Compute SVD of the jacobian
        U, s, Vh = np.linalg.svd(jac, full_matrices=False)

        # Invert U and V matrices
        U = U.transpose()
        Vh = Vh.transpose()

        # Invert S matrix and add damping
        ss = s * s
        ss += rho
        s /= ss

        # Compute joints velocities
        q_dot = Vh @ (s * (U @ v_e))

        return new_JntArray(q_dot)


    def _send_command(self, jnt_pos, verbose=False):
        ''' Send a command to set the joints position '''

        command = Float64MultiArray()
        command.data = [x for x in jnt_pos]

        if verbose:
            rospy.loginfo(f"Sending: {command.data}")

        self._command_publisher.publish(command)



if __name__ == "__main__":
    
    rospy.init_node("kuka_controller")

    # Load the robot structure from urdf file
    urdf_file = sys.argv[1]
    res, tree = kdl_parser.treeFromFile(urdf_file)
    if not res:
        rospy.logerr(f"Couldn't load tree from file\n{urdf_file}")
        sys.exit(1)

    # Get arm chain
    chain = tree.getChain("iiwa_link_0", "iiwa_link_7")

    # Get topic names from paremeter server
    joint_command_topic = rospy.get_param("joint_command_topic")
    joint_state_topic = rospy.get_param("joint_state_topic")
    desired_pose_topic = rospy.get_param("desired_pose_topic")
    desired_vel_topic = rospy.get_param("desired_vel_topic")

    # Initialize controller
    c = ArmController(chain, desired_pose_topic, desired_vel_topic, joint_state_topic, joint_command_topic)

    # Start controler
    c.run(verbose=False)
