import PyKDL as kdl
import numpy as np


def new_JntArray(joints_pos):
    ''' Creates a new kdl.JntArray from the given joint positions '''

    j = kdl.JntArray(len(joints_pos))
    for i in range(len(joints_pos)):
        j[i] = joints_pos[i]
    
    return j

def to_string_JntArray(j):
    ''' Convert a kdl.JntArray into a string '''
    
    len = j.rows()
    s = "[ "
    for i in range(len):
        s += str(j[i])
        if i < len-1:
            s += "  "
    s += " ]"

    return s

def point_to_kdl_vector(point):
    ''' Transform a Point (geometry_msgs) into a kdl.Vector '''
    return kdl.Vector(point.x, point.y, point.z)

def twist_to_kdl_twist(twist):
    ''' Transform a Twist (geometry_msgs) into a kdl.Twist '''
    vel = kdl.Vector(twist.linear.x, twist.linear.y, twist.linear.z)
    rot = kdl.Vector(twist.angular.x, twist.angular.y, twist.angular.z)
    tw = kdl.Twist(vel, rot)
    return tw


# ================== #
# KDL to Numpy utils #
# ================== #

def Jac_to_numpy(jac):
    ''' Convert a kdl.Jacobian into a np.ndarray '''

    J = np.ndarray(shape=(jac.rows(), jac.columns()))
    for i in range(jac.columns()):
        col = jac.getColumn(i)
        for k in range(6):
            J[k, i] = col[k]

    return J

def Twist_to_numpy(tw):
    ''' Convert a kdl.Twist into a np.ndarray '''

    a = np.ndarray(shape=(6))
    for i in range(6):
        a[i] = tw[i]
    
    return a
