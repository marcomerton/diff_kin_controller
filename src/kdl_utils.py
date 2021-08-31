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


# ================ #
# Quaternion utils #
# ================ #

def quaternion_orientation_error(Qd, Qe):
    ''' Compute the orientation error (i.e. the relative rotation)
    between two quaternions '''
    e = Qe[1] * Qd[0] - Qd[1] * Qe[0] - Qd[0] * Qe[0]
    # e[0] = Qd.x * Qe.w + Qe.x * Qd.w + Qd.y * Qe.z - Qd.z * Qe.y
    # e[1] = Qd.y * Qe.w + Qe.y * Qd.w + Qd.z * Qe.x - Qd.x * Qe.z
    # e[2] = Qd.z * Qe.w + Qe.z * Qd.w + Qd.x * Qe.y - Qd.y * Qe.x
    
    return e

def quaternion_to_couple(Q):
    ''' Transform a Quaternion (geometry_msgs) or a tuple
    into a (kdl.Vector, scalar) couple
    '''
    if isinstance(Q, tuple) and len(Q) == 4:
        return (kdl.Vector(Q[0], Q[1], Q[2]), Q[3])
    else:
        return (kdl.Vector(Q.x, Q.y, Q.z), Q.w)


# ================== #
# KDL to Numpy utils #
# ================== #

def Jac_to_numpy(jac):
    ''' Convert a kdl.Jacobian into a np.ndarray '''

    j = np.ndarray(shape=(jac.rows(), jac.columns()))
    for i in range(jac.columns()):
        col = jac.getColumn(i)
        for k in range(6):
            j[k, i] = col[k]

    return j

def Twist_to_numpy(tw):
    ''' Convert a kdl.Twist into a np.ndarray '''

    a = np.ndarray(shape=(6))
    for i in range(6):
        a[i] = tw[i]
    
    return a
