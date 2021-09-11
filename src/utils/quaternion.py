from geometry_msgs.msg import Quaternion


def euler_derivative(qs, qe, delta_t):
    ''' Compute the time approximate derivative of a
    quaternion using Euler method'''
    q_dot = Quaternion()
    q_dot.w = (qe.w - qs.w) / delta_t
    q_dot.x = (qe.x - qs.x) / delta_t
    q_dot.y = (qe.y - qs.y) / delta_t
    q_dot.z = (qe.z - qs.z) / delta_t

    return q_dot

def conjugate(q): 
    ''' Return the conjugate of the given quaternion '''
    q_star = Quaternion()
    q_star.w = q.w
    q_star.x = -q.x
    q_star.y = -q.y
    q_star.z = -q.z

    return q_star

def get_angular_velocity(qs, qe, delta_t):
    ''' Compute the angular velocity to move from orientation
    qs to orientation qe in delta_t '''
    q_dot = euler_derivative(qs, qe, delta_t)
    q_star = conjugate(qs)

    omega_hat_x = 2 * q_dot.x * q_star.x
    omega_hat_y = 2 * q_dot.y * q_star.y
    omega_hat_z = 2 * q_dot.z * q_star.z

    return (omega_hat_x, omega_hat_y, omega_hat_z)

def orientation_error(Qd, Qe):
    ''' Compute the orientation error (i.e. the relative rotation)
    between two quaternions '''
    ex = Qd.x * Qe.w - Qe.x * Qd.w - Qd.y * Qe.z + Qd.z * Qe.y
    ey = Qd.y * Qe.w - Qe.y * Qd.w - Qd.z * Qe.x + Qd.x * Qe.z
    ez = Qd.z * Qe.w - Qe.z * Qd.w - Qd.x * Qe.y + Qd.y * Qe.x
    
    return (ex, ey, ez)
