# import
import math
import numpy as np
from ..attitude import attitude
from ..geoparams import geoparams
#from ..geoparams import geomag
#from ..psd import time_series_from_psd

def _calc_true_sensor_org(pos_n, vel_b, att, c_nb, vel_dot_b, att_dot, ref_frame, g):
    # velocity in N
    vel_n = c_nb.dot(vel_b)

    # Calculate rotation rate of n w.r.t e in n and e w.r.t i in n
    # For the NED frame, the NED frame rotation and Earth rotation rate is calculated
    # For the virtual inertial frame, they are not needed and simply set to zeros.
    w_en_n = np.zeros(3)
    w_ie_n = np.zeros(3)
    if ref_frame == 0:
        earth_param = geoparams.geo_param(pos_n)
        rm = earth_param[0]
        rn = earth_param[1]
        g = earth_param[2]
        sl = earth_param[3]
        cl = earth_param[4]
        w_ie = earth_param[5]
        rm_effective = rm + pos_n[2]
        rn_effective = rn + pos_n[2]
        gravity = np.array([0, 0, g])
        w_en_n[0] = vel_n[1] / rn_effective              # wN
        w_en_n[1] = -vel_n[0] / rm_effective             # wE
        w_en_n[2] = -vel_n[1] * sl /cl / rn_effective    # wD
        w_ie_n[0] = w_ie * cl
        w_ie_n[2] = -w_ie * sl
    else:
        gravity = [0, 0, g]

    # Calculate rotation rate of b w.r.t n expressed in n.
    # Calculate rotation rate from Euler angle derivative using ZYX rot seq.
    sh = math.sin(att[0])
    ch = math.cos(att[0])
    w_nb_n = np.zeros(3)
    w_nb_n[0] = -sh*att_dot[1] + c_nb[0, 0]*att_dot[2]
    w_nb_n[1] = ch*att_dot[1] + c_nb[1, 0]*att_dot[2]
    w_nb_n[2] = att_dot[0] + c_nb[2, 0]*att_dot[2]
    # Calculate rotation rate from rotation quaternion
    # w_nb_n = np.zeros(3)

    # Velocity derivative
    vel_dot_n = c_nb.dot(vel_dot_b) + attitude.cross3(w_nb_n, vel_n)
    # Position derivative
    pos_dot_n = np.zeros(3)
    if ref_frame == 0:
        pos_dot_n[0] = vel_n[0] / rm_effective      # Lat
        pos_dot_n[1] = vel_n[1] / rn_effective / cl # Lon
        pos_dot_n[2] = -vel_n[2]                    # Alt
    else:
        pos_dot_n[0] = vel_n[0]
        pos_dot_n[1] = vel_n[1]
        pos_dot_n[2] = vel_n[2]
    # Gyroscope output
    gyro = c_nb.T.dot(w_nb_n + w_en_n + w_ie_n)
    # Acceleration output
    w_ie_b = c_nb.T.dot(w_ie_n)
    acc = vel_dot_b + attitude.cross3(w_ie_b+gyro, vel_b) - c_nb.T.dot(gravity)
    return acc, gyro, vel_dot_n, pos_dot_n

def _calc_true_sensor_f1(pos_n, vel_b, att, c_nb, vel_dot_b, att_dot, ref_frame, g):
    if ref_frame == 0:
        raise ValueError("ref_frame should not be 0")

    # velocity in N
    vel_n = c_nb.dot(vel_b)

    # Calculate rotation rate of n w.r.t e in n and e w.r.t i in n
    # For the NED frame, the NED frame rotation and Earth rotation rate is calculated
    # For the virtual inertial frame, they are not needed and simply set to zeros.
    w_en_n = np.zeros(3)
    w_ie_n = np.zeros(3)
    gravity = [0, 0, g]

    # Calculate rotation rate of b w.r.t n expressed in n.
    # Calculate rotation rate from Euler angle derivative using ZYX rot seq.
    sh = math.sin(att[0])
    ch = math.cos(att[0])
    w_nb_n = np.zeros(3)
    w_nb_n[0] = -sh*att_dot[1] + c_nb[0, 0]*att_dot[2]
    w_nb_n[1] = ch*att_dot[1] + c_nb[1, 0]*att_dot[2]
    w_nb_n[2] = att_dot[0] + c_nb[2, 0]*att_dot[2]
    # Calculate rotation rate from rotation quaternion
    # w_nb_n = np.zeros(3)

    # Velocity derivative
    vel_dot_n = c_nb.dot(vel_dot_b) + attitude.cross3(w_nb_n, vel_n)
    # Position derivative
    pos_dot_n = np.zeros(3)
    pos_dot_n[0] = vel_n[0]
    pos_dot_n[1] = vel_n[1]
    pos_dot_n[2] = vel_n[2]

    # Gyroscope output
    gyro = c_nb.T.dot(w_nb_n + w_en_n + w_ie_n)

    # Acceleration output
    w_ie_b = c_nb.T.dot(w_ie_n)
    acc = vel_dot_b + attitude.cross3(w_ie_b+gyro, vel_b) - c_nb.T.dot(gravity)
    
    return acc, gyro, vel_dot_n, pos_dot_n


def calc_true_sensor_output(pos_n, vel_b, att, c_nb, vel_dot_b, att_dot, ref_frame, g):
    """
    Calculate true IMU results from attitude change rate and velocity
    change rate.
    attitude change rate is input in the form of Euler angle derivatives and
    converted into angular velocity. Velocity change rate is expressed in
    the body frame. Position change rate is also calculated. If simulation is
    done in the NED frame, the position change rate is in the form of Lat, Lon
    and alt derivatives. Otherwise, it is given in m/s.
    Args:
        pos_n: For NED, it is the absolute LLA position. Otherwise, it is relative
            motion.
        vel_b: Velocity in the body frame, m/s.
        att: Euler angles, [yaw pitch roll], rot seq is ZYX, rad.
        c_nb: Transformation matrix from b to n corresponding to att.
        vel_dot_b: Velocity change rate in the body frame, m/s/s
        att_dot: Euler angle change rate, [yaw_d, pitch_d, roll_d], rad/s
        ref_frame: See doc of function PathGen.
        g: Gravity, only used when ref_frame==1, m/s/s.
    Returns:
        [0]: 3x1 true accelerometer output in the body frame, m/s/s
        [1]: 3x1 true gyro output in the body frame, rad/s
        [2]: 3x1 velocity change rate in the navigation frame, m/s/s
        [3]: 3x1 position change rate in the navigation frame, m/s
    """
    #return _calc_true_sensor_org(pos_n, vel_b, att, c_nb, vel_dot_b, att_dot, ref_frame, g)
    return _calc_true_sensor_f1(pos_n, vel_b, att, c_nb, vel_dot_b, att_dot, ref_frame, g)
