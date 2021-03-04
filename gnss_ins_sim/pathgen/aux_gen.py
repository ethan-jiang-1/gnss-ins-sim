import math
import numpy as np
#from ..attitude import attitude
from ..geoparams import geoparams
#from ..geoparams import geomag
from ..psd import time_series_from_psd

def acc_gen(fs, ref_a, acc_err, vib_def=None, vel_bias=None):
    """
    Add error to true acc data according to acclerometer model parameters
    Args:
        fs: sample frequency, Hz.
        ref_a: nx3 true acc data, m/s2.
        acc_err: accelerometer error parameters.
            'b': 3x1 acc constant bias, m/s2.
            'b_drift': 3x1 acc bias drift, m/s2.
            'vrw': 3x1 velocity random walk, m/s2/root-Hz.
        vib_def: Vibration model and parameters. Vibration type can be random, sinunoida or
            specified by single-sided PSD.
            Generated vibrating acc is expressed in the body frame.
            'type' == 'random':
                Normal distribution. 'x', 'y' and 'z' give the 1sigma values along x, y and z axis.
                units: m/s2
            'type' == 'sinunoidal'
                Sinunoidal vibration. 'x', 'y' and 'z' give the amplitude of the sine wave along
                x, y and z axis. units: m/s2.
            'type' == 'psd'. Single sided PSD.
                'freq':  frequency, in unit of Hz
                'x': x axis, in unit of m2/s4/Hz.
                'y': y axis, in unit of m2/s4/Hz.
                'z': z axis, in unit of m2/s4/Hz.
            'type' == 'vel'

    Returns:
        a_mea: nx3 measured acc data
    """
    dt = 1.0/fs
    # total data count
    n = ref_a.shape[0]
    ## simulate sensor error
    # static bias
    acc_bias = acc_err['b']
    # bias drift
    acc_bias_drift = bias_drift(acc_err['b_corr'], acc_err['b_drift'], n, fs)
    # vibrating acceleration
    acc_vib = np.zeros((n, 3))
    if vib_def is not None:
        if vib_def['type'].lower() == 'psd':
            acc_vib[:, 0] = time_series_from_psd.time_series_from_psd(vib_def['x'],
                                                                      vib_def['freq'], fs, n)[1]
            acc_vib[:, 1] = time_series_from_psd.time_series_from_psd(vib_def['y'],
                                                                      vib_def['freq'], fs, n)[1]
            acc_vib[:, 2] = time_series_from_psd.time_series_from_psd(vib_def['z'],
                                                                      vib_def['freq'], fs, n)[1]
        elif vib_def['type'] == 'random':
            acc_vib[:, 0] = vib_def['x'] * np.random.randn(n)
            acc_vib[:, 1] = vib_def['y'] * np.random.randn(n)
            acc_vib[:, 2] = vib_def['z'] * np.random.randn(n)
        elif vib_def['type'] == 'sinusoidal':
            acc_vib[:, 0] = vib_def['x'] * np.sin(2.0*math.pi*vib_def['freq']*dt*np.arange(n))
            acc_vib[:, 1] = vib_def['y'] * np.sin(2.0*math.pi*vib_def['freq']*dt*np.arange(n))
            acc_vib[:, 2] = vib_def['z'] * np.sin(2.0*math.pi*vib_def['freq']*dt*np.arange(n))
        else:
            raise ValueError("not_supported")

    # accelerometer white noise
    acc_noise = np.random.randn(n, 3)
    acc_noise[:, 0] = acc_err['vrw'][0] / math.sqrt(dt) * acc_noise[:, 0]
    acc_noise[:, 1] = acc_err['vrw'][1] / math.sqrt(dt) * acc_noise[:, 1]
    acc_noise[:, 2] = acc_err['vrw'][2] / math.sqrt(dt) * acc_noise[:, 2]
    # true + constant_bias + bias_drift + noise
    a_mea = ref_a + acc_bias + acc_bias_drift + acc_noise + acc_vib
    return a_mea

def gyro_gen(fs, ref_w, gyro_err, vel_bias=None):
    """
    Add error to true gyro data according to gyroscope model parameters
    Args:
        fs: sample frequency, Hz.
        ref_w: nx3 true acc data, rad/s.
        gyro_err: gyroscope error parameters.
            'b': 3x1 constant gyro bias, rad/s.
            'b_drift': 3x1 gyro bias drift, rad/s.
            'arw': 3x1 angle random walk, rad/s/root-Hz.
    Returns:
        w_mea: nx3 measured gyro data
    """
    dt = 1.0/fs
    # total data count
    n = ref_w.shape[0]
    ## simulate sensor error
    # static bias
    gyro_bias = gyro_err['b']
    # bias drift Todo: first-order Gauss-Markov model
    gyro_bias_drift = bias_drift(gyro_err['b_corr'], gyro_err['b_drift'], n, fs)
    # gyroscope white noise
    gyro_noise = np.random.randn(n, 3)
    gyro_noise[:, 0] = gyro_err['arw'][0] / math.sqrt(dt) * gyro_noise[:, 0]
    gyro_noise[:, 1] = gyro_err['arw'][1] / math.sqrt(dt) * gyro_noise[:, 1]
    gyro_noise[:, 2] = gyro_err['arw'][2] / math.sqrt(dt) * gyro_noise[:, 2]

    gyro_vel = np.zeros((n, 3))
    if vel_bias is not None:
        pass

    # true + constant_bias + bias_drift + noise
    w_mea = ref_w + gyro_bias + gyro_bias_drift + gyro_noise + gyro_vel
    return w_mea

def bias_drift(corr_time, drift, n, fs):
    """
    Bias drift (instability) model for accelerometers or gyroscope.
    If correlation time is valid (positive and finite), a first-order Gauss-Markov model is used.
    Otherwise, a simple normal distribution model is used.
    Args:
        corr_time: 3x1 correlation time, sec.
        drift: 3x1 bias drift std, rad/s.
        n: total data count
        fs: sample frequency, Hz.
    Returns
        sensor_bias_drift: drift of sensor bias
    """
    # 3 axis
    sensor_bias_drift = np.zeros((n, 3))
    for i in range(0, 3):
        if not math.isinf(corr_time[i]):
            # First-order Gauss-Markov
            a = 1 - 1/fs/corr_time[i]
            b = 1/fs*drift[i]
            #sensor_bias_drift[0, :] = np.random.randn(3) * drift
            drift_noise = np.random.randn(n, 3)
            for j in range(1, n):
                sensor_bias_drift[j, i] = a*sensor_bias_drift[j-1, i] + b*drift_noise[j-1, i]
        else:
            # normal distribution
            sensor_bias_drift[:, i] = drift[i] * np.random.randn(n)
    return sensor_bias_drift

def gps_gen(ref_gps, gps_err, gps_type=0):
    '''
    Add error to true GPS data according to GPS receiver error parameters
    Args:
        ref_gps: If gps_type is 0, [Lat, Lon, Alt, vx, vy, vz], [rad, rad, m].
                 If gps_type is 1, [x, y, z, vx, vy, vz], [m, m, m].
                 ref_gps data are expressed in the navigation frame.
        gps_err: GPS reeceiver parameters.
            'stdp': RMS position error, [m, m, m].
            'stdv': RMS velocity error, [m/s, m/s, m/s].
        gps_type: GPS data type.
            0: default, position is in the form of [Lat, Lon, Alt], rad, m
            1: position is in the form of [x, y, z], m
    Returns:
        gps_mea: ref_gps with error.
    '''
    # total data count
    n = ref_gps.shape[0]
    pos_err = gps_err['stdp'].copy()
    # If position is in the form of LLA, convert gps_err['stdp'] to LLA error
    if gps_type == 0:   # GPS is in the form of LLA, stdp meter to rad
        earth_param = geoparams.geo_param(ref_gps[0, 0:3])
        pos_err[0] = pos_err[0] / earth_param[0]
        pos_err[1] = pos_err[1] / earth_param[1] / earth_param[4]
    ## simulate GPS error
    pos_noise = pos_err * np.random.randn(n, 3)
    vel_noise = gps_err['stdv'] * np.random.randn(n, 3)
    gps_mea = np.hstack([ref_gps[:, 0:3] + pos_noise,
                         ref_gps[:, 3:6] + vel_noise])
    return gps_mea

def odo_gen(ref_odo, odo_err):
    '''
    Add error to true odometer data.
    Args:
        ref_odo: nx3, true odometer data, m/s.
        odo_err: odometer error profile.
            'scale': scalar, scale factor error.
            'stdv': scalar, RMS velocity error.
    Returns:
        odo_mea: nx1, measured odometer output.
    '''
    n = ref_odo.shape[0]
    odo_mea = np.random.randn(n)
    odo_mea = odo_err['scale']*ref_odo + odo_err['stdv']*odo_mea
    return odo_mea

def mag_gen(ref_mag, mag_err):
    """
    Add error to magnetic data.
    Args:
        ref_mag: nx3 true magnetic data, uT.
        mag_err: Magnetometer error parameters.
            'si': 3x3 soft iron matrix
            'hi': hard iron array, [ox, oy, oz], uT
            'std': RMS of magnetometer noise, uT
    Returns:
        mag_mea: ref_mag with error, mag_mea = si * (ref_mag + hi) + noise
    """
    # total data count
    n = ref_mag.shape[0]
    # add error
    mag_mea = ref_mag + mag_err['hi']
    mag_mea = mag_mea.dot(mag_err['si'].T)
    mag_noise = mag_err['std'] * np.random.randn(n, 3)
    return mag_mea + mag_noise
