
#from e_utils.ex_env import EnvVar
from gnss_ins_sim.xext.xcsc import prompt_cyan, prompt_green, prompt_red
import pandas as pd
import numpy as np


fs = 100
freq_low = 2
filter_order = 10

class GravityFilterSos(object):
    def __init__(self):
        from scipy import signal

        self.sos = signal.butter(filter_order, freq_low, 'hp', fs=fs, output='sos')

    def filter(self, acc_data_one, which_axis):
        from scipy import signal
        new_acc_data_one = signal.sosfilt(self.sos, acc_data_one)
        return new_acc_data_one
    
    def get_skip_num(self):
        return 200

class GravityFilterSos2(object):
    def __init__(self):
        from scipy import signal
        self.sos = signal.butter(filter_order, freq_low, 'hp', fs=fs, output='sos')

    def filter(self, acc_data_one, which_axis):
        from scipy import signal
        new_acc_data_one = signal.sosfiltfilt(self.sos, acc_data_one)
        return new_acc_data_one
    
    def get_skip_num(self):
        return 10

## see https://developer.android.com/reference/android/hardware/SensorEvent.html for how
class GravityFilterAndroid(object):
    def __init__(self):
        self.alpha = 0.8
        self.gravity = [0, 0, 9.81]

    def filter(self, acc_data_one, which_axis):
        new_acc_data_one = np.zeros((len(acc_data_one)))
        for i in range(len(acc_data_one)):
            self.gravity[which_axis] = self.alpha * self.gravity[which_axis] + (1 - self.alpha) * acc_data_one[i]
            new_acc_data_one[i] = acc_data_one[i] - self.gravity[which_axis]
        return new_acc_data_one

    def get_skip_num(self):
        return 30

class GravityFilterNone(object):
    def __init__(self):
        pass

    def filter(self, acc_data_one, which_axis):
        return acc_data_one

    def get_skip_num(self):
        return 0

def _gravity_in_name(gravity_in):
    if gravity_in == 0:
        return 'x'
    elif gravity_in == 1:
        return 'y'
    elif gravity_in == 2:
        return 'z'
    return None

def _build_df(acc_data, gyro_data):
    data = []
    for ad, wd in zip(acc_data, gyro_data):
        data.append([ad[0], ad[1], ad[2], wd[0], wd[1], wd[2]])
    np_data = np.array(data)
    df_data = pd.DataFrame(np_data, columns=["ax", "ay", "az", "wx", "wy", "wz"])
    return df_data

def _inspect_gravity(acc_data, gyro_data):
    df_data = _build_df(acc_data, gyro_data)
    np_data = df_data.values
    
    gravity_in = None
    gravity_mean = None
    np_data_ts = np_data.transpose(1, 0)
    for i in range(3):
        adm = np.mean(np_data_ts[i])
        if adm > 9.0 or adm < -9.0:
            if gravity_in is None:
                gravity_in = i
                gravity_mean = adm
            else:
                raise ValueError("found two gravity data in different angle?")

    return df_data, gravity_in, gravity_mean


def get_gravity_filter():
    return GravityFilterAndroid()
    # gf = None
    # name = EnvVar.get_grv_filter_name()
    # if name == "sos":  # this is what we have in the past, not a very sharp cut
    #     gf = GravityFilterSos()
    # elif name == "sos2":  # this has extermely goot sharp cut, but not easy to implment in running env
    #     gf = GravityFilterSos2()
    # elif name in ["android", "default"]:  # andorid has easy implementation, not good as sos2, but better than sos, and is very simple
    #     gf = GravityFilterAndroid()
    # elif name == "none":
    #     gf = GravityFilterNone()
    # else:
    #     raise ValueError("unknown_type" + name)
    # return gf
    

def filter_gravity_in_acc(acc_data, gyro_data, has_gravity=[], src=None):
    df_data, gravity_in, gravity_mean = _inspect_gravity(acc_data, gyro_data)
    prompt_cyan("orginal acc/gyro")
    print(df_data.describe())
    if gravity_in is None:
        print("### FGIA: no gravity found in acc, return original one")
        return acc_data
    
    print("### FGIA: find gravity in {} val {} / from src: {}, has_gravity: {}".format(_gravity_in_name(gravity_in), gravity_mean, src, has_gravity))
    if gravity_in != 2:
        prompt_red("gravity is not align with z? but in {}".format(_gravity_in_name(gravity_in)))

    gf = get_gravity_filter()

    acc_data_ts = acc_data.transpose(1, 0)

    filtered = 0
    new_acc_data = []
    acc_x = acc_data_ts[0]
    if (0 in has_gravity or len(has_gravity) == 0) and gravity_in == 0:
        #new_acc_x = signal.sosfilt(sos, acc_x)
        new_acc_x = gf.filter(acc_x, 0)
        filtered += 1
    else:
        new_acc_x = acc_x

    acc_y = acc_data_ts[1]
    if (1 in has_gravity or len(has_gravity) == 0) and gravity_in == 1:
        #new_acc_y = signal.sosfilt(sos, acc_y)
        new_acc_y = gf.filter(acc_y, 1)
        filtered += 1
    else:
        new_acc_y = acc_data_ts[1]

    acc_z = acc_data_ts[2]
    if (2 in has_gravity or len(has_gravity) == 0) and gravity_in == 2:
        #new_acc_z = signal.sosfilt(sos, acc_z)
        new_acc_z = gf.filter(acc_z, 2)
        filtered += 1
    else:
        new_acc_z = acc_z   

    acc_data_ts[0] = new_acc_x
    acc_data_ts[1] = new_acc_y
    acc_data_ts[2] = new_acc_z

    new_acc_data = acc_data_ts.transpose(1, 0)

    df_new = _build_df(new_acc_data, gyro_data)
    prompt_cyan("filtered acc/gyro")
    print(df_new.describe())    
    if filtered != 0:
        prompt_green("### FGIA: orginal acc/gyro after filter gravity, number of axies filtered: ", filtered)
    else:
        prompt_red("### FGIA: orginal acc/gyro after filter gravity, number of axies filtered: ", filtered)

    return new_acc_data
