import numpy as np

class VelBias(object):
    def __init__(self, odo, ini_vel):
        self.cur_odo = odo 
        self.ini_vel = ini_vel
        self.cur_odo_delta = np.zeros(self.cur_odo.shape)
        self.cur_odo_shift = np.zeros(self.cur_odo.shape)
        for i in range(len(self.cur_odo)):
            if i != 0:
                self.cur_odo_delta[i] = self.cur_odo[i] - self.cur_odo[i-1]
            self.cur_odo_shift[i] = self.cur_odo[i] - ini_vel
        print("odo_speed\t", self.cur_odo.mean(axis=0), self.cur_odo.var(axis=0))
        print("odo_delta\t", self.cur_odo_delta.mean(axis=0), self.cur_odo_delta.var(axis=0))
        print("odo_shift\t", self.cur_odo_shift.mean(axis=0), self.cur_odo_shift.var(axis=0))
        print("  ini_vel\t", self.ini_vel)
    
    def get_ini_vel(self):
        return self.ini_vel

    def get_vel_shift(self):
        return self.cur_odo_shift
