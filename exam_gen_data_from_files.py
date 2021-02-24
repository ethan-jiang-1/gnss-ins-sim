# -*- coding: utf-8 -*-
# Filename: test_ins_sim.py

"""
Test ins_sim.
Created on 2018-04-23
@author: dongxiaoguang
"""

import os
import math
import numpy as np
from gnss_ins_sim.sim import imu_model
from gnss_ins_sim.sim import ins_sim

# globals
D2R = math.pi/180

motion_def_path = os.path.abspath('.//demo_motion_def_files//')
fs = 100.0          # IMU sample frequency
fs_gps = 10.0       # GPS sample frequency
fs_mag = fs         # magnetometer sample frequency, not used for now


ref_frame = 1

def get_ini_pos_vel_att(def_fname):
    ini_pos_vel_att = np.genfromtxt(motion_def_path+def_fname, delimiter=',', skip_header=1, max_rows=1)
    print("ini_pos_vel_att 0(deg)\n", ini_pos_vel_att)

    ini_pos_vel_att[0] = ini_pos_vel_att[0] * D2R
    ini_pos_vel_att[1] = ini_pos_vel_att[1] * D2R
    ini_pos_vel_att[6:9] = ini_pos_vel_att[6:9] * D2R
    # add initial states error if needed
    ini_vel_err = np.array([0.0, 0.0, 0.0]) # initial velocity error in the body frame, m/s
    ini_att_err = np.array([0.0, 0.0, 0.0]) # initial Euler angles error, deg
    ini_pos_vel_att[3:6] += ini_vel_err
    ini_pos_vel_att[6:9] += ini_att_err * D2R
    print("ini_pos_vel_att 1(rds)\n", ini_pos_vel_att)
    return ini_pos_vel_att

def get_imu(imu_type):
    #imu = imu_model.IMU(accuracy='mid-accuracy', axis=6, gps=False)
    # ethan' change
    if imu_type == "h":
        return imu_model.IMU(accuracy='high-accuracy', axis=6, gps=True, odo=True)
    elif imu_type == "m":
        return imu_model.IMU(accuracy='high-accuracy', axis=6, gps=True, odo=True)
    elif imu_type == "l":
        return imu_model.IMU(accuracy='high-accuracy', axis=6, gps=True, odo=True)
    else:
        raise ValueError("not-support")


def gen_base_data(def_fname, imu_type, data_dir, num_run):
    '''
    Generate data that will be used by test_gen_data_from_files()
    '''
    # imu model
    imu = get_imu(imu_type)

    # start simulation
    sim = ins_sim.Sim([fs, fs_gps, fs_mag],
                      motion_def_path+def_fname,
                      ref_frame=ref_frame,
                      imu=imu,
                      mode=None,
                      env=None,
                      algorithm=None)
    sim.run(num_run)
    # save simulation data to files
    sim_result = sim.results(data_dir)
    print(sim_result)
    #sim.plot(['ref_accel', 'accel', 'ref_gyro', 'gyro', 'ref_pos','ref_att_quat', 'ref_vel'])


def gen_intergration_data_from_files(def_fname, data_dir, output_dir, num_run):
    '''
    test data generation from files.
    '''
    #### start simulation
    #### Algorithm
    # Free integration in a virtual inertial frame
    from demo_algorithms import free_integration
    '''
    Free integration requires initial states (position, velocity and attitude). You should provide
    theses values when you create the algorithm object.
    '''
    ini_pos_vel_att = get_ini_pos_vel_att(def_fname)

    # create the algorith object
    # free_integration.FreeIntegration: Integrate gyro to get attitude, double integrate linear acceleration to get position.
    algo = free_integration.FreeIntegration(ini_pos_vel_att)

    #### start simulation
    sim = ins_sim.Sim([fs, 0.0, 0.0],
                      data_dir,
                      ref_frame=ref_frame,
                      imu=None,
                      mode=None,
                      env=None,
                      algorithm=algo,
                      gen_gt=True)
    # run the simulation for 1000 times
    sim.run_intergration(num_run)
    # generate simulation results, summary
    sim_result = sim.results(output_dir, err_stats_start=-1, gen_kml=True)
    print(sim_result)
    #sim.plot(['ref_accel', 'gt_accel', 'accel', 'ref_gyro', 'gt_gyro', 'gyro'])
    #sim.plot(['ref_att_quat', 'gt_att_quat', 'att_quat'])
    #sim.plot(['ref_pos', 'gt_pos', 'pos', 'ref_att_quat', 'gt_att_quat', 'att_quat'])
    #sim.plot(['ref_vel', 'gt_vel', 'vel'])


if __name__ == '__main__':
    #def_fname = "//motion_def-90deg_turn.csv"
    #def_fname = "//motion_def-ins.csv"
    #def_fname = "//motion_def-ins-ethan.csv"
    #def_fname = "//motion_def-90deg_turn.csv"

    movement_name = "smpfwd"
    num_run = 3
    imu_type = "l"

    def_fname = "//motion_def-x-{}.csv".format(movement_name)

    dir_of_logged_files = os.path.abspath('.//x_sim_output//tmp//')
    dir_of_output_files = os.path.abspath('.//x_sim_output//{}-{}_{}//'.format(movement_name, imu_type, num_run))

    import shutil
    if os.path.isdir(dir_of_logged_files):
        shutil.rmtree(dir_of_logged_files)
    gen_base_data(def_fname, imu_type, dir_of_logged_files, num_run)
    gen_intergration_data_from_files(def_fname, dir_of_logged_files, dir_of_output_files, num_run)
 