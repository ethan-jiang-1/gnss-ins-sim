from datetime import datetime
import numpy as np
import pandas as pd
import os
import sys
app_root = os.path.dirname(os.path.dirname(__file__))
app_root = os.path.dirname(app_root)
srcs = []
srcs.append(app_root)
for src in srcs:
    if src not in sys.path:
        sys.path.append(src)

from gnss_ins_sim.xext.ex_sdata_gravity import filter_gravity_in_acc

def _load_processed_data(path_name, route_name, route_num):
    print(path_name, route_name, route_num)

    fn_acc = "{}/accel-{}.csv".format(path_name, route_num)
    fn_gyro = "{}/gyro-{}.csv".format(path_name, route_num)
    fn_pos = "{}/pos-algo0_{}.csv".format(path_name, route_num)
    fn_ori = "{}/att_quat-algo0_{}.csv".format(path_name, route_num)

    df_acc = pd.read_csv(fn_acc)
    df_gyro = pd.read_csv(fn_gyro)
    df_pos = pd.read_csv(fn_pos)
    df_ori = pd.read_csv(fn_ori)

    print(df_acc)
    print(df_acc.describe())
    print(df_gyro)
    print(df_gyro.describe())
    print(df_pos)
    print(df_pos.describe())
    print(df_ori)
    print(df_ori.describe())

    acc_data = df_acc.values
    gyro_data = df_gyro.values
    pos_data = df_pos.values
    ori_data = df_ori.values 

    # len_data = len(acc_data)
    # if path_name.find("-ins") != -1:
    #     acc_data = acc_data[25000:55000,:]
    #     gyro_data = gyro_data[25000:55000,:]
    #     pos_data = pos_data[25000:55000,:]
    #     ori_data = ori_data[25000:55000,:]   

    len_data = len(acc_data)
    min_x = np.inf
    min_y = np.inf
    min_z = np.inf
    for i in range(len_data):
        gyro_data[i] = gyro_data[i] * (3.14159 / 180)
        min_x = min(pos_data[i][0], min_x)
        min_y = min(pos_data[i][1], min_y)
        min_z = min(pos_data[i][2], min_z)

    for i in range(len_data):
        pos_data[i][0] -= min_x
        pos_data[i][1] -= min_y
        pos_data[i][2] -= min_z

    acc_data = filter_gravity_in_acc(acc_data, gyro_data, src="sim_org_algo")

    ts_ref = []
    for ts in range(len(gyro_data)):
        ts_ref.append(ts/100)

    new_imu = []
    for ad, gd in zip(acc_data, gyro_data):
        new_imu.append([ad[0], ad[1], ad[2], gd[0], gd[1], gd[2]])

    new_gt = []
    for dpd, drd in zip(pos_data, ori_data):
        new_gt.append([dpd[0], dpd[1], dpd[2], drd[0], drd[1], drd[2], drd[3]])
    return ts_ref, new_gt, new_imu
    

def _save_to_csv(route_name, route_num, df_imu, df_gt):
    now = datetime.now()
    date_str = "{}-{:02d}-{:02d}".format(now.year, now.month, now.day)
    data_root = "ds_Sim/{}-{}_{}".format(date_str, route_name, route_num)
    print("saving {} ...", data_root)
    if not os.path.isdir(data_root):
        os.mkdir(data_root)    

    df_imu.to_csv(data_root + "/imu.csv")
    df_gt.to_csv(data_root + "/gt.csv")    

def _dump_to_new_location(route_name, route_num, ts_ref, gt_new, imu_new):
    print("\n\n")
    columns_gt = ["ts", "px", "py", "pz", "rw", "rx", "ry", "rz"]
    tsf = np.array(ts_ref)
    gt_data = []
    for ts, gtd in zip(tsf, gt_new):
        gt_data.append([ts, gtd[0], gtd[1], gtd[2], gtd[3], gtd[4], gtd[5], gtd[6]])
    df_gt = pd.DataFrame(gt_data, columns=columns_gt)
    print(df_gt)
    print(df_gt.describe())

    columns_imu = ["ts", "ax", "ay", "az", "wx", "wy", "wz"]
    imu_data = []
    for ts, msd in zip(tsf, imu_new):
        imu_data.append([ts, msd[0], msd[1], msd[2], msd[3], msd[4], msd[5]])        
    df_imu = pd.DataFrame(imu_data, columns=columns_imu)
    print(df_imu)
    print(df_imu.describe())

    _save_to_csv(route_name, route_num, df_imu, df_gt)


def do_split_sdir(name):
    print(name)
    path_name = "x_sim_output/" + name

    if os.path.isdir(path_name):
        nns = name.split("_")
        route_name = nns[0]
        route_nums = int(nns[1])
        for route_num in range(route_nums):
            ts_ref, gt_new, imu_new = _load_processed_data(path_name, route_name, route_num)
            _dump_to_new_location(route_name, route_num, ts_ref, gt_new, imu_new)


if __name__ == '__main__':
    names = ["smpfwd-l_3"]

    for name in names:
        do_split_sdir(name)

