import matplotlib.pyplot as plt
import pandas as pd

class XEnv(object):
    inspect_data = True
    plot_data = True

    @classmethod
    def get_inspect_data(cls):
        return cls.inspect_data

    @classmethod
    def get_plot_data(cls):
        return cls.plot_data



class XPlot(object):
    @classmethod
    def plot_accel(cls, acc_data, figsize=(10, 10), title=None):
        df_acc = pd.DataFrame(acc_data, columns=["ax", "ay", "az"])
        print(df_acc.describe())

        fig = plt.figure(figsize=figsize)
        if title is not None:
            plt.title(title)
        fig.add_subplot(311)
        plt.plot(df_acc["ax"].values, label="ax")      
        plt.legend()
        fig.add_subplot(312)
        plt.plot(df_acc["ay"].values, label="ay")      
        plt.legend()
        fig.add_subplot(313)
        plt.plot(df_acc["az"].values, label="az") 
        plt.legend()
        plt.show()

    @classmethod
    def plot_gyro(cls, gyro_data, figsize=(10, 10), title=None):
        df_gyro = pd.DataFrame(gyro_data, columns=["wx", "wy", "wz"])
        print(df_gyro.describe())

        fig = plt.figure(figsize=figsize)
        if title is not None:
            plt.title(title)
        fig.add_subplot(311)
        plt.plot(df_gyro["wx"].values, label="wx")      
        plt.legend()
        fig.add_subplot(312)
        plt.plot(df_gyro["wy"].values, label="wy")      
        plt.legend()
        fig.add_subplot(313)
        plt.plot(df_gyro["wz"].values, label="wz") 
        plt.legend()
        plt.show()  