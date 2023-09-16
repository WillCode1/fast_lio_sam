import pandas as pd
import numpy as np
import matplotlib.pyplot as plt


fastlio_log = pd.read_csv('./fast_lio_log.csv', delimiter=',')
timestamp = fastlio_log['timestamp']
preprocess_time = fastlio_log['preprocess time']
incremental_time = fastlio_log['incremental time']
search_time = fastlio_log['search time']
total_time = fastlio_log['total time']

# keyframe_pose = np.loadtxt('./keyframe_pose.txt', dtype=np.float32, delimiter=' ')
# keyframe_pose_optimized = np.loadtxt('./keyframe_pose_optimized.txt', dtype=np.float32, delimiter=' ')
# gnss_pose = np.loadtxt('./gnss_pose.txt', dtype=np.float32, delimiter=' ')
state_predict = np.loadtxt('./state_predict.txt', dtype=np.float32, delimiter=' ')
state_update = np.loadtxt('./state_update.txt', dtype=np.float32, delimiter=' ')


def fast_lio_time_log_analysis():
    plt.plot(timestamp, preprocess_time, label='preprocess_time')
    plt.plot(timestamp, incremental_time, label='incremental_time')
    plt.plot(timestamp, search_time, label='search_time')
    plt.plot(timestamp, total_time, label='total_time')

    # plt.xlim(0, 1000)
    # plt.ylim(0, 200)

    plt.xlabel('timestamp')
    plt.ylabel('cost times(ms)')
    plt.legend()
    plt.title('fast_lio_time_log_analysis')
    plt.show()


def system_state_analysis():
    fig, axs = plt.subplots(4, 2)
    lab_pre = ['', 'pre-x', 'pre-y', 'pre-z']
    lab_out = ['', 'out-x', 'out-y', 'out-z']
    a_pre = np.loadtxt('./state_predict.txt')
    a_out = np.loadtxt('./state_update.txt')
    time = a_pre[:, 0]
    axs[0, 0].set_title('imu_rpy')
    axs[1, 0].set_title('imu_xyz')
    axs[2, 0].set_title('imu_velocity')
    axs[3, 0].set_title('bg')
    axs[0, 1].set_title('ba')
    axs[1, 1].set_title('ext_R')
    axs[2, 1].set_title('ext_T')
    axs[3, 1].set_title('gravity')
    index = [0, 1, 2, 5, 6, 7, 8, 9]
    for i in range(1, 4):
        for j in range(8):
            axs[j % 4, j // 4].plot(time, a_pre[:, i + index[j] * 3], label=lab_pre[i])
            axs[j % 4, j // 4].plot(time, a_out[:, i + index[j] * 3], label=lab_out[i])
            # axs[j % 4, j // 4].plot(time, a_pre[:, i + j * 3], '.-', label=lab_pre[i])
    for j in range(8):
        axs[j % 4, j // 4].grid()
        axs[j % 4, j // 4].legend()
    plt.suptitle("system_state_analysis")
    plt.show()


def z_axis_drift_analysis():
    fig, axs = plt.subplots(2, 2)
    lab_out = ['', 'out-x', 'out-y', 'out-z']
    a_out = np.loadtxt('./state_update.txt')
    time = a_out[:, 0]
    axs[0, 0].set_title('imu_position')
    axs[1, 0].set_title('ba')
    # axs[2, 0].set_title('imu_velocity')
    axs[0, 1].set_title('imu_eular')
    axs[1, 1].set_title('bg')
    axs[0, 0].plot(time, a_out[:, 6], label='z-axis')
    axs[0, 1].plot(time, a_out[:, 1], label='roll')     # x-axis
    axs[0, 1].plot(time, a_out[:, 2], label='pitch')    # y-axis

    axs[1, 1].plot(time, a_out[:, 16], label=lab_out[1])
    axs[1, 1].plot(time, a_out[:, 17], label=lab_out[2])
    axs[1, 1].plot(time, a_out[:, 18], label=lab_out[3])

    axs[1, 0].plot(time, a_out[:, 19], label=lab_out[1])
    axs[1, 0].plot(time, a_out[:, 20], label=lab_out[2])
    axs[1, 0].plot(time, a_out[:, 21], label=lab_out[3])

    for j in range(4):
        axs[j % 2, j // 2].grid()
        axs[j % 2, j // 2].legend()
    plt.suptitle("z_axis_drift_analysis")
    plt.show()


# fast_lio_time_log_analysis()
# system_state_analysis()
z_axis_drift_analysis()
