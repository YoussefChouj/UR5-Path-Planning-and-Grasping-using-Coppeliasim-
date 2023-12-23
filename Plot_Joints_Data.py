import json
import pandas as pd
import matplotlib.pyplot as plt

def load_data(file_path):
    with open(file_path, 'r') as file:
        return json.load(file)

def plot_all_joint_data(data, path_index):
    data_types = ['positions', 'velocities', 'accelerations', 'torques']
    num_joints = 6

    for data_type in data_types:
        plt.figure(figsize=(15, 10))
        plt.suptitle(f'Joint Data ({data_type.capitalize()}) - Path {path_index}')

        for joint_number in range(1, num_joints + 1):
            joint_data = []
            time_data = []
            cumulative_time = 0

            for entry in data:
                if entry['index'] == path_index:
                    joint_data.append(entry[data_type][joint_number - 1])
                    cumulative_time += entry['time_step']
                    time_data.append(cumulative_time)

            plt.subplot(2, 3, joint_number)
            plt.plot(time_data, joint_data)
            plt.title(f'Joint {joint_number}')
            plt.xlabel('Time (s)')
            plt.ylabel(data_type.capitalize())
            plt.grid(True)

        # Adjust the layout
        plt.subplots_adjust(hspace=0.4, wspace=0.3)
        plt.show()

# Load data
open_path_file = 'C:\\Users\\youssef\\UR5_robot_machine_learning\\UR5_RG2\\openPathData.json'
close_path_file = 'C:\\Users\\youssef\\UR5_robot_machine_learning\\UR5_RG2\\closePathData.json'

open_data = load_data(open_path_file)
close_data = load_data(close_path_file)

# Example usage
path_index_to_plot = 2  # Change this to the desired path index

plot_all_joint_data(close_data, path_index_to_plot)
# To plot data for close paths, use: plot_all_joint_data(close_data, path_index_to_plot)
