import pandas as pd
import yaml
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.animation import FuncAnimation

def calculate_total_torque_per_row(df, cp_locations, joint_origin):
    print("cp_locations: ", cp_locations)
    print("joint_origin: ", joint_origin)
    torques = []
    for index, row in df.iterrows():
        total_torque = np.array([0.0, 0.0, 0.0])
        for i,cp in enumerate(cp_locations):
            # Calculate force vector for each center pressure point
            # Placeholder calculations
            r = np.array([cp['x'], cp['y'], cp['z']])
            F = np.array([row['rw_cp_force_vector_x_' + str(i)], row['rw_cp_force_vector_y_' + str(i)], row['rw_cp_force_vector_z_' + str(i)]])
            torque = np.cross(r - joint_origin, F)
            total_torque += torque
        torques.append(total_torque)
        # print(f"Row {index}: Total Torque = {total_torque}")
        # print(f"Row {index}: Total Torque = {total_torque}")
    return np.array(torques)

# Plotting functions
def plot_torque_over_time(torques, time):
    fig, axs = plt.subplots(3, 1, figsize=(10, 15))
    axs[0].plot(time, torques[:, 0], label='Torque X')
    axs[1].plot(time, torques[:, 1], label='Torque Y')
    axs[2].plot(time, torques[:, 2], label='Torque Z')
    
    for ax in axs:
        ax.legend()
        ax.grid(True)
        ax.set_xlabel('Time')
        ax.set_ylabel('Torque')

    axs[0].set_title('Torque in X direction over Time')
    axs[1].set_title('Torque in Y direction over Time')
    axs[2].set_title('Torque in Z direction over Time')

    plt.tight_layout()
    plt.savefig('rw_torque_3dir.png')
    plt.show()

def update_graph(num, data, line):
    line.set_data(data[:2, :num])
    line.set_3d_properties(data[2, :num])
    return line,

def animate_torque_vectors(torques):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    data = np.array(torques).T  # Transpose to fit the update_graph function's requirements

    line, = ax.plot(data[0, 0:1], data[1, 0:1], data[2, 0:1])

    ani = FuncAnimation(fig, update_graph, len(time), fargs=(data, line),
                        interval=100, blit=False)
    ani.save('rw_torque_animation.gif', writer='Pillow', fps=60)
    plt.show()

# Uncomment the following line to run the animation (adjust as necessary)

# Main script   
csv_filename = '../data/aerodynamics_data.csv'
yaml_filename = '../aerodynamics_config.yaml'

csv_data = pd.read_csv(csv_filename)
with open(yaml_filename, 'r') as file:
    yaml_data = yaml.safe_load(file)

joint_origin = [0, 0, 0]  # Placeholder, adjust as necessary

torques = calculate_total_torque_per_row(csv_data, yaml_data['center_pressure_body_rw'], joint_origin)
time = csv_data['time']

plot_torque_over_time(torques, time)
animate_torque_vectors(torques)
