import os
import numpy as np
import pandas as pd

current_dir = os.path.dirname(os.path.abspath(__file__))
def max_vel_cost():
    csv_path = os.path.join(current_dir, '..', 'opt_data', 'four_bar_vel_end.csv')
    df = pd.read_csv(csv_path)
    df = df[(df['time'] >= 1.0) & (df['time'] <= 5.0)]
    print("Calculating cost")
    # print(df.head())
    # print(df.columns)
    avg_vz = df['vz'].mean()
    max_vz = df['vz'].max()
    
    df['speed'] = np.sqrt(df['vx']**2 + df['vy']**2 + df['vz']**2)
    avg_speed = df['speed'].mean()
    max_speed = df['speed'].max()
    
    print('Average vz: ', avg_vz)
    print('Max vz: ', max_vz)
    print('Average speed: ', avg_speed)
    print('Max speed: ', max_speed)
    
    #cost function optimize based on z vel max
    
    return -avg_vz