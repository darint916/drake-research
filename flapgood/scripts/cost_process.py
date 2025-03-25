import os
import numpy as np
import pandas as pd
from util.message import Message

current_dir = os.path.dirname(os.path.abspath(__file__))
def max_vel_cost():
    csv_path = os.path.join(current_dir, '..', 'opt_data', 'four_bar_vel_end.csv')
    df = pd.read_csv(csv_path)
    df = df[(df['time'] >= 3.0) & (df['time'] <= 6.0)]
    
    print("Calculating cost")
    #split upstroke downstroke
    df_upstroke = df[df['vz'] > 0]
    df_downstroke = df[df['vz'] < 0]
    
    avg_upstroke = df_upstroke['vz'].mean() if not df_upstroke.empty else 0
    avg_downstroke = abs(df_downstroke['vz'].mean()) if not df_downstroke.empty else 0
    Message.debug('Average upstroke: ')
    Message.info(avg_upstroke)
    Message.debug('Average downstroke: ')
    Message.info(avg_downstroke)
    angle_csv_path = os.path.join(current_dir, '..', 'opt_data', 'four_bar_tip_angle.csv')
    df_angle = pd.read_csv(angle_csv_path)
    df_angle = df_angle[(df_angle['time'] >= 3.0) & (df_angle['time'] <= 6.0)]
    max_angle = df_angle['angle'].max()
    print('Max angle: ', max_angle)
    min_angle = df_angle['angle'].min()
    print('Min angle: ', min_angle)
    flap_range = max_angle - min_angle
    Message.info('Flap range: ')
    Message.debug(flap_range)
    
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
    # up_weight = 
    # cost = -()2 ,1, 1.5
    angle_weight = ((max_angle - 10) / 60) * avg_downstroke
    print('Angle weight: ', angle_weight)
    cost = avg_downstroke ** 2 / avg_upstroke
    print('stroke cost: ', cost)
    cost += angle_weight
    print('Total cost: ', cost)
    return (-cost, flap_range, avg_downstroke, avg_upstroke) 
# 60 30-70
# 
if __name__ == '__main__':
    max_vel_cost()