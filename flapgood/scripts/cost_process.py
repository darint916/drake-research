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
    
    #constraint fix
    csv_path = os.path.join(current_dir, '..', 'opt_data', 'four_bar_yellow_angle.csv')
    df = pd.read_csv(csv_path)
    df = df[(df['time'] >= 3.0) & (df['time'] <= 6.0)]
    df_angle_exceed = df[(df['angle'] > 80) | (df['angle'] < -80)]
    if not df_angle_exceed.empty:
        abs_angle_exceed = df_angle_exceed['angle'].abs()
        Message.warning('Angle exceed on C (yellow bar) timestamp counts: ')
        print(len(df_angle_exceed))
        print('highest exceed angle: ', df_angle_exceed['angle'].max())
        print('setting cost 1000 + angle')
        cost = 1000
        return (cost, flap_range, avg_downstroke, avg_upstroke)
        
    #E bar constraint
    csv_path = os.path.join(current_dir, '..', 'opt_data', 'four_bar_joint_de_angle.csv')
    df = pd.read_csv(csv_path)
    df = df[(df['time'] >= 3.0) & (df['time'] <= 6.0)]
    df_angle_exceed = df[(df['angle'] > 63) | (df['angle'] < -63)]
    if not df_angle_exceed.empty:
        abs_angle_exceed = df_angle_exceed['angle'].abs()
        Message.warning('Angle exceed on E (joint de) timestamp counts: ')
        print(len(df_angle_exceed))
        print('highest exceed angle: ', df_angle_exceed['angle'].max())

        print('setting cost 1000 + angle')
        cost = 1000
        return (cost, flap_range, avg_downstroke, avg_upstroke)

    #cost function optimize based on z vel max
    # up_weight = 
    # cost = -()2 ,1, 1.5
    zero_point = 30
    peak_weight = 0.4
    target = 75
    drop_off_factor = 2
    sigma = (target - zero_point) / drop_off_factor
    gaussian = np.exp(-0.5 * ((flap_range - target) / sigma) ** 2)
    slope = peak_weight / (target - zero_point)
    
    if flap_range > target and flap_range < target + (zero_point * 1.3):
        angle_weight = peak_weight
    elif flap_range >= zero_point:
        angle_weight = peak_weight * gaussian
    else:
        angle_weight = slope * (flap_range - zero_point)


    # angle_weight = max(-((max_angle - 120) ** 2) + 1, -1) * avg_downstroke  
    print('Angle weight: ', angle_weight)
    cost = avg_downstroke - (avg_upstroke / 9)
    print('stroke cost: ', cost)
    cost += angle_weight * avg_downstroke
    print('Total cost: ', cost)
    return (-cost, flap_range, avg_downstroke, avg_upstroke) 
# 60 30-70
# 
if __name__ == '__main__':
    max_vel_cost()