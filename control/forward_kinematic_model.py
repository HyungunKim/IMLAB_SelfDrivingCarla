import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import math
import matplotlib.pyplot as plt

REAR_WB = 1.3851432
FRONT_WB = -0.73497885
THROT_ACCEL = 3.5279548
BRAKE_ACCEL = 11.411816
STEER_GAIN = 0.2029629 
AERO_RESIST = 0.044634644
ROLL_RESIST = 0.839375
GRAV_RESIST = -2.2889855

def make_train_data(df):    
    df['yaw'] = df['yaw'] * 3.14159265359/180   # 180 -> radian

    npvx = df['vx'].values                      # calculate vel(speed)
    npvy = df['vy'].values
    npspds = np.sqrt(npvx**2 + npvy**2)
    df['spds'] = npspds
    
    # delete unnecessary data
    df.pop('z')
    df.pop('vz')
    df.pop('roll')
    df.pop('reverse')
    df['spds'] = npspds
    
    # necessary data
    x_df_locs = df[['x', 'y']]
    x_df_yaws = df[['yaw']]
    x_df_spds = df[['spds']]
    x_df_pich = df[['pitch']]
    x_df_acts = df[['steer','throttle','brake']]

    xlocs = x_df_locs.values
    xyaws = x_df_yaws.values
    xspds = x_df_spds.values
    xpich = x_df_pich.values
    xacts = x_df_acts.values
    
    X = np.hstack([xlocs, xyaws, xspds, xpich, xacts])

    return X

def TimeTest(X, Act_np, time):
    dt = 0.1 # time step
    idx = int(time*10)
    X_locs = X[:2]
    X_yaws = X[2:3]
    X_spds = X[3:4]
    
    X_acts = Act_np[idx][4:]
    X_pich = X_acts[0:1]
    steer = X_acts[1:2]
    throt = X_acts[2:3]
    brake = X_acts[3:4] 

    if (brake == 0):
        accel = THROT_ACCEL*throt
    else:
        accel = -BRAKE_ACCEL
    accel -= AERO_RESIST * X_spds + ROLL_RESIST + GRAV_RESIST * X_pich
    
    wheel = STEER_GAIN * steer
    beta = np.arctan(REAR_WB * np.tan(wheel)/(FRONT_WB + REAR_WB)) # Rad
        
    next_locs = X_locs + (X_spds * (np.concatenate([np.cos(X_yaws+beta), np.sin(X_yaws+beta)], -1) * dt))
    next_yaws = X_yaws + (X_spds * (np.sin(beta) * dt))/(REAR_WB)
    next_spds = X_spds + accel * dt
    
    if next_yaws > np.pi:
        next_yaws -= 2 * np.pi
    elif next_yaws < -np.pi:
        next_yaws += 2 * np.pi
    
    return np.concatenate([next_locs, next_yaws, np.maximum(np.zeros_like(next_spds), next_spds)], -1)


def forward(data):
    '''
    if data is numpy, the data.shape must be (time_sequence, 8)
    The eight elements must be (x, y, yaw, spds, pitch, steer, throttle, brake) in order.
    else if data is dataframe, data have (x, y, yaw, spds, pitch, steer, throttle, brake) by columns
    '''
    if type(data) == pd.core.frame.DataFrame:
        x = make_train_data(data)      # x - (x, y, yaw, spds, pitch, steer, throttle, brake)
    else:
        x = data
    
    dt = 0.1
    future_time = 5
    future_sequence = int(future_time/dt)

    data_length = len(data)
    predict_model = np.zeros((data_length-future_sequence,future_sequence,4))  # file_number, time_index, time_step, feature
    
    for j in range(data_length-future_sequence):
        time = j * 0.1   
        next_x = x[j]
        
        next_x = next_x[:4]
        
        for k in range(future_sequence):
            for p in range(4):
                predict_model[j][k][p] = next_x[p]  
            next_x = TimeTest(next_x, x, time+k*0.1)
    print(predict_model.shape)
    return predict_model
