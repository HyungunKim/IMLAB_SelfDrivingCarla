
  
import numpy as np
import pandas as pd
import math

REAR_WB = 1.3851432
FRONT_WB = -0.73497885
THROT_ACCEL = 3.5279548
BRAKE_ACCEL = 11.411816
STEER_GAIN = 0.2029629 
AERO_RESIST = 0.044634644
ROLL_RESIST = 0.839375
GRAV_RESIST = -2.2889855


def TimeTest(X, X_acts):
    dt = 0.1 # time step
    X_locs = X[:2]
    X_yaws = X[2:3]
    X_spds = X[3:4]
    
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