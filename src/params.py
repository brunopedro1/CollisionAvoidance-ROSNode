"""
Instituto Superior Tecnico, University of Lisbon

Part of the Master`s Thesis in Aerospace Engineering:
"Development of a Sense and Avoid System for Small Fixed-wing UAV" 

Author: Bruno Pedro

2024
"""

import numpy as np


# Sensor parameters (body frame: x to front, y to right wing)

# Sonar 1 (MB1202, i2c_address=0x68, device_id=7497753)
POS_X_SONAR1 = 0.2 # meters
POS_Y_SONAR1 = 0.2 # meters
YAW_SONAR1 = 0 # degrees

# Sonar 2 (MB1242, i2c_address=0x70, device_id=7499801)
POS_X_SONAR2 = 0.2
POS_Y_SONAR2 = -0.2
YAW_SONAR2 = 0

# Laser 1 (LW20/c, i2c_address=0x66, device_id=7562777)
POS_X_LASER1 = 0.2
POS_Y_LASER1 = 0.1
YAW_LASER1 = 1

# Laser 2 (LW20/c, i2c_address=0x67, device_id=7563033)
POS_X_LASER2 = 0.2
POS_Y_LASER2 = -0.1
YAW_LASER2 = -1

# LIDAR (SF45, TELEM3, device_id=7536653)
POS_X_LIDAR = 0.2
POS_Y_LIDAR = 0


# Kalman filter tuning parameters

# Sonar
DT_SONAR = 0.1 
P_SONAR = 7
R_SONAR = 1
Q_SONAR = np.array([[1e-1, 0],
                    [0, 1e-1]])

# Laser
DT_LASER = 0.05
P_LASER = 20
R_LASER = 1
Q_LASER = np.array([[1e-1, 0],
                    [0, 1e-1]])
                    
                    
# Polar histogram parameters

MIN_ANGLE = -45 # degrees
MAX_ANGLE = 45
STEP_ANGLE = 10
GAMMA = 3  
TIME_CLEAN_BINS = 1 # seconds


# Avoidance parameters

THRESHOLD = 0.90
SETPOINT_STEP = 3

           
                    
