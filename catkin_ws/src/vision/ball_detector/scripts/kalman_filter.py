#!/usr/bin/env python3

""" 
    Obtain from: https://github.com/hector-aviles/WebotsROSArticuloCocheAutonomoAppliedAI/blob/main/catkin_ws/src/eir2023/scripts/kalman_filter.py
    CLASS TO IMPLEMENT THE EXTENDED KALMAN FILTER TO TRACK OBJECTS
    THE SYSTEM TO ESTIMATE POSITION & VELOCITY IN 2D IS:

    P_x = P_x + dtV_x + w
    P_y = P_y + dtV_y + w
    V_x = V_x         + w
    V_y = V_y         + w
   
    z_x = P_x + 0P_y + 0Vx + 0Vy + w
    z_y = 0P_x + P_y + 0Vx + 0Vy + w
"""

import numpy as np

class EKF:

    # INIT OBJECTS
    def __init__( self, delta_t, Q, R ):
        self.delta_t = delta_t
        self.x_hat = np.zeros((4, 1))
        self.P_hat = np.identity(4)
        self.x = np.zeros((4, 1))                       # INITIAL SYSTEM
        self.P = np.identity(4)                       # SYSTEM UNCERTAINTY
        self.I = np.identity(4)                       # IDENTITY MATRIX 6x6   
        self.R = R                                    # SENSOR NOISE COV MATRIX
        self.Q = Q                                    # PROCESS NOISE COV MATRIX
        self.H = np.array([                           # JACOBIAN OF H
            [1, 0, 0, 0],
            [0, 1, 0, 0]
        ])
        
        self.F = np.array([                           # JACOBIAN OF F FUNCTION
            [1, 0, delta_t,    0   ],
            [0, 1,    0   , delta_t],
            [0, 0,    1   ,    0   ],
            [0, 0,    0   ,    1   ],
        ])

    def predict(self):
        self.x_hat = np.dot(self.F, self.x)
        self.P_hat = np.dot(np.dot(self.F, self.P), np.transpose(self.F)) + self.Q
        return self.x_hat
        
    def update(self, Z):
        z = np.array([
            [Z[0]],
            [Z[1]]
        ])
        e = z - np.dot(self.H, self.x_hat)                                   # y = z - H * x'
        S = np.dot(np.dot(self.H, self.P_hat), np.transpose(self.H)) + self.R    # S = H * P' * H_t + R
        K = np.dot(np.dot(self.P_hat, np.transpose(self.H)), np.linalg.inv(S))   # K = P' * H_t * S ^-1
        self.x = self.x_hat + np.dot(K, e)                                   # x = x' + K * y
        self.P = np.dot(( self.I - np.dot(K, self.H)), self.P_hat)               # P = (I -  k * H) * P'
        return self.x
    """
    # KALMAN FILTER METHOD
    def estimate(self, Z):
        # PREDICT
        x_hat = np.dot(self.F, self.x)                                      # x' = F * x + U
        P = np.dot(np.dot(self.F, self.P), np.transpose(self.F)) + self.Q  # P' = F * P  * F_t + Q

        # UPDATE
        z = np.array([
            [Z[0]],
            [Z[1]]
        ])

        e = z - np.dot(self.H, x_hat)                                   # y = z - H * x'
        S = np.dot(np.dot(self.H, P), np.transpose(self.H)) + self.R    # S = H * P' * H_t + R
        K = np.dot(np.dot(P, np.transpose(self.H)), np.linalg.inv(S))   # K = P' * H_t * S ^-1
        self.x = x_hat + np.dot(K, e)                                   # x = x' + K * y
        self.P = np.dot(( self.I - np.dot(K, self.H)), P)               # P = (I -  k * H) * P'
        return self.x
    """