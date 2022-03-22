#!/usr/bin/env python3

""" 
    CLASS TO IMPLEMENT THE EXTENDED KALMAN FILTER TO TRACK OBJECTS
    THE SYSTEM TO ESTIMATE POSITION & VELOCITY IN 2D IS:

    P_x = P_x + dtV_x + w
    P_y = P_y + dtV_y + w
    V_x = V_x         + w
    V_y = V_y         + w
    
"""

import numpy as np
import itertools

class Kalman_Filter:

    new_id = itertools.count().__next__                 # GENERATES ID'S

    # INIT OBJECTS
    def __init__( self ):

        self.id = Kalman_Filter.new_id()
        self.delta_t = 1.0/20.0                         # SAMPLING TIME
        self.x = np.zeros((4, 1))                       # INITIAL SYSTEM
        self.P_k = np.identity(4)                       # SYSTEM UNCERTAINTY
        self.I_k = np.identity(4)                       # IDENTITY MATRIX 6x6   
        self.R_k = np.identity(2) * 0.01                # MOVEMENT NOISE
        self.Q_k = np.identity(4) * 0.1                 # SYSTEM NOISE
        self.H_k = np.array([                           # JACOBIAN OF H
            [1, 0, 0, 0],
            [0, 1, 0, 0]
        ])
        
        self.F_k = np.array([                           # JACOBIAN OF X SYSTEM
            [1, 0, self.delta_t, 0],
            [0, 1, 0, self.delta_t],
            [0, 0, 1, 0],
            [0, 0, 0, 1],
        ])

        
    
    # KALMAN FILTER METHOD
    def ekf(self, measure):
        # PREDICT
        x_hat = np.dot(self.F_k, self.x)                                                  # x' = F * x + U
        P_hat = np.dot(np.dot(self.F_k, self.P_k),  np.transpose(self.F_k)) + self.Q_k    # P' = F * P  * F_t + Q

        # UPDATE
        z = np.array([
            [measure[0]],
            [measure[2]]
        ])

        y = z - np.dot(self.H_k, x_hat)                                                   # y = z - H * x'
        S_k = np.dot(np.dot(self.H_k, P_hat), np.transpose(self.H_k)) + self.R_k          # S = H * P' * H_t + R
        K_k = np.dot(np.dot(P_hat, np.transpose(self.H_k)), np.linalg.inv(S_k))           # K = P' * H_t * S ^-1
        self.x = x_hat + np.dot(K_k, y)                                                   # x = x' + K * y
        self.P_k = np.dot(( self.I_k - np.dot(K_k, self.H_k)), P_hat)                     # P = (I -  k * H) * P'

        
