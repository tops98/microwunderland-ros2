import numpy as np
import matplotlib.pyplot as plt
from dataclasses import dataclass


class CVD_KF_Config:
    _P:np.ndarray
    _R:np.ndarray
    _Q:np.ndarray
    _H:np.ndarray
    _dim_x:int
    _dim_z:int
    
    def __init__(self,
        num_measurements:int,
        initial_uncertenty:np.ndarray,
        measurment_uncertenty:np.ndarray,
        process_noise:np.ndarray,
        observation_matrix:np.ndarray = None) -> None:

        if len(measurment_uncertenty) != num_measurements:
            raise ValueError(f"dimensions are not matching measurment_uncertenty ({len(measurment_uncertenty)}) != {num_measurements}")
        if len(initial_uncertenty) != num_measurements*3:
            raise ValueError(f"dimensions are not matching initial_uncertenty ({len(initial_uncertenty)}) != {num_measurements*3}")
        if len(process_noise) != num_measurements*3:
            raise ValueError(f"dimensions are not matching process_noise ({len(initial_uncertenty)}) != {num_measurements*3}")
        
        r_matrix = np.zeros((num_measurements, num_measurements,))
        for i,value in enumerate(measurment_uncertenty):
            r_matrix[i,i]=value**2

        self._R = r_matrix
        self._P = self._create_covarianz_matrix(initial_uncertenty)
        self._Q = self._create_covarianz_matrix(process_noise)

        if observation_matrix is None:
            self._H = self._create_observation_matrix(num_measurements,num_measurements*3)
        else:
            self._H = observation_matrix


    def _create_observation_matrix(self, num_measurements:int, num_states:int) -> np.ndarray:
        matrix = np.zeros((num_measurements,num_states))
        for i in range(num_measurements):
            matrix[i,i] = 1
        return matrix

    def _create_covarianz_matrix(self, array:np.ndarray) -> np.ndarray:

        offset = array.size // 3
        cov_matrix = np.zeros((array.size,array.size))
        for y in range(array.size):
            velocity_colum = (y+offset) % array.size
            accelaration_colum = (y+2*offset) % array.size
            
            cov_matrix[y,y] = array[y]**2
            cov_matrix[y,velocity_colum] = array[y]*array[velocity_colum] 
            cov_matrix[y,accelaration_colum] = array[y]*array[accelaration_colum]

        return cov_matrix
        

    @property
    def R(self) -> np.ndarray:
        return self._R
    @property
    def P(self) -> np.ndarray:
        return self._P
    @property
    def Q(self) -> np.ndarray:
        return self._Q
    @property
    def H(self) -> np.ndarray:
        return self._H
    @property
    def num_measurments(self) -> int:
        return self._R.shape[0]
    @property
    def num_state(self) -> int:
        return self._P.shape[0]




# Kalman Filter for constant velocity dynamics
class CVD_KalmanFilter:
    
    def __init__(self, config:CVD_KF_Config, initial_state:np.ndarray=None) -> None:
        if initial_state is not None:
            if initial_state.size % 3 != 0 or initial_state.size >= config.num_state*3:
                raise ValueError("invalid number of variables:\n State vector expects position, velocity and accelaration for every axis")
            self._X = initial_state
        else:
            self._X = np.zeros(config.num_state)
       # uncertainty in estimate
        self._P = config.P
        # uncertainty in measurments
        self._R = config.R
        # uncertainty in dynamic model aka. process noise
        self._Q = config.Q
        # observation matrix
        self._H = config.H

    def predict(self, dt:float) -> None:
        # create state transition matrix
        F = np.zeros((self._X.size, self._X.size))
        offset = self._X.size//3
        for i in range(offset): 
            # distance equation
            F[i][i] = 1
            F[i][i+offset] = dt
            F[i][i+2*offset] = 0.5*dt**2
            # velocity equation
            F[i+offset][i+offset] = 1
            F[i+offset][i+2*offset] = dt
            # accelaration
            F[i+2*offset][i+2*offset] = 1

        # propagate state through time
        self._X = F.dot(self._X)
        # propagate estimate uncertainty through time
        self._P = F.dot(self._P)+self._Q

    def update(self, measurments:np.ndarray) -> None:
        # calculate Kalman Gain
        ph_t = self._P.dot(self._H.transpose())
        pr = np.linalg.inv(self._H.dot(self._P).dot(self._H.transpose())+self._R)
        self._K = ph_t.dot(pr)

        # update state estimate
        HX = self._H.dot(self._X)
        self._X = self._X + self._K.dot(measurments - HX)
        # update uncertainty of estimate
        KH = self._K.dot(self._H)
        K_R_Kt = self._K.dot(self._R).dot(self._K.transpose())
        I = np.eye(self._X.size)
        IKH = I-KH
        self._P = IKH.dot(self._P).dot(IKH.transpose()) + K_R_Kt

