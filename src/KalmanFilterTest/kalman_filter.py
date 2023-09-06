import numpy as np
import matplotlib.pyplot as plt

# Kalman Filter for constant velocity dynamics
class CVD_KalmanFilter:

    
    def __init__(self, dim_x:int, dim_z:int, initial_state:np.ndarray=None) -> None:
        if initial_state is not None:
            if initial_state.size % 3 != 0:
                raise ValueError("invalid number of variables:\n State vector expects position, velocity and accelaration for every axis")
            self._X = initial_state
        else:
            self._X = np.zeros(dim_x)
        # uncertainty in estimate
        self._P = np.eye(dim_x)
        # uncertainty in measurments
        self._R = np.eye(dim_z)
        # uncertainty in model aka. process noise
        self._Q = np.eye(dim_x)
        # observation matrix
        self._H = np.ones((dim_z, dim_x))

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
        p = self._P.dot(self._H.transpose())
        pr = np.linalg.inv(self._H.dot(self._P).dot(self._H.transpose())+self._R)
        self._K = p.dot(pr)


        # update state estimate
        HX = self._H.dot(self._X)
        self._X = self._X + self._K.dot(measurments - HX)
        # update uncertainty of estimate
        KH = self._K.dot(self._H)
        K_R_Kt = self._K.dot(self._R).dot(self._K.transpose())
        I = np.eye(self._X.size)
        IKH = I-KH
        self._P = IKH.dot(self._P).dot(IKH.transpose()) + K_R_Kt

