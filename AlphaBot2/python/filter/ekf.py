import numpy as np


class EKF:
    def __init__(self):
        # Initialize EKF parameters all units are in meters and radians
        self.x = np.zeros((5, 1)) #[x; y; x_dot; y_dot; theta]
        self.x_bar = np.zeros((5, 1))

        self.S = np.eye(5)
        self.S_bar = np.eye(5)

        self.Gx = np.eye(5)
        self.Gx[0,2] = 1
        self.Gx[1,3] = 1
        self.Gu = np.array([[0, 0],
                            [0, 0],
                            [np.cos(self.x[4,0]), 0],
                            [np.sin(self.x[4,0]), 0],
                            [0, 1]
                            ])

        self.R = np.eye(2)*0.2
        self.H = np.array([[1, 0, 0, 0, 0],
                           [0, 1, 0, 0, 0],
                           [0, 0, 0, 0, 1],
                           ])
        self.Q = np.eye(3)*0.1
        self.K = 1

    def prediction(self, u, dt):
        self.Gx[0, 2] = dt
        self.Gx[1, 3] = dt
        self.Gx[2, 2] = 1
        self.Gx[3, 3] = 1
        self.Gu[2, 0] = dt * np.cos(self.x_bar[4, 0])
        self.Gu[3, 0] = dt * np.sin(self.x_bar[4, 0])
        self.Gu[4, 1] = dt
        
        self.x_bar = self.Gx @ self.x_bar + self.Gu @ u
        self.S_bar = self.Gx @ self.S_bar @ self.Gx.T + self.Gu @ self.R @ self.Gu.T
        
        return self.x_bar


    def measurement_update(self, z):
        self.K = self.S_bar @ self.H.T @ np.linalg.inv(self.H @ self.S_bar @ self.H.T + self.Q)
        self.x = self.x_bar + self.K @ (z - self.H @ self.x_bar)
        self.S = (np.eye(3) - self.K @ self.H) @ self.S_bar
        self.S_bar = self.S
        self.x_bar = self.x
        return self.x
