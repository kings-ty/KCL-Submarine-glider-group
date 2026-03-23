# V[:,:,N-1] = QT
# for n in range(N-2, -1, -1):
#     V[:,:,n] = Q + np.transpose(A)@(V[:,:,n+1] - V[:,:,n+1])@
#     @np.linalg.inv(np.transpose(B)@V[:,:,n+1]@B+R)
#     @np.transpose(B)
#     @V[:,:,n+1]@A

# for n in range(N-2, -1, -1):
#     L [: ,: , n ] = np . linalg . inv ( np . transpose ( B ) @V [: ,: , n +1] @B + R
# ) @np . transpose ( B ) @V [: ,: , n +1] @A
# return [L,V]

# V[:,:,N-1] = QT
# for n in range(N-2,-1,-1):
#     V [: ,: , n ] = Q + np.transpose(A) @ ( V [: ,: , n +1] - V [: ,: , n
# +1] @B@np . linalg . inv ( np . transpose ( B ) @V [: ,: , n +1] @B + R )
# @np . transpose ( B ) @V [: ,: , n +1]) @A
# for n in rnage(N-2,-1,-1)

# Use the following template and implment the Kalman filter update and predict steps as functions of
# the KalmanFilter class.

import numpy as np
import matplotlib.pyplot as plt


class KalmanFilter(object):
    def __init__(
        self, A=None, B=None, H=None, Sigmaw=None, Sigmav=None, P=None, x0=None
    ):
        if A is None or H is None:
            raise ValueError("System dynamics matrix A and observer matrix H needed.")

        self.n = A.shape[1]
        self.m = H.shape[0]

        self.A = A
        self.H = H
        self.B = 0 if B is None else B
        self.Sigmaw = np.eye(self.n) if Sigmaw is None else Sigmaw
        self.Sigmav = np.eye(self.m) if Sigmav is None else Sigmav
        self.P = np.eye(self.n) if P is None else P
        self.x = np.zeros((self.n, 1)) if x0 is None else x0

    def predict(self, u=0):
        # Predict state
        if np.isscalar(u) and u == 0:
            self.x = self.A @ self.x
        else:
            self.x = self.A @ self.x + self.B @ u

        # Predict covariance
        self.P = self.A @ self.P @ self.A.T + self.Sigmaw
        return self.x, self.P

    def update(self, z):
        # Innovation / residual
        y = z - self.H @ self.x

        # Innovation covariance
        S = self.H @ self.P @ self.H.T + self.Sigmav

        # Kalman gain
        K = self.P @ self.H.T @ np.linalg.inv(S)

        # Update state
        self.x = self.x + K @ y

        # Update covariance
        I = np.eye(self.n)
        self.P = (I - K @ self.H) @ self.P

        return self.x, self.P
