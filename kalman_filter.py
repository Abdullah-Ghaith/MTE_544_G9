

import numpy as np



# TODO Part 3: Comment the code explaining each part
class kalman_filter:
    
    # TODO Part 3: Initialize the covariances and the states    
    def __init__(self, P,Q,R, x, dt):
        
        self.P=P
        self.Q=Q
        self.R=R
        self.x=x
        '''
            Robot state vector. [x,y,th,w,v,vdot]
        '''
        self.dt = dt
        


    # TODO Part 3: Replace the matrices with Jacobians where needed        
    def predict(self):

        self.A = self.jacobian_A() #This is the Jacobian of G
        self.C = self.jacobian_H() #This is the Jacobian of H
        
        self.motion_model()
        
        self.P= np.dot( np.dot(self.A, self.P), self.A.T) + self.R

    # TODO Part 3: Replace the matrices with Jacobians where needed
    def update(self, z):

        S=np.dot(np.dot(self.C, self.P), self.C.T) + self.Q
            
        kalman_gain=np.dot(np.dot(self.P, self.C.T), np.linalg.inv(S))
        
        surprise_error= z - self.measurement_model()
        
        self.x=self.x + np.dot(kalman_gain, surprise_error)
        self.P=np.dot( (np.eye(self.A.shape[0]) - np.dot(kalman_gain, self.C)) , self.P)
        
    
    # TODO Part 3: Implement here the measurement model
    # NOTE Completed.
    def measurement_model(self):
        x, y, th, w, v, vdot = self.x
        
        # NOTE Do we need to integrate v?
        # NOTE Are ax and ay correct or is the Jacobian wrong?
        return np.array([
            v,# v
            w,# w
            vdot, # ax
            v*w, # ay
        ])
        
    # TODO Part 3: Impelment the motion model (state-transition matrice)
    # NOTE Completed.
    def motion_model(self):
        
        x, y, th, w, v, vdot = self.x
        dt = self.dt
        
        # Numerically Integrate states based on linear acceleration and angular velocity. 
        self.x = np.array([
            x + v * np.cos(th) * dt,
            y + v * np.sin(th) * dt,
            th + w * dt,
            w,
            v  + vdot*dt,
            vdot,
        ])
        


    # TODO Part 3: Implement the jacobian of the A matrix (motion)
    def jacobian_A(self):
        x, y, th, w, v, vdot = self.x
        dt = self.dt
        
        return np.array([
            #x, y,               th,                  w,                    v,             vdot
            [1, 0,              -v * np.sin(th) * dt, 0,                   np.cos(th) * dt, 0],
            [0, 1,               v * np.cos(th) * dt, 0,                   np.sin(th) * dt, 0],
            [0, 0,               1,                   dt,                  0,               0],
            [0, 0,               0,                   1,                   0,               0],
            [0, 0,               0,                   0,                   1,               dt],
            [0, 0,               0,                   0,                   0,               1]
        ])
    
    
    # TODO Part 3: Implement here the jacobian of the H matrix (measurements)    
    def jacobian_H(self):
        x, y, th, w, v, vdot=self.x

        print(f"W:{w}")
        print(f"V:{v}")
        #x, y,th, w, v,vdot
        return np.array([
            [0,0,0  , 0, 1, 0], # x
            [0,0,0  , 1, 0, 0], # y
            [0,0,0  , 0, 0, 1], # ax
            [0,0,0  , v, w, 0] # ay
        ])
        
        # return np.array([
        #     v,# v
        #     w,# w
        #     vdot * np.cos(th), # ax
        #     vdot * np.sin(th), # ay
        # ])
        
    # TODO Part 3: return the states here    
    def get_states(self):
        return self.x
