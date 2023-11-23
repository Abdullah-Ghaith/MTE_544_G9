#Import necessary module
import numpy as np

# Define a class for the Kalman filter
class kalman_filter:
    
    # Initialize the Kalman filter with covariances, states, and time step
    def __init__(self, P,Q,R, x, dt):
        self.P=P  # State covariance
        self.Q=Q  # Process noise covariance
        self.R=R  # Measurement noise covariance
        self.x=x  # State vector
        self.dt = dt  # Time step

    # Predict the next state and update the state covariance
    def predict(self):
        self.A = self.jacobian_A()  # Jacobian of the state transition function
        self.C = self.jacobian_H()  # Jacobian of the measurement function
        self.motion_model()  # Update the state using the motion model
        self.P= np.dot( np.dot(self.A, self.P), self.A.T) + self.R  # Update the state covariance

    # Update the state and state covariance based on a new measurement
    def update(self, z):
        S=np.dot(np.dot(self.C, self.P), self.C.T) + self.Q  # Innovation covariance
        kalman_gain=np.dot(np.dot(self.P, self.C.T), np.linalg.inv(S))  # Kalman gain
        surprise_error= z - self.measurement_model()  # Measurement residual
        self.x=self.x + np.dot(kalman_gain, surprise_error)  # Update the state
        self.P=np.dot( (np.eye(self.A.shape[0]) - np.dot(kalman_gain, self.C)) , self.P)  # Update the state covariance

    # Define the measurement model
    def measurement_model(self):
        x, y, th, w, v, vdot = self.x
        return np.array([
            v,  # Velocity
            w,  # Angular velocity
            vdot,  # Linear acceleration
            v*w,  # Angular acceleration
        ])

    # Define the motion model
    def motion_model(self):
        x, y, th, w, v, vdot = self.x
        dt = self.dt
        self.x = np.array([
            x + v * np.cos(th) * dt,  # Update x position
            y + v * np.sin(th) * dt,  # Update y position
            th + w * dt,  # Update orientation
            w,  # Keep angular velocity
            v  + vdot*dt,  # Update velocity
            vdot,  # Keep linear acceleration
        ])

    # Define the Jacobian of the state transition function
    def jacobian_A(self):
        x, y, th, w, v, vdot = self.x
        dt = self.dt
        return np.array([
            #x, y,               th,                  w,                    v,             vdot
            [1, 0, -v * np.sin(th) * dt, 0, np.cos(th) * dt, 0],
            [0, 1,  v * np.cos(th) * dt, 0, np.sin(th) * dt, 0],
            [0, 0,  1,                   dt, 0,               0],
            [0, 0,  0,                   1,  0,               0],
            [0, 0,  0,                   0,  1,               dt],
            [0, 0,  0,                   0,  0,               1]
        ])

    # Define the Jacobian of the measurement function
    def jacobian_H(self):
        x, y, th, w, v, vdot=self.x
        return np.array([
            [0,0,0, 0, 1, 0],  # Partial derivatives of velocity
            [0,0,0, 1, 0, 0],  # Partial derivatives of angular velocity
            [0,0,0, 0, 0, 1],  # Partial derivatives of linear acceleration
            [0,0,0, v, w, 0]   # Partial derivatives of angular acceleration
        ])

    # Return the current state
    def get_states(self):
        return self.x
