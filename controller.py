# Import necessary modules
from pid import PID_ctrl
import numpy as np
from utilities import euler_from_quaternion, calculate_angular_error, calculate_linear_error

# Define constant for PI
M_PI=3.1415926535

# Define constants for PID control types
P=0; PD=1; PI=2; PID=3

# Define a controller class
class controller:
    
    # Initialize the controller with PID parameters
    def __init__(self, klp=0.2, klv=0.2, kli=0.2, kap=0.2, kav=0.2, kai=0.2):
        # Create PID controllers for linear and angular velocities
        self.PID_linear=PID_ctrl(PID, klp, klv, kli, filename_="linear.csv")
        self.PID_angular=PID_ctrl(PID, kap, kav, kai, filename_="angular.csv")

    # Method to calculate velocity requests
    def vel_request(self, pose, goal, status):
        # Calculate linear and angular errors
        e_lin=calculate_linear_error(pose, goal)
        e_ang=calculate_angular_error(pose, goal)

        # Update PID controllers with errors and status
        linear_vel=self.PID_linear.update([e_lin, pose[3]], status)
        angular_vel=self.PID_angular.update([e_ang, pose[3]], status) 

        # Limit velocities to a maximum of 0.5
        linear_vel = 0.5 if linear_vel > 1.0 else linear_vel
        angular_vel= 0.5 if angular_vel > 1.0 else angular_vel

        # Return velocities
        return linear_vel, angular_vel
    

# Define a trajectory controller class that inherits from controller
class trajectoryController(controller):

    # Initialize the trajectory controller with PID parameters and additional parameters
    def __init__(self, klp=0.2, klv=0.2, kli=0.2, kap=0.2, kav=0.2, kai=0.2, lookAhead=1.0, targetVel=1.0):
        super().__init__(klp, klv, kli, kap, kav, kai)
        self.lookAhead=lookAhead
        self.targetVelocity=targetVel
    
    # Method to calculate velocity requests
    def vel_request(self, pose, listGoals, status):
        # Determine the goal to look at
        goal=self.lookFarFor(pose, listGoals)
        
        # Determine the final goal
        finalGoal=listGoals[-1]
        
        # Calculate linear and angular errors
        e_lin=calculate_linear_error(pose, finalGoal)
        e_ang=calculate_angular_error(pose, goal)

        # Update PID controllers with errors and status
        linear_vel=self.PID_linear.update([e_lin, pose[3]], status)
        angular_vel=self.PID_angular.update([e_ang, pose[3]], status) 

        # Limit velocities to a maximum of 0.5
        linear_vel = 0.5 if linear_vel > 1.0 else linear_vel
        angular_vel= 0.5 if angular_vel > 1.0 else angular_vel

        # Return velocities
        return linear_vel, angular_vel

    # Method to determine which goal to look at
    def lookFarFor(self, pose, listGoals):
        # Convert pose and goals to numpy arrays
        poseArray=np.array([pose[0], pose[1]]) 
        listGoalsArray=np.array(listGoals)

        # Calculate squared distances to all goals
        distanceSquared=np.sum((listGoalsArray-poseArray)**2, axis=1)
        # Find the index of the closest goal
        closestIndex=np.argmin(distanceSquared)

        # Return the goal that is 3 steps ahead of the closest, or the last goal if there are less than 3 goals left
        return listGoals[ min(closestIndex + 3, len(listGoals) - 1) ]