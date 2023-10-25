import numpy as np


from pid import PID_ctrl
from utilities import calculate_angular_error, calculate_linear_error

M_PI=3.1415926535

P=0; PD=1; PI=2; PID=3

class controller:
    
    
    # Default gains of the controller for linear and angular motions
    def __init__(self, klp=0.2, klv=0.2, kli=0.2, kap=0.2, kav=0.2, kai=0.2):
        
        # TODO Part 5 and 6: Modify the below lines to test your PD, PI, and PID controller
        self.PID_linear=PID_ctrl(P, klp, klv, kli, filename_="linear.csv")
        self.PID_angular=PID_ctrl(P, kap, kav, kai, filename_="angular.csv")

    
    def vel_request(self, pose, goal, status):
        
        #calculate the error in linear and angular velocities
        e_lin=calculate_linear_error(pose, goal)
        e_ang=calculate_angular_error(pose, goal)

        # Calculate the new desired linear and angular velocities from pose and errors
        linear_vel=self.PID_linear.update([e_lin, pose[3]], status)
        angular_vel=self.PID_angular.update([e_ang, pose[3]], status)
        
        # TODO Part 4: Add saturation limits for the robot linear and angular velocity
        linear_vel = max(-0.31, min(0.31, linear_vel)) # Tb4 specs say max linear velocity is 0.31 m/s in safe mode
        angular_vel = max(-1.9, min(1.9, angular_vel)) # Tb4 specs say max angular velocity is 1.9 rad/s
        
        return linear_vel, angular_vel
    

class trajectoryController(controller):

    def __init__(self, klp=0.2, klv=0.2, kli=0.2, kap=0.2, kav=0.2, kai=0.2, lookAhead=1.0):
        
        super().__init__(klp, klv, kli, kap, kav, kai)
        self.lookAhead=lookAhead
    
    def vel_request(self, pose, listGoals, status):
        
        goal=self.lookFarFor(pose, listGoals)
        
        #final goal of trajectory is last set of points from list of goals
        finalGoal=listGoals[-1]
        
        #calculate the error in linear and angular velocities
        e_lin=calculate_linear_error(pose, finalGoal)
        e_ang=calculate_angular_error(pose, goal)
        
        # Calculate the new desired linear and angular velocities from pose and errors
        linear_vel=self.PID_linear.update([e_lin, pose[3]], status)
        angular_vel=self.PID_angular.update([e_ang, pose[3]], status) 

        # TODO Part 4: Add saturation limits for the robot linear and angular velocity
        linear_vel = max(-0.31, min(0.31, linear_vel)) # Tb4 specs say max linear velocity is 0.31 m/s in safe mode
        angular_vel = max(-1.9, min(1.9, angular_vel)) # Tb4 specs say max angular velocity is 1.9 rad/s
        
        return linear_vel, angular_vel

    def lookFarFor(self, pose, listGoals):
        
        poseArray=np.array([pose[0], pose[1]]) 
        listGoalsArray=np.array(listGoals)

        distanceSquared=np.sum((listGoalsArray-poseArray)**2,
                               axis=1)
        closestIndex=np.argmin(distanceSquared)

        return listGoals[ min(closestIndex + 3, len(listGoals) - 1) ]
