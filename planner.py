import math
# Type of planner
POINT_PLANNER=0; TRAJECTORY_PLANNER=1

class planner:
    def __init__(self, type_):

        self.type=type_

    
    def plan(self, goalPoint=[-1.0, -1.0, 0.0]):
        
        if self.type==POINT_PLANNER:
            return self.point_planner(goalPoint)
        
        elif self.type==TRAJECTORY_PLANNER:
            return self.trajectory_planner() 


    def point_planner(self, goalPoint):
        x = goalPoint[0]
        y = goalPoint[1]
        theta = goalPoint[2]
        return x, y, theta

    # TODO Part 6: Implement the trajectories here
    def trajectory_planner(self):
        '''
        The trajectory planner works by returning a list of points
        that the robot should follow. The robot will move from the
        first point in the list to the second, from the second to
        the third, and so on.

        The distance between each point in the list (and hence the granularity of the points composing the trajectory) in the list is set by the x_step var

        Uncomment the lines 41-45 to use the trajectory planner
        with the quadratic function instead of sigmoid and comment
        out lines 49-54

        # Quadratic
        # goals = []
        # x_step = 0.13 #
        # for i in range(10):
        #     goals.append([x_step*i, (x_step*i)**2])
        # return goals
        '''

        # Sigmoid
        goals = []
        x_step = 0.22
        for i in range(20):
            x = x_step*i - 1.5
            goals.append([x, 1/(1+math.exp(-1*(x/4)))])
        return goals
    

        # the return should be a list of trajectory points: [ [x1,y1], ..., [xn,yn]]
        # return 

