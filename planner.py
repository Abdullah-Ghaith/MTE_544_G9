from mapUtilities import *
from a_star import *

# CONSTANTS
POINT_PLANNER=0; TRAJECTORY_PLANNER=1

class planner:
    def __init__(self, type_, mapName="room"):
        # Initialize the planner with the type of planner and the map name
        self.type=type_
        self.mapName=mapName

    def plan(self, startPose, endPose):
        # Depending on the type of planner, call the appropriate planning function
        if self.type==POINT_PLANNER:
            return self.point_planner(endPose)
        elif self.type==TRAJECTORY_PLANNER:
            self.costMap=None
            self.initTrajectoryPlanner()
            return self.trajectory_planner(startPose, endPose)

    def point_planner(self, endPose):
        # For point planner, simply return the end pose
        return endPose

    def initTrajectoryPlanner(self):
        # Initialize the trajectory planner by creating the cost map
        # TODO PART 5 Create the cost-map, the laser_sig is 
        # the standard deviation for the gausiian for which
        # the mean is located on the occupant grid  . 
        self.m_utilites=mapManipulator(filename_ = self.mapName+".yaml",  laser_sig=0.5)
        self.costMap=self.m_utilites.make_likelihood_field()

    def trajectory_planner(self, startPoseCart, endPoseCart):
        # This is to convert the cartesian coordinates into the 
        # the pixel coordinates of the map image, remmember,
        # the cost-map is in pixels. You can by the way, convert the pixels
        # to the cartesian coordinates and work by that index, the a_star finds
        # the path regardless. 
        # TODO PART 5 convert the cell pixels into the cartesian coordinates
        startPose=self.m_utilites.position_2_cell(startPoseCart)
        endPose=self.m_utilites.position_2_cell(endPoseCart)
        
        # Use the A* search algorithm to find the path
        path = search(self.costMap, startPose, endPose, heuristic_type='euclidean')

        # TODO PART 5 return the path as list of [x,y]
        # Convert the path from pixel coordinates to cartesian coordinates and return it
        return list(map(self.m_utilites.cell_2_position, path)) #trajectory

if __name__=="__main__":
    m_utilites=mapManipulator()
    map_likelihood=m_utilites.make_likelihood_field()
    # This part of the code can be used to test the search algorithm independently of the ros2 hassles