#! /usr/bin/env python

# Import the needed types.
from comp0037_planner_controller.dijkstra_planner import DijkstraPlanner
from comp0037_planner_controller.occupancy_grid import OccupancyGrid
import map_getter
import rospy

# Initialise node
rospy.init_node('dijkstra_standalone', anonymous=True)

# Mapgetter  helps load maps off the map server
mapGetter = map_getter.MapGetter()
occupancyGrid = mapGetter.getMapFromServer()

start = rospy.get_param("start_pose")
goal = rospy.get_param("goal_pose")

planner = DijkstraPlanner('Dijkstra Search', occupancyGrid)

planner.setRunInteractively(True)

planner.setWindowHeightInPixels(400)

goalReached = planner.search(start, goal)

path = planner.extractPathToGoal()
