from occupancy_grid import OccupancyGrid

from comp0037_planner_controller.astar_planner import AstarPlanner
from comp0037_planner_controller.greedy_planner import GreedyPlanner
from comp0037_planner_controller.dijkstra_planner import DijkstraPlanner
from comp0037_planner_controller.fifo_planner import FIFOPlanner
from comp0037_planner_controller.lifo_planner import LIFOPlanner

occupancyGrid = OccupancyGrid(21, 8, 0.5)

for x in range(3, 17):
    occupancyGrid.setCell(x, 4, 1)
occupancyGrid.setCell(16, 3, 1)
occupancyGrid.setCell(16, 2, 1)
occupancyGrid.setCell(17, 2, 1)
occupancyGrid.setCell(18, 2, 1)
occupancyGrid.setCell(19, 2, 1)

start = (1, 3)
goal = (18, 3)


planner = AstarPlanner('astar Search', occupancyGrid, "dijkstra")
# planner = GreedyPlanner('greedy Search', occupancyGrid)
# planner = DijkstraPlanner('dij Search', occupancyGrid)
# planner = FIFOPlanner('Breadth First Search', occupancyGrid);
# planner = LIFOPlanner('Depth First Search', occupancyGrid)

planner.setRunInteractively(True)

planner.setWindowHeightInPixels(400)

goalReached = planner.search(start, goal)

path = planner.extractPathToGoal()