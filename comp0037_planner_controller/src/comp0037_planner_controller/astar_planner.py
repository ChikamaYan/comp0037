# -*- coding: utf-8 -*-

from dijkstra_planner import DijkstraPlanner
from search_grid import SearchGrid
from cell import *
from collections import deque
from planned_path import PlannedPath
import math
import rospy


class AstarPlanner(DijkstraPlanner):

    def __init__(self, title, occupancyGrid, heuristic="constant"):
        DijkstraPlanner.__init__(self, title, occupancyGrid)
        self.heuristic = heuristic

    def computeHeuristic(self, cell):
        if self.heuristic == "constant":
            return 10

        if self.heuristic == "euclidean":
            return math.sqrt((cell.coords[0] - self.goal.coords[0])**2 + (cell.coords[1] - self.goal.coords[1])**2)

        k_coords = cell.coords
        G_coords = self.goal.coords
        if self.heuristic == "octile":
            result = max(abs(k_coords[0]-G_coords[0]), abs(k_coords[1]-G_coords[1]))\
                + (math.sqrt(2)-1) * \
                min(abs(k_coords[0]-G_coords[0]), abs(k_coords[1]-G_coords[1]))
            return result

        if self.heuristic == "manhattan":

            return abs(k_coords[0]-G_coords[0])+abs(k_coords[1]-G_coords[1])

    def resolveDuplicate(self, cell, parentCell):
        currentPathCost = cell.pathCost
        newPathCost = parentCell.pathCost - self.computeHeuristic(parentCell) + \
            self.computeLStageAdditiveCost(parentCell, cell) + self.computeHeuristic(cell)

        if newPathCost <= currentPathCost:
            # TODO: a good idea here is to choose the parent with less degree when travel cost is equal
            cell.parent = parentCell
            cell.pathCost = newPathCost
            self.priorityQueue.sort(key=lambda c: c.pathCost)

    def computeCellCost(self, cell):
        ans = 0
        if cell == self.start and cell.parent is None:
            ans = self.computeHeuristic(cell)
        else:
            ans = cell.parent.pathCost - self.computeHeuristic(cell.parent) + \
                self.computeLStageAdditiveCost(cell.parent, cell) + self.computeHeuristic(cell)
        
        return ans
        # return self.getPathEndingAtCell(cell).travelCost + self.computeHeuristic(cell)