# -*- coding: utf-8 -*-

from dijkstra_planner import DijkstraPlanner
from search_grid import SearchGrid
from cell import *
from collections import deque
from planned_path import PlannedPath
import math
import rospy


class AstarPlanner(DijkstraPlanner):

    def __init__(self, title, occupancyGrid, heuristic="manhattan"):
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

        print("ERROR: heuristic not defined")
        return None

    def pushCellOntoQueue(self, cell):
        self.assignCellCosts(cell)

        for i in range(len(self.priorityQueue)):
            if self.priorityQueue[i].getOverallCost() > cell.getOverallCost():
                self.priorityQueue.insert(i, cell)
                return
        self.priorityQueue.append(cell)


    def resolveDuplicate(self, cell, parentCell):
        currentPathCost = cell.pathCost
        newPathCost = parentCell.pathCost - self.computeHeuristic(parentCell)+ \
            self.computeLStageAdditiveCost(parentCell, cell) + self.computeHeuristic(cell)

        currentAngleCost = cell.angleCost
        newAngleCost = parentCell.angleCost + self.computeAngleTurned(parentCell.parent,parentCell,cell)
        if newPathCost < currentPathCost or (newPathCost == currentPathCost and newAngleCost < currentAngleCost):
            # choose the path with less angle turned if the distance cost is the same

            cell.parent = parentCell
            cell.pathCost = newPathCost
            cell.angleCost = newAngleCost
            
            self.priorityQueue.sort(key=lambda c: c.getOverallCost()) 

    def assignCellCosts(self, cell):
        # path =self.getPathEndingAtCell(cell)
        # cell.pathCost = path.travelCost + self.computeHeuristic(cell)
        # cell.angleCost = path.angleTurned
        # return


        # pathCost = 0
        # angleCost = 0
        # if cell == self.start and cell.parent is None:
        #     pathCost = self.computeHeuristic(cell)
        #     angleCost = 0
        # else:
        #     pathCost = cell.parent.pathCost - self.computeHeuristic(cell.parent) + \
        #         self.computeLStageAdditiveCost(cell.parent, cell) + self.computeHeuristic(cell)
        #     if (cell.parent.parent is not None):
        #         angleCost = cell.parent.angleCost + self.computeAngleTurned(cell.parent.parent,cell.parent,cell)

        # cell.pathCost = pathCost
        # cell.angleCost = angleCost

        super(AstarPlanner,self).assignCellCosts(cell)
        cell.heuristic = self.computeHeuristic(cell)