# -*- coding: utf-8 -*-

from dijkstra_planner import DijkstraPlanner
from search_grid import SearchGrid
import math


class AstarPlanner(DijkstraPlanner):

    def __init__(self, title, occupancyGrid, heuristic="squared_euclidean", heuristicWeight=0.01):
        DijkstraPlanner.__init__(self, title, occupancyGrid)
        self.heuristic = heuristic
        self.heuristicWeight = heuristicWeight

    def computeHeuristic(self, cell):
        heuristic = 0

        k_coords = cell.coords
        G_coords = self.goal.coords
        if self.heuristic == "constant":
            heuristic = 10

        elif self.heuristic == "euclidean":
            heuristic =  math.sqrt((cell.coords[0] - self.goal.coords[0])**2 + (cell.coords[1] - self.goal.coords[1])**2)

        elif self.heuristic == "squared_euclidean":
            heuristic = (cell.coords[0] - self.goal.coords[0])**2 + (cell.coords[1] - self.goal.coords[1])**2

        elif self.heuristic == "octile":
            heuristic = max(abs(k_coords[0]-G_coords[0]), abs(k_coords[1]-G_coords[1]))\
                + (math.sqrt(2)-1) * \
                min(abs(k_coords[0]-G_coords[0]), abs(k_coords[1]-G_coords[1]))

        elif self.heuristic == "manhattan":
            heuristic = abs(k_coords[0]-G_coords[0])+abs(k_coords[1]-G_coords[1])
        else:
            print("ERROR: heuristic not defined")
            return None
        
        return heuristic * self.heuristicWeight

    def pushCellOntoQueue(self, cell):
        self.assignCellCosts(cell)

        for i in range(len(self.priorityQueue)):
            if self.priorityQueue[i].getOverallCost() > cell.getOverallCost():
                self.priorityQueue.insert(i, cell)
                return
        self.priorityQueue.append(cell)


    def resolveDuplicate(self, cell, parentCell):
        currentPathCost = cell.pathCost
        newPathCost = parentCell.pathCost + \
            self.computeLStageAdditiveCost(parentCell, cell)

        currentAngleCost = cell.angleCost
        newAngleCost = parentCell.angleCost + self.computeAngleTurned(parentCell.parent,parentCell,cell)
        if newPathCost < currentPathCost or (newPathCost == currentPathCost and newAngleCost < currentAngleCost):
            # choose the path with less angle turned if the distance cost is the same

            cell.parent = parentCell
            cell.pathCost = newPathCost
            cell.angleCost = newAngleCost

            # update current cell's next cell as well -- just for debugging
            # nextCells = self.getNextSetOfCellsToBeVisited(cell)
            # for c in nextCells:
            #     if c.parent == cell:
            #         self.assignCellCosts(c) 
            
            self.priorityQueue.sort(key=lambda c: c.getOverallCost()) 

    def assignCellCosts(self, cell):
        # print("get path for cell {}".format(cell.coords))
        path =self.getPathEndingAtCell(cell)
        # print("cost is: {}".format(path.travelCost))
        cell.pathCost = path.travelCost
        cell.heuristic = self.computeHeuristic(cell)
        cell.angleCost = path.angleTurned
        return

        # super(AstarPlanner,self).assignCellCosts(cell)
        # cell.heuristic = self.computeHeuristic(cell)