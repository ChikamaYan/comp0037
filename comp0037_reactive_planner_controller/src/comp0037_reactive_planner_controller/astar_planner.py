# -*- coding: utf-8 -*-

from cell_based_forward_search import CellBasedForwardSearch
import math
import copy

# only admissible heuristics are meant to be used
class AstarPlanner(CellBasedForwardSearch):

    def __init__(self, title, occupancyGrid, heuristic="octile", heuristicWeight=1.0):
        CellBasedForwardSearch.__init__(self, title, occupancyGrid)
        self.priorityQueue = list()
        self.heuristic = heuristic
        self.heuristicWeight = heuristicWeight

    # Check the queue size is zero
    def isQueueEmpty(self):
        return not self.priorityQueue

    def popCellFromQueue(self):
        cell = self.priorityQueue.pop(0)
        return cell

    def getQueueLen(self):
        return len(self.priorityQueue)

    def computeHeuristic(self, cell):
        heuristic = 0

        k_coords = cell.coords
        G_coords = self.goal.coords
        if self.heuristic == "constant":
            heuristic = 10

        elif self.heuristic == "euclidean":
            heuristic =  math.sqrt((cell.coords[0] - self.goal.coords[0])**2 + (cell.coords[1] - self.goal.coords[1])**2)

        # elif self.heuristic == "squared_euclidean":
        #     heuristic = (cell.coords[0] - self.goal.coords[0])**2 + (cell.coords[1] - self.goal.coords[1])**2

        elif self.heuristic == "octile":
            heuristic = max(abs(k_coords[0]-G_coords[0]), abs(k_coords[1]-G_coords[1]))\
                + (math.sqrt(2)-1) * \
                min(abs(k_coords[0]-G_coords[0]), abs(k_coords[1]-G_coords[1]))

        elif self.heuristic == "manhattan":
            heuristic = abs(k_coords[0]-G_coords[0])+abs(k_coords[1]-G_coords[1])

        # elif self.heuristic == "dijkstra":
        #     if cell == self.start and cell.heuristic == 0:
        #         dj = DijkstraPlanner(self.title, copy.deepcopy(self.occupancyGrid))
        #         dj.setShowGraphics(False)
        #         dj.search(G_coords, k_coords)
        #         for x in range(self.occupancyGrid.getWidthInCells()):
        #             for y in range(self.occupancyGrid.getHeightInCells()):
        #                 c = dj.searchGrid.grid[x][y]
        #                 s = self.searchGrid.grid[x][y]
        #                 s.heuristic = c.pathCost if c.pathCost != 0 else 9999

        #     if cell == self.goal:
        #         heuristic = 0
        #     else:
        #         heuristic = cell.heuristic

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
        # self.assignCellCosts(cell)
        # re-assigning cost is needed to avoid infinite loop
        currentPathCost = cell.pathCost
        newPathCost = parentCell.pathCost + \
            self.computeLStageAdditiveCost(parentCell, cell)

        currentAngleCost = cell.angleCost
        newAngleCost = parentCell.angleCost + self.computeAngleTurned(parentCell.parent,parentCell,cell)
        if newPathCost < currentPathCost:
            # or (newPathCost == currentPathCost and newAngleCost < currentAngleCost)

            cell.parent = parentCell
            # self.assignCellCosts(cell)
            cell.pathCost = newPathCost
            cell.angleCost = newAngleCost

            self.priorityQueue.sort(key=lambda c: c.getOverallCost()) 

    def assignCellCosts(self, cell):
        pathCost = 0
        angleCost = 0
        if not (cell == self.start and cell.parent is None):
            pathCost = cell.parent.pathCost + self.computeLStageAdditiveCost(cell.parent, cell)
            if (cell.parent.parent is not None):
                angleCost = cell.parent.angleCost + self.computeAngleTurned(cell.parent.parent,cell.parent,cell)

        cell.pathCost = pathCost
        cell.angleCost = angleCost
        cell.heuristic = self.computeHeuristic(cell)
