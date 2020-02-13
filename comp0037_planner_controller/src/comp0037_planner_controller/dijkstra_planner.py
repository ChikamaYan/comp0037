# -*- coding: utf-8 -*-

from cell_based_forward_search import CellBasedForwardSearch
from search_grid import SearchGrid
from cell import *
from collections import deque
from planned_path import PlannedPath
import operator
from math import *
import rospy


class DijkstraPlanner(CellBasedForwardSearch):

    # Construct the new planner object
    def __init__(self, title, occupancyGrid):
        CellBasedForwardSearch.__init__(self, title, occupancyGrid)
        self.priorityQueue = list()

    # Push cell to queue according to C (distance travelled from start) order. Leftmost is the smallest
    def pushCellOntoQueue(self, cell):
        self.assignCellCosts(cell)

        for i in range(len(self.priorityQueue)):
            if self.priorityQueue[i].getOverallCost() > cell.getOverallCost():
                self.priorityQueue.insert(i, cell)
                # print("Cell costs are: " + str(map(lambda c:c.getOverallCost(),self.priorityQueue)))
                # print("Cell distance costs are: " + str(map(lambda c:c.pathCost,self.priorityQueue)))
                # print("Cell heuristics are: " + str(map(lambda c:c.heuristic,self.priorityQueue)))
                # self.plannerDrawer.waitForKeyPress()
                return
        self.priorityQueue.append(cell)
        # self.priorityQueue.sort(key=lambda c: c.getOverallCost())


    # Check the queue size is zero
    def isQueueEmpty(self):
        return not self.priorityQueue

    def popCellFromQueue(self):
        cell = self.priorityQueue.pop(0)
        return cell

    def resolveDuplicate(self, cell, parentCell):
        currentPathCost = cell.pathCost
        newPathCost = parentCell.pathCost + \
            self.computeLStageAdditiveCost(parentCell, cell)

        currentAngleCost = cell.angleCost
        newAngleCost = parentCell.angleCost + self.computeAngleTurned(parentCell.parent,parentCell,cell)
        if newPathCost <= currentPathCost:
        # a different version that chooses the path with less angle turned if the distance cost is the same
        # if newPathCost < currentPathCost or (newPathCost == currentPathCost and newAngleCost < currentAngleCost):
            cell.parent = parentCell
            cell.pathCost = newPathCost
            cell.angleCost = newAngleCost
            
            # can just pop the cell out and re-insert to ensure best performance
            self.priorityQueue.sort(key=lambda c: c.getOverallCost())

    def getQueueLen(self):
        return len(self.priorityQueue)

    def assignCellCosts(self, cell):
        pathCost = 0
        angleCost = 0
        if cell == self.start and cell.parent is None:
            pathCost = 0
            angleCost = 0
        else:
            pathCost = cell.parent.pathCost + self.computeLStageAdditiveCost(cell.parent, cell)
            if (cell.parent.parent is not None):
                angleCost = cell.parent.angleCost + self.computeAngleTurned(cell.parent.parent,cell.parent,cell)

        cell.pathCost = pathCost
        cell.angleCost = angleCost

        # path = self.getPathEndingAtCell(cell)

        # if (path.travelCost!=pathCost):
        #     print("different: {} and {}".format(path.travelCost,pathCost))
        #     print("the types are: {}, {}".format(type(path.travelCost),type(pathCost)))
        #     self.plannerDrawer.waitForKeyPress()

        # if (round(path.angleTurned,4)!=round(angleCost,4)):
        #     print("different: {} and {}".format(path.angleTurned,angleCost))
        #     self.plannerDrawer.waitForKeyPress()

        

        

