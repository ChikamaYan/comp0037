# -*- coding: utf-8 -*-

from cell_based_forward_search import CellBasedForwardSearch
from Queue import PriorityQueue
import math

# This class implements the greedy planner using Euclidean distance to the goal to order

class GreedyPlanner(CellBasedForwardSearch):

    # Construct the new planner object
    def __init__(self, title, occupancyGrid):
        CellBasedForwardSearch.__init__(self, title, occupancyGrid)
        self.priorityQueue = PriorityQueue()

    # Push cell to queue according to Euclidean distance order. Leftmost is the smallest
    def pushCellOntoQueue(self, cell):
        cell.heuristic = self.computeHeuristic(cell)
    
        self.priorityQueue.put((cell.heuristic,cell))

    # Check the queue size is zero
    def isQueueEmpty(self):
        return self.priorityQueue.empty()

    # Pull the cell with least Euclidean distance
    def popCellFromQueue(self):
        cell = self.priorityQueue.get()[1]
        print(cell)
        return cell

    def resolveDuplicate(self, cell, parentCell):
        # Nothing to do in self case
        pass

    def getQueueLen(self):
        return self.priorityQueue.qsize()

    # computer the heuristic from given cell to goal. Here it would be Euclidean distance
    def computeHeuristic(self,cell):
        return math.sqrt((cell.coords[0] - self.goal.coords[0])**2 + (cell.coords[1] - self.goal.coords[1])**2)
