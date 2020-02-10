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
        if newPathCost < currentPathCost or (newPathCost == currentPathCost and newAngleCost < currentAngleCost):
            # choose the path with less angle turned if the distance cost is the same

            cell.parent = parentCell
            cell.pathCost = newPathCost
            cell.angleCost = newAngleCost
            
            self.priorityQueue.sort(key=lambda c: c.getOverallCost())

    def getQueueLen(self):
        return len(self.priorityQueue)

    # overwrite
    def search(self, startCoords, goalCoords):
        # Make sure the queue is empty. We do this so that we can keep calling
        # the same method multiple times and have it work.
        while (self.isQueueEmpty() == False):
            self.popCellFromQueue()

        # Create or update the search grid from the occupancy grid and seed
        # unvisited and occupied cells.
        if (self.searchGrid is None):
            self.searchGrid = SearchGrid.fromOccupancyGrid(self.occupancyGrid)
        else:
            self.searchGrid.updateFromOccupancyGrid()

        # Get the start cell object and label it as such. Also set its
        # path cost to 0.
        self.start = self.searchGrid.getCellFromCoords(startCoords)
        self.start.label = CellLabel.START
        self.start.pathCost = 0

        # Get the goal cell object and label it.
        self.goal = self.searchGrid.getCellFromCoords(goalCoords)
        self.goal.label = CellLabel.GOAL

        # If the node is being shut down, bail out here.
        if rospy.is_shutdown():
            return False

        # Draw the initial state
        self.resetGraphics()

        # Insert the start on the queue to start the process going.
        self.markCellAsVisitedAndRecordParent(self.start, None)
        self.pushCellOntoQueue(self.start)

        # Reset the count
        self.numberOfCellsVisited = 0

        # Indicates if we reached the goal or not
        self.goalReached = False

        # Iterate until we have run out of live cells to try or we reached the goal.
        # This is the main computational loop and is the implementation of
        # LaValle's pseudocode
        while (self.isQueueEmpty() == False):

            # Check if ROS is shutting down; if so, abort. This stops the
            # planner from hanging.
            if rospy.is_shutdown():
                return False

            cell = self.popCellFromQueue()

            self.assignCellCosts(cell)
            # print("parent is " + str(cell.parent))
            # self.plannerDrawer.waitForKeyPress()
            if (self.hasGoalBeenReached(cell) == True):
                self.goalReached = True
                # break
                # don't break here because dijkstra requires the whole queue to be empty
                # instead, if goal has been reached, just stop adding new cells that would have a higher cost than minimal cost

            if not (self.goalReached and cell.getOverallCost() >= self.goal.getOverallCost()):
                cells = self.getNextSetOfCellsToBeVisited(cell)
                for nextCell in cells:
                    if (self.hasCellBeenVisitedAlready(nextCell) == False):
                        self.markCellAsVisitedAndRecordParent(nextCell, cell)
                        self.pushCellOntoQueue(nextCell)
                        self.numberOfCellsVisited = self.numberOfCellsVisited + 1
                    else:
                        self.resolveDuplicate(nextCell, cell)
                    # update maximum length of queue
                    self.maxQueueLen = max(
                        self.maxQueueLen, self.getQueueLen())

            # Now that we've checked all the actions for this cell,
            # mark it as dead
            self.markCellAsDead(cell)

            # Draw the update if required
            self.drawCurrentState()

        # Do a final draw to make sure that the graphics are shown, even at the end state
        self.drawCurrentState()

        print "numberOfCellsVisited = " + str(self.numberOfCellsVisited)
        print("maxQueueLen = " + str(self.maxQueueLen))

        if self.goalReached:
            print "Goal reached"
        else:
            print "Goal not reached"

        return self.goalReached

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

    def computeAngleTurned(self, cell1, cell2, cell3):
        """
        compute the angel turned when going cell1->cell2->cell3
        """
        if (cell1 is None or cell2 is None or cell3 is None):
            return 0

        currentDirection = list(map(operator.sub,cell3.coords,cell2.coords))
        preDirection = list(map(operator.sub,cell2.coords,cell1.coords))
        dotProduct = currentDirection[0] * preDirection[0] + currentDirection[1] * preDirection[1]
        cosValue = dotProduct / float(sqrt((sum(map(lambda x:x*x,currentDirection)))) * float(sqrt(sum(map(lambda x:x*x,preDirection)))))
        degree = degrees(acos(cosValue))

        return degree
        

        

