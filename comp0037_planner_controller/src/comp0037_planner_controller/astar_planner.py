# -*- coding: utf-8 -*-

from dijkstra_planner import DijkstraPlanner
from search_grid import SearchGrid
from cell import *
from collections import deque
from planned_path import PlannedPath
import math
import rospy

class AstarPlanner(DijkstraPlanner):

    def pushCellOntoQueue(self, cell):
        cell.pathCost = self.getPathEndingAtCell(cell).travelCost + self.computeHeuristic(cell)

        for i in range(len(self.priorityQueue)):
            if self.priorityQueue[i].pathCost > cell.pathCost:
                self.priorityQueue.insert(i,cell)
                return
        self.priorityQueue.append(cell)

    def computeHeuristic(self,cell):
        return 0

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
            pathToCurrent = self.getPathEndingAtCell(cell)

            cell.pathCost = pathToCurrent.travelCost + self.computeHeuristic(cell)
            if (self.hasGoalBeenReached(cell) == True):
                self.goalReached = True
                # break
                # don't break here because dijkstra requires the whole queue to be empty
                # instead, if goal has been reached, just stop adding new cells that would have a higher cost than minimal cost
            
            if not (self.goalReached and cell.pathCost>= self.goal.pathCost):
                cells = self.getNextSetOfCellsToBeVisited(cell)
                for nextCell in cells:
                    if (self.hasCellBeenVisitedAlready(nextCell) == False):
                        self.markCellAsVisitedAndRecordParent(nextCell, cell)
                        self.pushCellOntoQueue(nextCell)
                        self.numberOfCellsVisited = self.numberOfCellsVisited + 1
                    else:
                        self.resolveDuplicate(nextCell, cell)
                    # update maximum length of queue
                    self.maxQueueLen = max(self.maxQueueLen,self.getQueueLen())

            # Now that we've checked all the actions for this cell,
            # mark it as dead
            self.markCellAsDead(cell)

            # Draw the update if required
            self.drawCurrentState()

        # Do a final draw to make sure that the graphics are shown, even at the end state
        self.drawCurrentState()
        
        print "numberOfCellsVisited = " + str(self.numberOfCellsVisited)
        print ("maxQueueLen = " + str(self.maxQueueLen))
        
        if self.goalReached:
            print "Goal reached"
        else:
            print "Goal not reached"

        return self.goalReached

