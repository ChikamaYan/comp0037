from controller_base import ControllerBase
from move2goal_controller import Move2GoalController

import rospy
from geometry_msgs.msg  import Twist
from geometry_msgs.msg  import Pose2D
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from math import pow,atan2,sqrt,pi
from planned_path import PlannedPath
import time
from math import degrees, fabs


class PathSimplifyingController(Move2GoalController):
    def __init__(self, occupancyGrid, optimise=True):
        Move2GoalController.__init__(self, occupancyGrid)
        self.totalDistance = 0
        self.totalAngle = 0
        self.totalTime = 0
        self.totalPlannedCost = 0
        self.totalPlannedAngle = 0
        self.pose = None
        self.plannerDrawer = None
        self.optimise = optimise

    def odometryCallback(self, odometry):
        odometryPose = odometry.pose.pose
        position = odometryPose.position
        orientation = odometryPose.orientation

        # print orientation
        if self.pose is None:
            self.pose = Pose2D()
            self.pose.x = position.x
            self.pose.y = position.y
            self.pose.theta = 2 * atan2(orientation.z, orientation.w)

        self.totalDistance += sqrt((position.x - self.pose.x) ** 2 + (position.y - self.pose.y) ** 2)
        self.totalAngle += degrees(fabs(self.shortestAngularDistance(
                            self.pose.theta, 2 * atan2(orientation.z, orientation.w))))
        # print "Distance {0:.4f}, angle {1:.4f}, time {2:4f}".format(self.totalDistance, self.totalAngle, self.totalTime)
        position = odometryPose.position
        orientation = odometryPose.orientation

        self.pose.x = position.x
        self.pose.y = position.y
        self.pose.theta = 2 * atan2(orientation.z, orientation.w)

    def simplifyPath(self, path):
        newPath = PlannedPath()
        newPath.goalReached = path.goalReached
        newPath.travelCost = path.travelCost
        newPath.angleTurned = path.angleTurned

        newPath.waypoints.append(path.waypoints[0])
        newPath.numberOfWaypoints = 1
        for i in range(1, len(path.waypoints) - 1):
            coord1 = path.waypoints[i - 1].coords
            coord2 = path.waypoints[i].coords
            coord3 = path.waypoints[i + 1].coords
            # print("{} {} {}".format(coord1, coord2, coord3))
            if coord3[0] - coord2[0] != coord2[0] - coord1[0] or coord3[1] - coord2[1] != coord2[1] - coord1[1]:
                newPath.waypoints.append(path.waypoints[i])
                newPath.numberOfWaypoints += 1
                # print "Yes"

        newPath.waypoints.append(path.waypoints[len(path.waypoints) - 1])
        newPath.numberOfWaypoints += 1
        # print "The following is the new path:"
        # for cell in newPath.waypoints:
        #     print cell.coords
        # print newPath.numberOfWaypoints
        # rospy.sleep(10)
        # self.plannerDrawer.flushAndUpdateWindow()
        # rospy.sleep(10000)
        return newPath

    def drivePathToGoal(self, path, goalOrientation, plannerDrawer):
        self.plannerDrawer = plannerDrawer
        rospy.loginfo('Driving path to goal with ' + str(len(path.waypoints)) + ' waypoint(s)')

        simplerPath = self.simplifyPath(path) if self.optimise else path
        for cell in simplerPath.waypoints:
            print cell.coords
        print simplerPath.numberOfWaypoints
        for cell in simplerPath.waypoints:
            waypoint = self.occupancyGrid.getWorldCoordinatesFromCellCoordinates(cell.coords)
            # rospy.loginfo("Driving to waypoint (%f, %f)", waypoint[0], waypoint[1])
            self.driveToWaypoint(waypoint)
            # Handle ^C
            if rospy.is_shutdown() is True:
                break

        rospy.loginfo('Rotating to goal orientation (' + str(goalOrientation) + ')')

        # Finish off by rotating the robot to the final configuration
        if rospy.is_shutdown() is False:
            self.rotateToGoalOrientation(goalOrientation)

        self.totalPlannedCost += path.travelCost
        self.totalPlannedAngle += path.angleTurned
        print "Planned cost {0:4f}, angle{1:4f}".format(self.totalPlannedCost, self.totalPlannedAngle)
        print "Distance {0:.4f}, angle {1:.4f}, time {2:4f}".format(self.totalDistance, self.totalAngle, self.totalTime)

    def driveToWaypoint(self, waypoint):
        vel_msg = Twist()

        dX = waypoint[0] - self.pose.x
        dY = waypoint[1] - self.pose.y
        distanceError = sqrt(dX * dX + dY * dY)
        angleError = self.shortestAngularDistance(self.pose.theta, atan2(dY, dX))

        prevTime = rospy.get_time()
        while (distanceError >= self.distanceErrorTolerance) & (not rospy.is_shutdown()):
            # print("Distance Error: {}\nAngular Error: {}".format(distanceError, angleError))
            if fabs(angleError) < self.driveAngleErrorTolerance:
                vel_msg.linear.x = max(0.0, min(self.distanceErrorGain * distanceError, 10.0))
                vel_msg.linear.y = 0
                vel_msg.linear.z = 0

            vel_msg.angular.x = 0
            vel_msg.angular.y = 0
            vel_msg.angular.z = max(-5.0, min(self.angleErrorGain * angleError, 5.0))

            self.totalTime += rospy.get_time() - prevTime
            self.velocityPublisher.publish(vel_msg)
            if self.plannerDrawer is not None:
                self.plannerDrawer.flushAndUpdateWindow()

            prevTime = rospy.get_time()
            self.rate.sleep()

            distanceError = sqrt(pow((waypoint[0] - self.pose.x), 2) + pow((waypoint[1] - self.pose.y), 2))
            angleError = self.shortestAngularDistance(self.pose.theta,
                                                      atan2(waypoint[1] - self.pose.y, waypoint[0] - self.pose.x))

        vel_msg.linear.x = 0
        vel_msg.angular.z = 0
        self.velocityPublisher.publish(vel_msg)


