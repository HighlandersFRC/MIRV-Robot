#!/usr/bin/env python3
import math
import numpy as np
from numpy import array
import actionlib
import time
import rospy
import sys
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import helpful_functions_lib as conversion_lib
import mirv_control.msg as ASmsg


class PurePursuit():
    wheelBaseWidth = 0.483
    nextPointDistanceDec = 0
    logData = [0, 0]
    maxAngularVel = 1.5
    maxDriveSpeed = 0.75
    currentMaxDriveSpeed = 0.5
    startingTheta = 3.14159/2
    lookAheadDist = 3
    allowedError = 0.83
    allowedErrorDist = 0.04
    robotCordList = []
    cordList = []
    currentTruckCord = [0, 0, 0]
    rosPubMsg = Twist()
    pub = rospy.Publisher("cmd_vel", Twist, queue_size=5)
    debugPub = rospy.Publisher("PPdebug", Float64MultiArray, queue_size=5)
    debugMsg = Float64MultiArray()
    feedback = ASmsg.PurePursuitFeedback()
    angleToTarget = 0
    result = ASmsg.PurePursuitResult()
    lastlooptime = 0
    upperBoundLine = []
    lowerBoundLine = []

    def __init__(self):
        rospy.init_node('PurePursuitController', anonymous=True)
        self._action_name = 'PurePursuitAS'

        self._as = actionlib.SimpleActionServer(self._action_name, ASmsg.PurePursuitAction, auto_start = False)
        self._as.register_goal_callback(self.ServerCallback)
        self._as.register_preempt_callback(self.preemptCallback)
    
        self._as.start()
        sub = rospy.Subscriber("/EKF/Odometry", Odometry, self.callBackOdom)
        rospy.loginfo_throttle(0.5, "Pure pursuit Output: LeftSpeed: {}, RightSpeed: {}".format(self.logData[0], self.logData[1]))

    def UpdateTargetPoints(self):
        newTruckCord = self.getTruckCord()
        theta = newTruckCord[2]
        xBRef = newTruckCord[0]
        yBRef = newTruckCord[1]
        # print([xBRef, yBRef, theta])
        length = 0
        for i in range(len(self.robotCordList)):
            xTBody = self.cordList[i][0]*math.cos(theta) + self.cordList[i][1]*math.sin(
                theta) - yBRef*math.sin(theta) - xBRef*math.cos(theta)
            yTBody = -self.cordList[i][0]*math.sin(theta) + self.cordList[i][1]*math.cos(
                theta) - yBRef*math.cos(theta) + xBRef*math.sin(theta)
            distToPoint = round(
                math.sqrt(math.pow(xTBody, 2) + math.pow(yTBody, 2)), 3)
            self.robotCordList[i] = [xTBody, yTBody, distToPoint]
        print("current Truck Position: {}".format(newTruckCord))
        print("target X,Y: {}, CurrentTheta: {}".format(
            self.robotCordList[0], theta))
        print("targetTruck X,Y: {}".format(self.cordList[0]))
    # helper methods for update target points

    def getTruckCord(self):
        return self.currentTruckCord

    def setPath(self, pathList):
        for point in pathList:
            print("uploading point: {}".format(point))
            self.cordList.append([point[0], point[1], 0])
            self.robotCordList.append([point[0], point[1], 0])
        print("uploaded path")
        print(self.robotCordList)

    def removeTargetPoint(self):
        if (abs(self.robotCordList[0][2]) < 2.5 and len(self.robotCordList) > 1):
            self.robotCordList.pop(0)
            self.cordList.pop(0)
            print("removing target point")
            if(self.robotCordList):
                self.nextPointDistanceDec = self.robotCordList[0][2]
            else:
                return True
        elif (self.robotCordList[0][2] < self.allowedError and len(self.robotCordList) <= 1):
            #calculate angle here
            self.angleToTarget = math.degrees(math.atan2(-self.robotCordList[0][1],self.robotCordList[0][0]))
            self.robotCordList.pop(0)
            self.cordList.pop(0)
            print("removing target point")
            return True

    def checkOutside(self):
        pass
        

    # calulates wheel velocity's for that given frame
    def calculateSpeedSide(self, maxSpeed, x, y, la):
        RobotTwist = [maxSpeed, 0]
        if(abs(y) > self.allowedErrorDist):
            cRad = self.generateRadius(y, la)
            ################################
            # Take circ of radius 4 and max speed of 1
            # circumference = 8*pi (4*2*pi)
            # time to travel the circle = circumference/maxSpeed
            # so it takes the same time to travel 2*pi radians
            # by definition of velocity (displacement/time)
            # we get 2*pi/8*pi = 1/4
            ################################
            angularVel = maxSpeed/cRad
            if(x<0):
               angularVel = 2.5*angularVel 
            if(y >= 0):
                RobotTwist = [maxSpeed, angularVel]
            else:
                RobotTwist = [maxSpeed, -angularVel]
        else:
            if(x < 0):
                if(y >= 0):
                    RobotTwist = [0, self.maxAngularVel]
                else:
                    RobotTwist = [0, -self.maxAngularVel]

        return (RobotTwist)

    # helper methods to calculateSpeedSides
    def generateRadius(self, y, la):
        return(math.pow(la, 2)/(abs(2*y)))

    def generateCircumference(self, rad):
        return(2*3.141592*rad)

    # determines the target cord for each snapshot

    def getTargetCordAndDriveSpeed(self, la, maxSpeed):
        closePoint = self.closestOut(la)
        farPoint = self.furthestIn(la)
        targetPoint = []
        snapshotLa = la
        if (closePoint and farPoint):
            targetPoint = self.doubleVectorIntercept(closePoint, farPoint, la)
        elif farPoint:
            targetPoint = farPoint
            snapshotLa = (targetPoint[0]**2 + targetPoint[1]**2)**0.5
            self.currentMaxDriveSpeed = 0.5
            if (snapshotLa < self.allowedError):
                self.currentMaxDriveSpeed = self.maxDriveSpeed
                return ("atDest")

        elif closePoint:
            targetPoint = self.vectorIntercept(closePoint, la)

        else:
            print("cannot determine target point")
        RobotTwist = self.calculateSpeedSide(
            maxSpeed, targetPoint[0], targetPoint[1], snapshotLa)
        return(RobotTwist)

    # helper method for getTargetCord
    def vectorIntercept(self, closePoint, la):
        cPO = array(closePoint)
        normalVect = (cPO)/(((cPO**2).sum()**0.5))
        targetPoint = normalVect*la
        # print("Vector Intercept")
        return targetPoint

    def doubleVectorIntercept(self, closePoint, farPoint, la):
        cPO = array(closePoint)
        fPI = array(farPoint)
        zeroVect = cPO-fPI
        prodOfMag = ((fPI**2).sum()**0.5)*((zeroVect**2).sum()**0.5)
        gama = 3.141592 - math.acos(((zeroVect.dot(fPI))/prodOfMag))
        zeta = math.asin((((fPI**2).sum()**0.5)*math.sin(gama))/(la))
        theta = 3.1415-gama-zeta
        distToEdge = la*(math.sin(theta))/math.sin(gama)
        unitP2Vect = ((cPO-fPI)/(((cPO-fPI)**2).sum()**0.5))
        p2EVect = distToEdge*unitP2Vect
        target = p2EVect+fPI
        # print("double Vector Intercept")
        return target

    def closestOut(self, la):
        closePoint = []
        for point in self.robotCordList:
            if(point[2] > la):
                return [point[0], point[1]]
        return closePoint

    def furthestIn(self, la):
        farPoint = []
        for point in self.robotCordList:
            if(point[2] <= la):
                farPoint = [point[0], point[1]]
            else:
                if(farPoint):
                    return [farPoint[0], farPoint[1]]
        return farPoint
    def cancelCallback(self):
        print(self._as.is_active())
        self._as.set_aborted()
        print(self._as.is_active())
        self.robotCordList.clear()
        self.robotCordList.clear()
        print(self.robotCordList)
        
    def preemptCallback(self):
        # self._as.set_preempted()
        print("__________________________________________________")
        if(self._as.is_new_goal_available()):
            print("preempt request recived")
            print(self._as.is_active())
            self._as.set_preempted()
            print(self._as.is_active())
            print(self.robotCordList)
        else:
            print("aborting goal")
            self.cancelCallback()

    def ServerCallback(self):
        print("got callback")
        goal = self._as.accept_new_goal()
        numTarget = goal.NumTargetPoints
        points = list(goal.TargetPoints)
        print(numTarget, points)
        pointList = []
        print("here")
        if(not(len(points)/2 == numTarget)):
            print("bad goal message")
        else:
            for i in range(numTarget):
                tempX = points[0]
                points.pop(0)
                tempY = points[0]
                points.pop(0)
                pointList.append([tempX, tempY])
                print(pointList)
            self.setPath(pointList)

    def callBackOdom(self, data):
        self.currentTruckCord[0] = data.pose.pose.position.x
        self.currentTruckCord[1] = data.pose.pose.position.y
        self.currentTruckCord[2] = conversion_lib.quat_from_pose2eul(
            data.pose.pose.orientation)[0]
        if(self._as.is_active()):
            if (self.cordList):
                self.UpdateTargetPoints()
                output = self.getTargetCordAndDriveSpeed(
                    self.lookAheadDist, self.currentMaxDriveSpeed)
                isFinished = self.removeTargetPoint()
                if(output != "atDest" and not isFinished):
                    # print("{}, {}".format(output[0][0],output[0][1]))
                    self.rosPubMsg.linear.x = output[0]
                    self.rosPubMsg.angular.z = output[1]
                    # self.rosPubMsg.linear.x = 0.5
                    # self.rosPubMsg.angular.z = 0
                    self.logData = output
                    # publish the feedback
                    self.feedback.currentTarget = self.cordList[0]
                    self.feedback.distanceToNext = self.robotCordList[0][2]
                    self._as.publish_feedback(self.feedback)
                    # self.rosPubMsg.data = [2.2, 4]
                else:
                    self.rosPubMsg.linear.x = 0
                    self.rosPubMsg.angular.z = 0
                    self.result.AtTargetPoint = True
                    self.result.errorToPoint = 0.5
                    self.result.finalTargetPoint = self.currentTruckCord
                    self.result.angleToTarget = self.angleToTarget
                    self.currentMaxDriveSpeed = self.maxDriveSpeed
                    self._as.set_succeeded(self.result)
                print(self.rosPubMsg)

            else:
                # ros.drive.setvel(0,0)
                self.result.AtTargetPoint = False
                self.result.errorToPoint = 0.5
                self.result.finalTargetPoint = [0,0]
                self._as.set_succeeded(self.result)
                self.rosPubMsg.linear.x = 0
                self.rosPubMsg.angular.z = 0
                print("no target Point")
            looptime = time.time()-self.lastlooptime
            self.lastlooptime = time.time()
            self.pub.publish(self.rosPubMsg)
            # print("looptime: {}".format(looptime))
    def run(self):
        try:
            rospy.spin()
        except KeyboardInterrupt:
            self.rosPubMsg.linear.x = 0
            self.rosPubMsg.angular.z = 0
            self.pub.publish(self.rosPubMsg)
        except:
            print("an error occurred in purePursuit.py")

    def __del__(self):
        print("exiting Program")
        self.rosPubMsg.linear.x = 0
        self.rosPubMsg.angular.z = 0
        self.pub.publish(self.rosPubMsg)


def callBack(data):
    controller.callbackOdom(data)


if __name__ == '__main__':
    controller = PurePursuit()
    controller.run()
