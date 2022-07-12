

import math


class PID:
    
    def setPID(self, kP, kI, kD, target):
        self.pVal = kP
        self.iVal = kI
        self.dVal = kD

    def __init__(self, kP, kI, kD, target):
        self.pVal = kP
        self.iVal = kI
        self.dVal = kD

        self. error = 0
        self.totalError = 0
        self.prevError = 0

        self.maxOutput = 1
        self.minOutput = -1

        self.setPoint = target
        self.output = 0
        self.result = 0

    def setMaxMinOutput(self, output):
        self.maxOutput = output
        self.minOutput = -output

    def updatePID(self, value):
        self.error = self.setPoint - value

        if ((self.error * self.pVal < self.maxOutput) and (self.error * self.pVal > self.minOutput)):
            self.totalError += self.error
        else:
            self.totalError = 0

        self.result = (self.pVal * self.error + self.iVal * self.totalError + self.dVal * (self.error - self.prevError))
        self.prevError = self.error
        self.result = self.clamp(self.result)
        return self.result

    def clamp(self, input):
        if(input > self.maxOutput):
            return self.maxOutput

        elif(input < self.minOutput):
            return self.minOutput

        else:
            return input
    
    def setSetPoint(self, setPoint):
        self.setPoint = setPoint