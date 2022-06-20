import math
import time
trueNorth = [0,1]
newCord = [40.47403755759644, -104.96908805060465]
prevCord = [40.473998790805695, -104.9690163015153]
vector = []
theta = 0.0

vector = [newCord[0]-prevCord[0],newCord[1]-prevCord[1]]
theta = math.acos(vector[1]/((vector[0]**2+vector[1]**2)**0.5))
if (vector[0] > 0):
    theta = 360-(theta*180/3.1415926)
else:
    theta = (theta*180/3.1415926)
print(theta)
