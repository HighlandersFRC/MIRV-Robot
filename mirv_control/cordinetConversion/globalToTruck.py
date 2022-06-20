import math
class GlobalToTruck():
    startingCord = [40.4741340078743, -104.96915706776484]
    NextCord = [40.57359123897166, -105.08378306972632]
    EarthSMaxis = 6378137
    eccentricity = 0.08181919
    def __init__(self):
        self.startingCord = [self.degToRad(self.startingCord[0]), self.degToRad(self.startingCord[1])]
        self.NextCord = [self.degToRad(self.NextCord[0]), self.degToRad(self.NextCord[1])]
        self.startingHeading = self.degToRad(90-139)
    def getCordDelta(self, cordDel):
        distance = self.EarthSMaxis/(1-((self.eccentricity**2)*(math.sin(cordDel[0]))**2))
        X = distance*math.cos(cordDel[1])*math.cos(cordDel[0])
        Y = distance*math.cos(cordDel[1])*math.sin(cordDel[0])
        return [X,Y]
    def convertToTruck(self):
        theta = self.startingHeading
        cord = self.NextCord
        cord1Cart = self.getCordDelta(self.startingCord) 
        cord2Cart = self.getCordDelta(self.NextCord)
        delta = [cord2Cart[0]-cord1Cart[0], cord2Cart[1]-cord1Cart[1]]
        XT = delta[0]*math.cos(theta)-delta[1]*math.sin(theta)
        YT = delta[0]*math.sin(theta)+delta[1]*math.cos(theta)

        print(delta)
        print("dist between: {}".format(((XT**2)+(YT**2))**.5))
        return [XT, YT]

    def degToRad(self, angle):
        rad = angle*3.1415926/180
        return (rad)

    def test(self):
        for i in range(180):
            self.startingHeading = self.degToRad(i)
            output = self.convertToTruck()
            print("output: {}, degrees: {}".format(output, i))

if __name__ == '__main__':
    calculator = GlobalToTruck()
    calculator.test()