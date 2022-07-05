# ['$GNVTG', '', 'T', '', 'M', '0.031', 'N', '0.057', 'K', 'D*38\r\n']
# ['$GNGGA', '233550.80', '4028.4417352', 'N', '10458.1531284', 'W', '5', '12', '0.83', '1517.168', 'M', '-21.474', 'M', '0.8', '0000*68\r\n']
from sensor_msgs.msg import NavSatFix, NavSatStatus

class GGAData():
    claimedAccuracy = 1.5
    position_covariance = []
    UTCTime = 0.0
    latitude = 0.0
    longitude = 0.0
    qualityIndicator = 0
    satellitesUsed = 0
    hdop = 0.0
    altitude = 0.0
    def loadMessage(self, message):
        try:
            self.UTCTime = float(message[1])
            self.latitude = self.loadLatitude(float(message[2]), message[3])
            self.longitude = self.loadLongitude(float(message[4]), message[5])
            self.loadQualityIndicator(float(message[6]))
            self.satellitesUsed = int(message[7])
            self.hdop = float(message[8])
            self.altitude = float(message[9])
            self.computeCovariance()
        except:
            raise Exception("Invalid Input message: " + message)

    def loadLongitude(self, longitude, heading):
        degrees = int(longitude/100)
        minutes = (longitude - degrees*100)/60 
        longitude = degrees+minutes   
        if (heading == 'E'):
            return longitude
        else:
            return (longitude)*(-1)

    def loadLatitude(self, latitude, heading):
        degrees = int(latitude/100)
        minutes = ((latitude/100) - degrees)*100/60 
        latitude = degrees+minutes  
        if (heading == 'N'):
            return (latitude)
        else:
            return (latitude)*(-1)
    def loadQualityIndicator(self, data):
        if(data == 0):
            self.qualityIndicator = NavSatStatus.STATUS_NO_FIX
        elif(data == 1):
            self.qualityIndicator = NavSatStatus.STATUS_FIX
        elif(data == 2):
            self.qualityIndicator = NavSatStatus.STATUS_SBAS_FIX
        elif(data == 6):
            self.qualityIndicator = NavSatStatus.STATUS_GBAS_FIX

    def computeCovariance(self):
        covariance = (self.claimedAccuracy*self.hdop)**2
        self.position_covariance = [covariance,0,0,0,covariance,0,0,0,0]

    def getCovariance(self):
        return self.position_covariance
    def getLongitude(self):
        return self.longitude
    def getLatitude(self):
        return self.latitude
    def getAltitude(self):
        return self.altitude
    def getQualityIndicator(self):
        return self.qualityIndicator
    def getUTCTime(self):
        return self.UTCTime
    def getHdop(self):
        return self.hdop
    def getSatellitesUsed(self):
        return self.satellitesUsed
    