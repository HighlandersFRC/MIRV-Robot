# ['$GNVTG', '', 'T', '', 'M', '0.031', 'N', '0.057', 'K', 'D*38\r\n']
# ['$GNGGA', '233550.80', '4028.4417352', 'N', '10458.1531284', 'W', '5', '12', '0.83', '1517.168', 'M', '-21.474', 'M', '0.8', '0000*68\r\n']

class GGAData():
    UTCTime = 0.0
    latitude = 0.0
    longitude = 0.0
    qualityIndicator = 0
    satellitesUsed = 0
    altitude = 0.0
    def loadMessage(self, message):
        try:
            self.UTCTime = float(message[1])
            self.latitude = self.loadLatitude(message[2], message[3])
            self.longitude = self.loadLongitude(message[4], message[5])
            self.qualityIndicator = float(message[6])
            self.satellitesUsed = int(message[7])
            self.altitude = float(message[9])
        except:
            raise Exception("Invalid Input message: " + message)

    def loadLongitude(self, longitude, heading):    
        if (heading == 'E'):
            return float(longitude)/100
        else:
            return (float(longitude)*(-1))/100

    def loadLatitude(self, latitude, heading):
        if (heading == 'N'):
            return float(latitude)/100
        else:
            return (float(latitude)*(-1))/100
    
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
    def getSatellitesUsed(self):
        return self.satellitesUsed
    