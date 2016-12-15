"""Motor class"""

class motor():
    def __init__(self):
        """Initialization""" 
        self.id = 0;
        
        self.biDirectional = True
        self.zeroSpeed = 95
        
        self.forwardStart = 95
        self.forwardEnd = 180
        self.forwardIncreaseDirection = 1
        
        self.reverseStart = 85
        self.reverseEnd = 0
        self.reverseIncreaseDirection = -1
        
        self.deadBandStart = 86
        self.deadBandEnd = 94
        
        self.maxVal = 180
        self.minVal = 0
        
        self.speed = self.forwardStart
        return
    
    def checkLimits(self, velocityVal):
        velocityVal = round(velocityVal)
        if velocityVal < self.minVal: return self.minVal
        if velocityVal > self.maxVal: return self.maxVal
        return velocityVal                
        
if __name__=="__main__":
    myMotor = motor(); 
    print "Created motor object"     