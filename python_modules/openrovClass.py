"""Openrov class"""

from motorClass import *

class openrov():
    def __init__(self):
        """Initialization""" 
        #State variables
        self.ledStatus = 0
        self.autoDepth = 0
        self.autoHeading = 0
        
        # Control ID
        self.controlID = 1;
        
        #Motors
        self.starMotor = motor()
        self.portMotor = motor()
        self.topMotor = motor()
        self.camMotor = motor()
        
        #Joystick controls
        self.joyDeadMan = -1
        self.joyCamDeadMan = -1
        self.joyAxisForwardReverse = -1
        self.joyAxisLeftRight = -1
        self.joyRotateAClocCloc = -1
        self.joyAxisUpDown = -1
        self.joyAxisCamServo = -1
        self.joyBtnLedStatus = -1
        self.joyBtnImuReset = -1
        self.joyBtnPresReset = -1
        self.joyBtnAutoDepth = -1
        self.joyBtnAutoHeading = -1
        self.joyBtnShutdown = -1
        
        self.configureMotors()
        self.configureStatusIndex()
               
    def configureMotors(self):
        "Configuring the motor constants"
        
        # Starboard motor constants
        self.starMotor.id = 2
        self.starMotor.biDirectional = True
        self.starMotor.forwardStart = 95
        self.starMotor.forwardEnd = 150 #180
        self.starMotor.forwardIncreaseDirection = 1        
        self.starMotor.reverseStart = 85
        self.starMotor.reverseEnd = 50 #20
        self.starMotor.reverseIncreaseDirection = -1      
        self.starMotor.deadBandStart = 86
        self.starMotor.deadBandEnd = 94
        self.starMotor.speed = 95

        # Port motor constants
        self.portMotor.id = 3
        self.portMotor.biDirectional = True
        self.portMotor.forwardStart = 95
        self.portMotor.forwardEnd = 150 #180
        self.portMotor.forwardIncreaseDirection = 1        
        self.portMotor.reverseStart = 85
        self.portMotor.reverseEnd = 50 #20
        self.portMotor.reverseIncreaseDirection = -1      
        self.portMotor.deadBandStart = 86
        self.portMotor.deadBandEnd = 94
        self.portMotor.speed = 95
        
        # Top motor constants
        self.topMotor.id = 4
        self.topMotor.biDirectional = True
        self.topMotor.forwardStart = 95
        self.topMotor.forwardEnd = 150 #180
        self.topMotor.forwardIncreaseDirection = 1        
        self.topMotor.reverseStart = 85
        self.topMotor.reverseEnd = 50 #20
        self.topMotor.reverseIncreaseDirection = -1      
        self.topMotor.deadBandStart = 86
        self.topMotor.deadBandEnd = 94
        self.topMotor.speed = 95  
        
        # Camera servo motor constants  
        self.camMotor.id = 5
        self.camMotor.biDirectional = False
        self.camMotor.forwardStart = 0
        self.camMotor.forwardEnd = 180
        self.camMotor.forwardIncreaseDirection = 1        
        self.camMotor.reverseStart = -1
        self.camMotor.reverseEnd = -1
        self.camMotor.reverseIncreaseDirection = -1      
        self.camMotor.deadBandStart = -1
        self.camMotor.deadBandEnd = -1
        self.camMotor.speed = 0  

        return
    
    def configureStatusIndex(self):
        self.defaultIndex = 0
        self.shutdownIndex = 1
        self.ledIndex = 2
        self.imuResetIndex = 3
        self.presResetIndex = 4
        self.autoDepthIndex = 5
        self.autoHeadingIndex = 6
        
    def setSpeed(self, currMotor, percentSpeed, direction):
        "Setting motor speed"
        if direction == 1:
            motorRange = abs(currMotor.forwardStart-currMotor.forwardEnd)
            motorInit = currMotor.forwardStart
            motorDiff = currMotor.forwardIncreaseDirection*motorRange*percentSpeed
            currMotor.speed = currMotor.checkLimits( motorInit+motorDiff)
        elif direction == -1:
            motorRange = abs(currMotor.reverseStart-currMotor.reverseEnd)
            motorInit = currMotor.reverseStart
            motorDiff = currMotor.reverseIncreaseDirection*motorRange*percentSpeed
            currMotor.speed = currMotor.checkLimits( motorInit+motorDiff)
        else:
            print "Bad motor direction"
            
        #print direction,motorRange, motorInit, motorDiff, currMotor.speed
        return        
        
if __name__=="__main__":
    myOpenrov = openrov(); 
    print "Created openrov object"   
    
    myOpenrov.configureMotors()
    print "Configured openrov motors successfully"
    
     
         
