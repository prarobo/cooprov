#!/usr/bin/env python

""" Joystick teleoperation node for openrov
This node takes joystick inputs and converts
them into openrov velocity direction inputs
which the openrov direction node can be use to set as motor velocities"""

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Joy
from openrovClass import openrov
from velocityCalc import computeStrYawVelocity

myOpenrov = openrov()
velocityPub = rospy.Publisher('openrov_joy_vel', String, queue_size=10)

class joystick():
    def __init__(self):
        """Initialization"""
        self.JOY_AXIS0 = 0 #Left/Right
        self.JOY_AXIS1 = 1 #Forward/reverse
        self.JOY_AXIS2 = 2 #Up/Down
        self.JOY_AXIS3 = 3 #Rotate in place
        self.JOY_AXES4 = 4 #Camera servo
        self.JOY_BUTTON1 = 0 #dead man
        self.JOY_BUTTON2 = 1 #shutdown
        self.JOY_BUTTON4 = 3 #led toggle
        self.JOY_BUTTON5 = 4 #auto depth toggle
        self.JOY_BUTTON6 = 5 #auto heading toggle
        self.JOY_BUTTON7 = 6 #cam servo deadman
        self.JOY_BUTTONSE = 10 #imu reset
        self.JOY_BUTTONST = 11 #pressure sensor reset   
        
        self.JOY_AXES_MAX_VAL = 1
        self.JOY_AXES_ORIGIN = 0
        self.JOY_AXES_MIN_VAL = -1
        
    def configureOpenrovControls(self):
        myOpenrov.joyDeadMan = self.JOY_BUTTON1
        myOpenrov.joyCamDeadMan = self.JOY_BUTTON7
        myOpenrov.joyAxisForwardReverse = self.JOY_AXIS1
        myOpenrov.joyAxisLeftRight = self.JOY_AXIS0
        myOpenrov.joyAxisUpDown = self.JOY_AXIS2
        myOpenrov.joyAxisAClocCloc = self.JOY_AXIS3
        myOpenrov.joyAxisServo = self.JOY_AXES4
        myOpenrov.joyBtnLedStatus = self.JOY_BUTTON4
        myOpenrov.joyBtnImuReset = self.JOY_BUTTONSE
        myOpenrov.joyBtnPresReset = self.JOY_BUTTONST
        myOpenrov.joyBtnAutoDepth = self.JOY_BUTTON5
        myOpenrov.joyBtnAutoHeading = self.JOY_BUTTON6
        myOpenrov.joyBtnShutdown = self.JOY_BUTTON2       
        
    def joyCallback(self,joyInput):
        #print joyInput
        
        statIndex = myOpenrov.defaultIndex
        
        #shutdown
        if joyInput.buttons[myOpenrov.joyBtnShutdown] == 1:
            rospy.loginfo("TELEOP:Shutdown button pressed")
            statIndex = myOpenrov.shutdownIndex
            statValue = 0
            myOpenrov.topMotor.speed = myOpenrov.topMotor.zeroSpeed
            myOpenrov.starMotor.speed = myOpenrov.starMotor.zeroSpeed
            myOpenrov.portMotor.speed = myOpenrov.portMotor.zeroSpeed
            myOpenrov.ledStatus = 0

        #led
        elif joyInput.buttons[myOpenrov.joyBtnLedStatus] == 1:
            if myOpenrov.ledStatus == 0:
                rospy.loginfo("TELEOP:Led button pressed: switched on")
                myOpenrov.ledStatus = 1
                statIndex = myOpenrov.ledIndex
                statValue = 1
            else:
                rospy.loginfo("TELEOP:Led button pressed: switched off")
                myOpenrov.ledStatus = 0
                statIndex = myOpenrov.ledIndex
                statValue = 0
                
        #Camera servo
        elif joyInput.buttons[myOpenrov.joyCamDeadMan] == 1 and \
                    joyInput.buttons[myOpenrov.joyAxisCamServo] > 0:
            rospy.loginfo("Servo rotation activated")
            myOpenrov.setSpeed(myOpenrov.camMotor,abs(joy.Input.buttons[myOpenrov.joyAxisCamServo]),1)
            
                
        #imu reset
        elif joyInput.buttons[myOpenrov.joyBtnImuReset] == 1:
            rospy.loginfo("TELEOP:Imu reset button pressed")
            statIndex = myOpenrov.imuResetIndex
            statValue = 0
                
        #pressure sensor reset
        elif joyInput.buttons[myOpenrov.joyBtnPresReset] == 1:
            rospy.loginfo("TELEOP:Pressure sensor reset button pressed")
            statIndex = myOpenrov.presResetIndex
            statValue = 0
            
        #auto depth
        elif joyInput.buttons[myOpenrov.joyBtnAutoDepth] == 1:
            if myOpenrov.autoDepth == 0:
                rospy.loginfo("TELEOP:Auto depth button pressed: switched on")
                myOpenrov.autoDepth = 1
                statIndex = myOpenrov.autoDepthIndex
                statValue = 1
            else:
                rospy.loginfo("TELEOP:Auto depth button pressed: switched off")
                myOpenrov.autoDepth = 0
                statIndex = myOpenrov.autoDepthIndex
                statValue = 0
        
        #auto heading
        elif joyInput.buttons[myOpenrov.joyBtnAutoHeading] == 1:
            if myOpenrov.autoHeading == 0:
                rospy.loginfo("TELEOP:Auto heading button pressed: switched on")
                myOpenrov.autoHeading = 1
                statIndex = myOpenrov.autoHeadingIndex
                statValue = 1
            else:
                rospy.loginfo("TELEOP:Auto heading pressed: switched off")
                myOpenrov.autoHeading = 0
                statIndex = myOpenrov.autoHeadingIndex
                statValue = 0
                
        #checking for velocity inputs
        else:
            
            if joyInput.buttons[myOpenrov.joyDeadMan] == 1:
                self.setVelocity(joyInput)           
            
        velocityStr = self.getVelocityString(myOpenrov.topMotor.speed)+ \
                      self.getVelocityString(myOpenrov.starMotor.speed)+ \
                      self.getVelocityString(myOpenrov.portMotor.speed)   
                      
        camServoStr = self.getVelocityString(myOpenrov.camMotor.speed)
        
        # Setting status string
        if statIndex != myOpenrov.defaultIndex:
            statStr = str(statIndex*100+statValue)
        else:
            statStr = 3*str(myOpenrov.defaultIndex)
            
        # Publishing velocity
        pubStr = statStr+camServoStr+velocityStr
        if pubStr != 15*"0":       
            velocityPub.publish(pubStr)
        return
    
    def setVelocity(self, joyInput):
        """Setting the motor velocities"""
        
         #Top motor
        if joyInput.axes[myOpenrov.joyAxisUpDown] != 0:
            self.setTopVelocity(joyInput.axes[myOpenrov.joyAxisUpDown])
            
        #Turn in place
        if joyInput.axes[myOpenrov.joyAxisAClocCloc] != 0:
            self.setTurnInPlace(joyInput.axes[myOpenrov.joyAxisAClocCloc])
            
        #Straight and yaw
        else:
            self.setStrAndYaw(joyInput.axes[myOpenrov.joyAxisForwardReverse], \
                                joyInput.axes[myOpenrov.joyAxisLeftRight])
             
        return
    
    def setTopVelocity(self, joyVal):
        """Setting top motor velocity"""
        
        # Going down
        if joyVal > 0:
            myOpenrov.setSpeed(myOpenrov.topMotor,abs(joyVal),1)
            
        # Coming up
        else:
            myOpenrov.setSpeed(myOpenrov.topMotor,abs(joyVal),-1)
        return

    def setTurnInPlace(self, joyVal):
        """Setting turn in place velocity"""
        
        #Turn in place anti-clockwise
        if joyVal > 0:
            myOpenrov.setSpeed(myOpenrov.portMotor,abs(joyVal),1)
            myOpenrov.setSpeed(myOpenrov.starMotor,abs(joyVal),-1)
            
        #Turn in place clockwise
        else:
            myOpenrov.setSpeed(myOpenrov.starMotor,abs(joyVal),1)
            myOpenrov.setSpeed(myOpenrov.portMotor,abs(joyVal),-1)
        return
    
    def setStrAndYaw(self, joyValX, joyValY):    
        """Setting straight and yaw velocity"""
        joyStarVal, joyStarDir, joyPortVal, joyPortDir = computeStrYawVelocity(joyValX, joyValY)
        
        #Star velocity
        myOpenrov.setSpeed(myOpenrov.starMotor, joyStarVal, joyStarDir)
        
        #Port velocity
        myOpenrov.setSpeed(myOpenrov.portMotor, joyPortVal, joyPortDir)
            
    def getVelocityString(self, velocityVal):
        """Composing motor velocities into a string"""
        
        #setting the velocity string field length
        if velocityVal < 10:
            velocityStr = "00"+str(velocityVal)
        elif velocityVal < 100:
            velocityStr = "0"+str(velocityVal)
        else:
            velocityStr = str(velocityVal)
        
        #removing trailing zeros and decimal point    
        if velocityStr[-2:]=='.0':
            velocityStr = velocityStr[:-2]
            
        return velocityStr
        

if __name__ == '__main__':
    try:
        myJoystick = joystick()
        myJoystick.configureOpenrovControls()        
        
        rospy.Subscriber("joy", Joy, myJoystick.joyCallback)
        rospy.init_node('openrov_teleop_joy', anonymous=True)
        r = rospy.Rate(30) # 10hz

        while not rospy.is_shutdown():
            r.sleep()
    except rospy.ROSInterruptException: 
        pass