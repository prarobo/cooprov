""" Computing straight and yaw velocities"""

import numpy as np

strDivPercent = 0.5
yawDivPercent = 1-strDivPercent
piby2 = np.pi/2
pi = np.pi
zeroErrorVal = 0.00001
deadZone = 0.1 

def computeStrYawVelocity(joyValX, joyValY):
    
    # Mapping from unit square to unit circle
    circX, circY = squareToCircleMapping(joyValX, joyValY)
    if abs(circX)<deadZone:   circX=0
    if abs(circY)<deadZone:   circY=0    
        
    #Input magnitude
    inputMag = np.sqrt(circX**2+circY**2)
    #Splitting the velocity into linear using sin theta and cos theta
    if inputMag>zeroErrorVal:
        linearDir = circX/inputMag
        angularDir = circY/inputMag
    else:
        linearDir = 0
        angularDir = 0
    
    #Setting motor directions
    joyStarDir = cmp(linearDir,0) if circX != 0 else 1
    joyPortDir = cmp(linearDir,0) if circX != 0 else 1
    
    #Setting turn directions (clockwise -1/ anti 1)
    turnDir = cmp(angularDir,0) if circY != 0 else 1    
    
    #Setting forward/backward velocities of the motors
    linearRatio = linearDir**2   
    strStarVal = linearRatio*inputMag    
    strPortVal = linearRatio*inputMag  

    #Setting forward/backward velocities of the motors
    angularRatio = angularDir**2
    if turnDir == -1:   #clockwise   
        turnPortVal = angularRatio*inputMag
        turnStarVal = 0
    else: #anti clockwise
        turnStarVal = angularRatio*inputMag
        turnPortVal = 0
                
    joyStarVal = strStarVal+turnStarVal
    joyPortVal = strPortVal+turnPortVal
    
    #print "JoyVal X = ", joyValX, "\tJoyVal Y = ", joyValY
    #print "Circular X = ", circX, "\tCircular Y = ", circY, "\tInput Magnitude = ", inputMag   
    #print "linearDir = ", linearDir, "\tangularDirection = ", angularDir
    #print "linearRatio = ", linearRatio, "\tangularRatio = ", angularRatio
    #print "strStarVal = ", strStarVal, "\tstrStarDir = ", joyStarDir
    #print "strPortVal = ", strPortVal, "\tstrPortDir = ", joyPortDir
    #print "turnStarVal = ", turnStarVal,  "\tturnPortVal = ", turnPortVal ,"\tturnDir = ", turnDir
    #print "joyStarVal = ", joyStarVal, "\tjoyPortVal = ", joyPortVal
    
    return joyStarVal, joyStarDir, joyPortVal, joyPortDir
    
    
    

    
            
    '''angularDir = np.arctan2(joyValY, joyValX)
    if angularDir<0: angularDir = angularDir+2*np.pi
    angularLen = np.sqrt(joyValX**2+joyValY**2)
    circularLen = abs(np.cos(angularDir)) if abs(np.cos(angularDir))>zeroErrorVal else 0
    #scaledLen = angularLen/circularLen if circularLen !=0 else 1
    scaledLen = 1 if angularLen>1  else angularLen
    
    yawPercent = yawDivPercent*scaledLen
    angStar = 0
    angPort = 0
    
    print "angularLen = ", angularLen, "circularLen = ", circularLen, "scaledLen = ", scaledLen 
    print "angularDir = ", angularDir, "yawPercent = ", yawPercent
            
    #First quadrant
    if angularDir >= 0 and angularDir <= piby2:
        angStar = (piby2-angularDir)/piby2
        angPort = 1

        joyStarDir = 1
        joyPortDir = 1
        
        print "I : ","angStar = ", angStar, "angPort = ",angPort
    
    #Fourth quadrant    
    elif angularDir >= 3*piby2 and angularDir < 2*pi:
        angStar = 1
        angPort = (angularDir-3*piby2)/piby2
        
        joyStarDir = 1
        joyPortDir = 1
        
        print "IV : ","angStar = ", angStar, "angPort = ",angPort
        
    #Second quadrant
    elif angularDir > piby2 and angularDir < pi:
        angStar = (piby2-angularDir)/piby2
        angPort = 1

        joyStarDir = -1
        joyPortDir = -1
        
        print "II : ","angStar = ", angStar, "angPort = ",angPort

    #Third quadrant
    elif angularDir >= pi and angularDir < 3*piby2:
        angStar = 1
        angPort = (3*piby2-angularDir)/piby2

        joyStarDir = -1
        joyPortDir = -1
        
        print "III : ","angStar = ", angStar, "angPort = ",angPort
                
    joyStarVal = joyStarVal + angStar * yawPercent
    joyPortVal = joyPortVal + angPort * yawPercent
            
    print "inValX = ", joyValX, "inValY = ", joyValY 
    print "outStarVal = ", joyStarVal, "outStarDir = ", joyStarDir
    print "outPortVal = ", joyPortVal, "outPortDir = ", joyPortDir
    
    return joyStarVal, joyStarDir, joyPortVal, joyPortDir'''

def squareToCircleMapping(sqX, sqY):
    circX=sqX*np.sqrt(1-sqY**2/2)
    circY=sqY*np.sqrt(1-sqX**2/2)
    return circX, circY