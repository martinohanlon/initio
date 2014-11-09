#Program to move initio robot using xbox 360 controller
#Martin O'Hanlon
#www.stuffaboutcode.com

#import modules
import sys
import motorControl
import RPi.GPIO as GPIO
import math
import XboxController
import time

class InitioXboxControl:

    def __init__(self):

        #setup gpio
        GPIO.setmode(GPIO.BOARD)

        #create motor control
        self.motors = motorControl.MotorController()
        
        #setup controller values
        self.xValue = 0
        self.yValue = 0

        #create xbox controller class
        self.xboxCont = XboxController.XboxController(deadzone = 30,
                                                 scale = 100,
                                                 invertYAxis = True)

        #setup call backs
        self.xboxCont.setupControlCallback(self.xboxCont.XboxControls.LTHUMBX, self.leftThumbX)
        self.xboxCont.setupControlCallback(self.xboxCont.XboxControls.LTHUMBY, self.leftThumbY)
        self.xboxCont.setupControlCallback(self.xboxCont.XboxControls.BACK, self.backButton)

        #start the controller
        self.xboxCont.start()

        self.running = True

    #call back funtions for left thumb stick
    def leftThumbX(self, xValue):
        self.xValue = xValue
        self.updateMotors()

    def leftThumbY(self, yValue):
        self.yValue = yValue
        self.updateMotors()

    def backButton(self, value):
        self.stop()

    def updateMotors(self):
        #calculate angle and length
        # angle will determine direction
        angle = math.degrees(math.atan2(self.xValue, self.yValue))
        # length will determine power
        length = math.hypot(self.xValue, self.yValue)

        if angle >= 0 and angle <= 45:
            #motor A is 0 - 100 based on length
            #motor B is 100 - 0 based on length and angle
            powerA = length
            powerB = length * ((45 - angle) / 45)
            powerA = max(0,min(100,round(powerA)))
            powerB = max(0,min(100,round(powerB)))
        elif angle > 45 and angle <= 90:
            #motor A is 0 - 100 based on length
            #motor B is 0 - -100 based on length and angle
            powerA = length
            powerB = (length * -1) * ((angle - 45) / 45)
            powerA = max(0,min(100,round(powerA)))
            powerB = max(-100,min(0,round(powerB)))
        elif angle > 90 and angle <= 135:
            #motor A is 100 - 0 based on length and angle
            #motor B is 0 - -100 based on length
            powerA = length * ((90 - angle) / 45 + 1)
            powerB = (length * -1)
            powerA = max(0,min(100,round(powerA)))
            powerB = max(-100,min(0,round(powerB)))
        elif angle > 135 and angle <= 180:
            #motor A is 0 - -100 based on length and angle
            #motor B is 0 - -100 based on length
            powerA = (length * -1) * ((angle - 135) / 45)
            powerB = (length * -1)
            powerA = max(-100,min(0,round(powerA)))
            powerB = max(-100,min(0,round(powerB)))   
        elif angle < 0 and angle >= -45:
            #motor A is 100 - 0 based on length and angle
            #motor B is 0 - 100 based on length
            powerA = length * ((-45 - angle) / -45)
            powerB = length
            powerA = max(0,min(100,round(powerA)))
            powerB = max(0,min(100,round(powerB)))
        elif angle < -45 and angle >= -90:
            #motor A is 0 - -100 based on length and angle
            #motor B is 0 - 100 based on length
            powerA = (length * -1) * ((angle - -45) / -45)
            powerB = length 
            powerA = max(-100,min(0,round(powerA)))
            powerB = max(0,min(100,round(powerB)))
        elif angle < -90 and angle >= -135:
            #motor A is 0 - -100 based on length
            #motor B is 100 - 0 based on length and angle
            powerA = (length * -1)
            powerB = length * ((-90 - angle) / -45 + 1)
            powerA = max(-100,min(0,round(powerA)))
            powerB = max(0,min(100,round(powerB)))
        elif angle < -135 and angle >= -180:
            #motor A is 0 - -100 based on length
            #motor B is 0 - -100 based on length and angle
            powerA = (length * -1)
            powerB = (length * -1) * ((angle - -135) / -45)
            powerA = max(-100,min(0,round(powerA)))
            powerB = max(-100,min(0,round(powerB)))
            
        #debug
        #print powerA
        #print powerB

        #if the power is 0 stop the motors
        if powerA == 0 and powerB == 0:
            self.motors.stop()
        #otherwise start them up
        else:
            self.motors.start(powerA, powerB)

    def stop(self):
        GPIO.cleanup()
        self.xboxCont.stop()
        self.running = False


if __name__ == '__main__':

    print ("started")
    try:
        #create class
        initioCont = InitioXboxControl()
        while initioCont.running:
            time.sleep(0.1)

    #Ctrl C
    except KeyboardInterrupt:
        print "User cancelled"

    #Error
    except:
        print "Unexpected error:", sys.exc_info()[0]
        raise
    
    finally:
        print 
        print ("stop")
        #if its still running (probably because an error occured, stop it
        if initioCont.running == True: initioCont.stop()
