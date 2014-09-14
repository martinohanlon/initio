#Navigation class to accurately move initio robot
#Martin O'Hanlon
#www.stuffaboutcode.com

#import modules
import sys
import motorControl
import time
import RPi.GPIO as GPIO
import math
import xbox_read

if __name__ == '__main__':

    print ("started")
    try:
        #setup gpio
        GPIO.setmode(GPIO.BOARD)

        #create motor control
        motors = motorControl.MotorController()

        #setup xbox controller values
        x1 = 0
        y1 = 0
        #TODO
        #yButton = False
        #bButton = False

        #read xbox controller event stream

        for event in xbox_read.event_stream(deadzone=7000, scale=101):

            #TODO - if the Y button and B button are both pressed, exit
            #it does exit but the program does finish due to xbox_read not exiting!
            #if event.key=='Y': yButton = event.is_press()
            #if event.key=='B': bButton = event.is_press()
            #if yButton and bButton: break

            #analogue stick
            if event.key=='X1':
                x1 = event.value
                #print("X1 - " + str(event.value))
            if event.key=='Y1':
                y1 = event.value
                #print("Y1 - " + str(event.value))

            #calculate angle and length
            # angle will determine direction
            angle = math.degrees(math.atan2(x1, y1))
            #  if angles are negative turn into 0-360
            #if angle < 0: angle = angle + 360
            # length will determine power
            length = math.hypot(x1, y1)

            #debug
            #print angle
            #print length

            #work out actrant (like quadrants but eights!)
            # - where the stick is pointing determins how the motors are controlled
            # - calculate the powers of motor a and b
            # disclaimer - Im not convinced this is the easiest or most efficient way of doing this, but I cant think of a better way!

            #Controller - Right hemi-sphere
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

            #Controller - Left hemi-sphere    
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
                motors.stop()
            #otherwise start them up
            else:
                motors.start(powerA, powerB)

    #Ctrl C
    except KeyboardInterrupt:
        print "User cancelled"

    #Error
    except:
        print "Unexpected error:", sys.exc_info()[0]
        raise

    finally:
        print 
        print ("cleanup")
        #cleanup gpio
        GPIO.cleanup()
 
