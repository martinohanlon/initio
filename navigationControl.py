#Navigation class to accurately move initio robot
#Martin O'Hanlon
#www.stuffaboutcode.com

#import modules
import sys
import motorControl
import time
import RPi.GPIO as GPIO
import math
import thread

#states
STATEMOVING = 1
STATESTOPPED = 0

#Constant of Proportionality
# multipler used to convert difference in motor position to power change
KP = 2
KPTURN = 3

#Turning Inside Wheel Fudge Factor,
# this is multiplied to the inside wheel initial speed as most motor speeds arent linear
# (i.e. at 50% power a motor doesnt turn half the speed of 100%)
TURNINSIDEWHEELFUDGE = 1

#Wheel circumference
#(in mm)
WHEELCIRCUMFERENCEMM = 174.0
#(in encoder ticks)
WHEELCIRCUMFERENCE = 36.0

#Encoder to mm ratio (wheel circumference in mm / wheel circumference to encoder ticks)
ENCODERTOMM = WHEELCIRCUMFERENCEMM / WHEELCIRCUMFERENCE

#Wheel gap - gap between the 2 wheels (or length of axel)
#(in mm)
WHEELGAPMM = 136.0
#(in encoder ticks)
WHEELGAP = WHEELGAPMM / ENCODERTOMM

class NavigationController:
    #initialise - motors is the MotorControl object
    def __init__(self, motors, distanceInMM = False):
        self.motors = motors
        self.state = STATESTOPPED

        #if distanceinMM is set to True all distance are consider to be in millimeters
        # if False all distances are in encoder ticks
        self.distanceInMM = distanceInMM

    #navigation state property
    @property
    def state(self):
        return self.state

    #motors property
    @property
    def motors(self):
        return self.state

    #move the robot in a straight line
    def straight(self, power, distance = None, ownThread = False):
        # if distance is not passed, continues forever and launches a seperate thread
        # if distance is passed, loops until distance is reached
        if distance == None:
            #set a really long distance - this is a 'bit' dirty...  But simple!
            distance = 99999999999
            #call it in its own thread
            ownThread = True
            
        if ownThread:
            #call it in its own thread
            thread.start_new_thread(self._straight,(power, distance))
        else:
            self._straight(power, distance)

    #move the robot in a staight line
    def _straight(self, power, distance):

        #convert distance
        if self.distanceInMM: distance = self._convertMillimetersToTicks(distance)
        
        #if the state is moving, set it to STOP
        if(self.state != STATESTOPPED): self.stop()
        
        #turn on both motors
        self.motors.start(power, power)

        #change state to moving
        self.state = STATEMOVING

        #keep track of the last motor error, so we only change if we need too
        lastMotorError = 0
        
        #loop until the distance is reached or it has been STOPPED, correcting the power so the robot goes straight
        while(self.motors.motorA.currentTicks < distance and self.state == STATEMOVING):
            
            #get the number of ticks of each motor
            #get the error by minusing one from the other
            motorError = self.motors.motorA.currentTicks - self.motors.motorB.currentTicks
            #print("motorError = " + str(motorError))

            #only change if the motorError has changed
            if(motorError != lastMotorError):

                #work out the value to slow the motor down by using the KP
                powerChange = (motorError * KP)
                #print("powerChange = " + str(powerChange))

                #in the unlikely event they are equal!
                if(powerChange == 0):
                    self.motors.start(power, power)
                else:
                    #if its going backward, turn the power change into a negative
                    if power < 0: powerChange * -1
                        
                    #if its a positive number
                    if(motorError > 0):
                        #set motor A to power - 'slow down value'
                        #set motor B to power
                        self.motors.start(power - powerChange, power)
                    #if its a negative number
                    if(motorError < 0):
                        #set motor A to power
                        #set motor B to power - 'slow down value'
                        self.motors.start(power, power - powerChange)

            #update last motor error
            lastMotorError = motorError

        # if they havent already been stopped - stop the motors
        self.stop()

    #turn the robot
    # if radius is not passed, robot turns on its axis)
    def turn(self, power, angle, radius = 0, ownThread = False):
        if ownThread:
            thread.start_new_thread(self._turn,(power, angle, radius))
        else:
            self._turn(power, angle, radius)

    def _turn(self, power, angle, radius):

        #convert radius
        if self.distanceInMM: radius = self._convertMillimetersToTicks(radius)

        #calculate lengths of the turning arc for both wheels
        wheelALength = (angle / 360.0) * (2 * math.pi *(radius + (WHEELGAP / 2)))
        wheelBLength = (angle / 360.0) * (2 * math.pi *(radius - (WHEELGAP / 2)))

        #debug
        print "wheelALength" + str(wheelALength)
        print "wheelBLength" + str(wheelBLength)

        #work out the outside and inside wheels
        #the wheel which is going further is the outside wheel
        # and calc the speed differentiator (i.e. the factor between the fast outside wheel and the slow inside wheel)
        if(wheelALength >= wheelBLength):
            speedDiff = wheelALength / wheelBLength 

            outsideLength = wheelALength
            insideLength = wheelBLength

            outsideMotor = motors.motorA
            insideMotor = motors.motorB
        else:
            speedDiff = wheelBLength / wheelALength

            outsideLength = wheelBLength
            insideLength = wheelALength

            outsideMotor = motors.motorB
            insideMotor = motors.motorA

        #debug
        #print speedDiff

        #calc the speed based on the power and the differentiator
        outsidePower = power
        insidePower = (power / speedDiff) * TURNINSIDEWHEELFUDGE
        #debug
        print outsidePower
        print insidePower
        
        #is the length negative?  If so negate the power
        if outsideLength < 0: outsidePower = outsidePower * -1
        if insideLength < 0: insidePower = insidePower * -1

        #change state to moving
        self.state = STATEMOVING

        #start the motors
        outsideMotor.start(outsidePower)
        insideMotor.start(insidePower)

        #keep track of the last error, only change, if the error has changed
        lastMotorError = 0

        #loop until the outside motor has covered the distance
        while(outsideMotor.currentTicks < abs(outsideLength)):

            #work out how far the inside wheel should have travelled?
            insideWheelTarget = round(outsideMotor.currentTicks / abs(speedDiff))
            #debug
            #print "insideWheelTarget " + str(insideWheelTarget)
            #print "insideMotor.currentTicks " + str(insideMotor.currentTicks)

            motorError = insideMotor.currentTicks - insideWheelTarget
            
            
            #has the error changed?
            if motorError != lastMotorError:
                #debug
                #print "motorError " + str(motorError)

                #work out the value to slow the motor down by using the KP
                powerChange = (abs(motorError) * KPTURN)
                #debug
                #print "powerChange " + str(powerChange)
                
                #if its going backward, turn the power change into a negative
                if insidePower < 0: powerChange * -1

                #if the error > 0 then slow down
                if(motorError > 0):
                    insideMotor.start(insidePower - powerChange)
                    
                #if the error < 1 then speed up
                if(motorError < 0):
                    insideMotor.start(insidePower + powerChange)
            
            lastMotorError = motorError

        self.stop()
        
    #stops the robot
    def stop(self):
        self.motors.stop()
        self.state = STATESTOPPED

    def _convertMillimetersToTicks(self, millimeters):
        print ENCODERTOMM
        return int(round(millimeters / ENCODERTOMM,0))

#tests
if __name__ == '__main__':

    try:
        #setup gpio
        GPIO.setmode(GPIO.BOARD)

        #create motor control
        motors = motorControl.MotorController()

        #create navigation control, use True for distances in millimeters
        nav = NavigationController(motors, True)

        #time.sleep(10)

        #run a straight line just through motor control
        #print("straight line, no correction")
        #motors.start(100,100)
        #time.sleep(10)
        #stop
        #print("stop")
        #motors.stop()
        #get length
        #print("encoder ticks")
        #print(motors.motorA.totalTicks)
        #print(motors.motorB.totalTicks)

        #time.sleep(10)

        #reset encoder ticks
        #motors.motorA.resetTotalTicks()
        #motors.motorB.resetTotalTicks()
        
        #run a straight line through nav control
        #print("straight line, with correction")
        #nav.straight(100, 2000)
        #get length
        #print("encoder ticks")
        #print(motors.motorA.totalTicks)
        #print(motors.motorB.totalTicks)

        #run a straight line threaded
        #print("straight line threaded")
        time.sleep(5)
        nav.straight(100, 10000)
        time.sleep(3)
        nav.stop()

        #turn 90 degrees with a 150 radius at 80% power
        #nav.turn(100, 90, 1000)
        #print("encoder ticks")
        #print(motors.motorA.totalTicks)
        #print(motors.motorB.totalTicks)

    #Ctrl C
    except KeyboardInterrupt:
        print "User cancelled"

    #Error
    except:
        print "Unexpected error:", sys.exc_info()[0]
        raise

    finally:
        print ("cleanup")
        #cleanup gpio
        GPIO.cleanup()
 

