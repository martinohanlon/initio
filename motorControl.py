#PiRoCon - 4Tronix Initio - Motor Controller
#Martin O'Hanlon
#www.stuffaboutcode.com

import sys
import time
import RPi.GPIO as GPIO

#motor pins
MOTORAFWRDPIN = 19
MOTORABWRDPIN = 21
MOTORBFWRDPIN = 24
MOTORBBWRDPIN = 26
#encoder pins
MOTORAENCODERPIN = 7
MOTORBENCODERPIN = 11

#motor states
STATEFORWARD = 1
STATESTOPPED = 0
STATEBACKWARD = -1

class MotorController:
    def __init__(self,
                 motorAForwardPin = MOTORAFWRDPIN,
                 motorABackwardPin = MOTORABWRDPIN,
                 motorBForwardPin = MOTORBFWRDPIN,
                 motorBBackwardPin = MOTORBBWRDPIN,
                 motorAEncoderPin = MOTORAENCODERPIN,
                 motorBEncoderPin = MOTORBENCODERPIN,):

        #setup motor classes
        self.motorA = Motor(motorAForwardPin, motorABackwardPin, motorAEncoderPin)
        self.motorB = Motor(motorBForwardPin, motorBBackwardPin, motorBEncoderPin)

    #motor properties
    @property
    def motorA(self):
        return self.motorA

    @property
    def motorB(self):
        return self.motorB

    #start
    def start(self, powerA, powerB = None):
        #if a second power isnt passed in, both motors are set the same
        if powerB == None: powerB = powerA
        self.motorA.start(powerA)
        self.motorB.start(powerB)

    #stop
    def stop(self):
        self.motorA.stop()
        self.motorB.stop()

    #rotate left
    def rotateLeft(self, power):
        self.motorA.start(power * -1)
        self.motorB.start(power)

    #rotate right
    def rotateRight(self, power):
        self.motorA.start(power)
        self.motorB.start(power * -1)

#class for controlling a motor
class Motor:
    def __init__(self, forwardPin, backwardPin, encoderPin):
        #persist values
        self.forwardPin = forwardPin
        self.backwardPin = backwardPin
        self.encoderPin = encoderPin

        #setup GPIO pins
        GPIO.setup(forwardPin, GPIO.OUT)
        GPIO.setup(backwardPin, GPIO.OUT)
        GPIO.setup(encoderPin,GPIO.IN,pull_up_down=GPIO.PUD_DOWN)
        #add encoder pin event
        GPIO.add_event_detect(encoderPin, GPIO.RISING, callback=self._encoderCallback,bouncetime=2)

        #setup pwm
        self.forwardPWM = GPIO.PWM(forwardPin,50)
        self.backwardPWM = GPIO.PWM(backwardPin,50)

        #setup encoder/speed ticks
        self.totalTicks = 0
        self.currentTicks = 0

        self.state = STATESTOPPED

    #motor state property
    @property
    def state(self):
        return self.state

    #start
    def start(self, power):        
        #forward or backward?
        # backward
        if power < 0:
            if self.state != STATEBACKWARD: self.stop()
            self._backward(power)
            self.state = STATEBACKWARD
            
        # forward  
        if power > 0:
            if self.state != STATEFORWARD: self.stop()
            self._forward(power)
            self.state = STATEFORWARD
            
        # stop
        if power == 0:
            self.stop()

    #stop
    def stop(self):
        #stop motor
        self.forwardPWM.stop()
        self.backwardPWM.stop()
        self.state = STATESTOPPED
        #reset ticks
        self.currentTicks = 0

    #private function to calculate the freq for the PWM
    def _calcPowerAndFreq(self, power):
        # make power between 0 and 100
        power = max(0,min(100,abs(power)))
        # make half of freq, minimum of 11
        freq = max(11,abs(power/2))
        return power, freq
    
    #forward
    def _forward(self, power):
        #start forward motor
        power, freq = self._calcPowerAndFreq(power)
        self.forwardPWM.ChangeFrequency(freq)
        self.forwardPWM.start(power)
        
    #backward
    def _backward(self, power):
        #start backward motor
        power, freq = self._calcPowerAndFreq(power)
        self.backwardPWM.ChangeFrequency(freq)
        self.backwardPWM.start(power)

    #encoder callback
    def _encoderCallback(self, pin):
        self.totalTicks += 1
        self.currentTicks += 1

#tests
if __name__ == '__main__':

    try:
        #setup gpio
        GPIO.setmode(GPIO.BOARD)
        
        #create motor control
        motors = MotorController()

        #forward
        print("forward")
        motors.start(100)
        time.sleep(2)
        
        print("encoder ticks")
        print(motors.motorA.totalTicks)
        print(motors.motorB.totalTicks)
        
        #backward 
        print("backward")
        motors.start(-50)
        time.sleep(2)

        #forward curve
        print("forward curve")
        motors.start(100,50)
        time.sleep(2)

        #rotate left
        print("rotate left")
        motors.rotateLeft(50)
        time.sleep(2)

        #rotate right
        print("rotate right")
        motors.rotateRight(50)
        time.sleep(2)
        
        #stop
        print("stop")
        motors.stop()

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
        
