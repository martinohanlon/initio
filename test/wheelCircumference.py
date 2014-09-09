import sys
import motorControl
import time
import RPi.GPIO as GPIO

#tests
if __name__ == '__main__':

    try:
        #setup gpio
        GPIO.setmode(GPIO.BOARD)

        #create motor control
        motors = motorControl.MotorController()

        motors.start(100)
        while motors.motorA.currentTicks < 36:
            time.sleep(0.0001)
        motors.stop()
        print motors.motorA.totalTicks
        
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
