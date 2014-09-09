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
        time.sleep(3)
        motors.stop()
        motors.start(-50,50)
        time.sleep(2)
        motors.stop()
        motors.start(50,50)
        time.sleep(1)
        motors.stop()
        motors.start(50,-50)
        time.sleep(2)
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
