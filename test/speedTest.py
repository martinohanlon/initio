import motorControl
import time
import RPi.GPIO as GPIO
import sys

leftCount = 0
rightCount = 0
leftPin = 7
rightPin = 11

def my_callback(pin):
    global leftCount
    global rightCount

    if pin == leftPin: leftCount = leftCount + 1
    if pin == rightPin: rightCount = rightCount + 1

if __name__ == '__main__':

    try:
        #setup gpio
        GPIO.setmode(GPIO.BOARD)

        #setup pins
        GPIO.setup(leftPin,GPIO.IN,pull_up_down=GPIO.PUD_DOWN)
        GPIO.add_event_detect(leftPin, GPIO.RISING, callback=my_callback,bouncetime=2)
        GPIO.setup(rightPin,GPIO.IN,pull_up_down=GPIO.PUD_DOWN)
        GPIO.add_event_detect(rightPin, GPIO.RISING, callback=my_callback,bouncetime=2)

        #create motor control
        motors = motorControl.MotorController()

        #move motor
        print("backward")
        motors.backward(25)

        time.sleep(3)

        print("stopped")
        motors.stop()

        print leftCount
        print rightCount
        
    #Ctrl C
    except KeyboardInterrupt:
        print("User cancelled")

    #Error
    except:
        print("Unexpected error:", sys.exc_info()[0])
        raise

    finally:
        print("cleanup")
        #cleanup gpio
        GPIO.cleanup()
        
