"""
This project is going to be awesome. Need to research I/O in python for the jetson nano,
also need to research tensorflow lite.

So far all this does is toggle on and off an LED on pin 12
"""

import Jetson.GPIO as GPIO
import time

output_pin = 18  # BOARD pin 12, BCM pin 18

def alarmFunction():
    buttonPin = 24 # pin 18
    GPIO.setup(buttonPin, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
    outputPin = 23 # pin 16
    # deadline = 100 # in miliseconds
    # ENSURE DEADLINE MET, IDK HOW TO DO THIS IN PYTHON
    while(GPIO.input(buttonPin)):
        # if(shouldExit): # figure this out for python
        #     exit();
        GPIO.output(outputPin, GPIO.HIGH)
        time.sleep(.5) # change as necessary
        GPIO.output(outputPin, GPIO.LOW)
        time.sleep(.5) # change as necessary
    return 1

# Handles the LED
def ledFunction():
    # From left to right the inputs of RGB LED are R, Ground, G, B
    redPin = 11 # pin 23
    GreenPin = 9 # pin 21
    bluePin = 10 # pin 19
    buttonPin = 22  # pin 15
    GPIO.setup(buttonPin, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
    buttonSetting = 0
    if( GPIO.input(buttonPin) ): # If button pressed - green
        # red condition
        GPIO.output(redPin, GPIO.LOW)
        GPIO.output(GreenPin, GPIO.HIGH)
        GPIO.output(bluePin, GPIO.LOW)
    else: # If button released - red
        # green condtion
        GPIO.output(redPin, GPIO.HIGH)
        GPIO.output(GreenPin, GPIO.LOW)
        GPIO.output(bluePin, GPIO.LOW)
    #deadline = 200 # in miliseconds
    # ENSURE DEADLINE MET, IDK HOW TO DO THIS IN PYTHON
    return 1

if __name__ == '__main__':
    # Pin Setup:
    # Board pin-numbering scheme
    GPIO.setmode(GPIO.BCM)
    # set pin as an output pin with optional initial state of HIGH
    GPIO.setup(output_pin, GPIO.OUT, initial=GPIO.HIGH)

    print("Starting demo now! Press CTRL+C to exit")
    try:
        while True:
            time.sleep(.1)
            alarmFunction()
            time.sleep(.1)
            ledFunction()
    finally:
        GPIO.cleanup()


    # curr_value = GPIO.HIGH
    # try:
    #     while True:
    #         time.sleep(1)
    #         # Toggle the output every second
    #         print("Outputting {} to pin {}".format(curr_value, output_pin))
    #         GPIO.output(output_pin, curr_value)
    #         curr_value ^= GPIO.HIGH
    # finally:
    #     GPIO.cleanup()


# PSUEDOCODE:

# def main():

    # humanDetected = false
    # feed = DeepSteam(webcamInput) # Get michael to do this part and get the feed as a variable
    # humanDetected = tensorflow.process(feed) # Will return true if human in frame (hopefully tensorflow has built in functions)

    # def rtPeriodicTask(humanDetected):
    #     if humanDetected:
    #         pthread taskThread
    #         pthread_create taskThread
    #         pthread_join(taskThread, deadLineTask)
    #     schedule(500) # Repeat task every so often as needed
    #
    # # Creates threads for all jobs, joins threads for all jobs
    # def deadLineTask():
    #     jobThread1 = dispatchChildThread()
    #     jobThread2 = dispatchChildThread()
    #     jobThread3 = dispatchChildThread()
    #     pthread_join(jobThread1, alarmFunction)
    #     pthread_join(jobThread2, ledFunction)
    #     pthread_join(jobThread3, fanFunction)

# Handles the alarm



#
# # Handles the fan
# def fanFunction:
#     output_pin = 7 # change as necessary
#     deadline = 200 # in miliseconds
#     # ENSURE DEADLINE MET, IDK HOW TO DO THIS IN PYTHON
#     GPIO.output(output_pin, GPIO.HIGH)
#     return 1