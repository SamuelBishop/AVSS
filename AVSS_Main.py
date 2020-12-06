"""
This project is going to be awesome. Need to research I/O in python for the jetson nano,
also need to research tensorflow lite.

So far all this does is toggle on and off an LED on pin 12
"""

import Jetson.GPIO as GPIO
import time

output_pin = 18  # BOARD pin 12, BCM pin 18

if __name__ == '__main__':
    # Pin Setup:
    # Board pin-numbering scheme
    GPIO.setmode(GPIO.BCM)
    # set pin as an output pin with optional initial state of HIGH
    GPIO.setup(output_pin, GPIO.OUT, initial=GPIO.HIGH)

    print("Starting demo now! Press CTRL+C to exit")
    curr_value = GPIO.HIGH
    try:
        while True:
            time.sleep(1)
            # Toggle the output every second
            print("Outputting {} to pin {}".format(curr_value, output_pin))
            GPIO.output(output_pin, curr_value)
            curr_value ^= GPIO.HIGH
    finally:
        GPIO.cleanup()

"""
PSUEDOCODE:

def main:
    humanDetected = false
    feed = DeepSteam(webcamInput) # Get michael to do this part and get the feed as a variable
    humanDetected = tensorflow.process(feed) # Will return true if human in frame (hopefully tensorflow has built in functions)
    
    def rtPeriodicTask(humanDetected):
        if humanDetected:
            pthread taskThread
            pthread_create taskThread
            pthread_join(taskThread, deadLineTask)
        schedule(500) # Repeat task every so often as needed
        
    # Creates threads for all jobs, joins threads for all jobs
    def deadLineTask():
        jobThread1 = dispatchChildThread()
        jobThread2 = dispatchChildThread()
        jobThread3 = dispatchChildThread()
        pthread_join(jobThread1, alarmFunction)
        pthread_join(jobThread2, ledFunction)
        pthread_join(jobThread3, fanFunction)
    
    # Handles the alarm
    def alarmFunction:
        output_pin = 5 # change as necessary
        deadline = 100 # in miliseconds
        # ENSURE DEADLINE MET, IDK HOW TO DO THIS IN PYTHON
        while(1):
            if(shouldExit): # figure this out for python
                exit();
            GPIO.output(output_pin, GPIO.HIGH)
            time.sleep(.1) # change as necessary
            GPIO.output(output_pin, GPIO.LOW)
            time.sleep(.1) # change as necessary
        return 1
        
    # Handles the LED
    def ledFunction:
        output_pin = 6 # change as necessary
        deadline = 200 # in miliseconds
        # ENSURE DEADLINE MET, IDK HOW TO DO THIS IN PYTHON
        GPIO.output(output_pin, GPIO.HIGH)
        return 1
        
    # Handles the fan
    def fanFunction:
        output_pin = 7 # change as necessary
        deadline = 200 # in miliseconds
        # ENSURE DEADLINE MET, IDK HOW TO DO THIS IN PYTHON
        GPIO.output(output_pin, GPIO.HIGH)
        return 1
"""