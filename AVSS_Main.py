"""
This project is going to be awesome. Need to research I/O in python for the jetson nano,
also need to research tensorflow lite.
So far all this does is toggle on and off an LED on pin 12
"""

import Jetson.GPIO as GPIO
import time
import nanocamera as nano
import cv2
import numpy as np

def create_camera():
# Create the Camera instance
    camera = nano.Camera(camera_type=1, device_id=0, width=640, height=480, fps=30)
    print('USB Camera ready? - ', camera.isReady())
    while camera.isReady():
        try:
            # read the camera image
            frame = camera.read()
            # display the frame
            cv2.imshow("Video Frame", frame)
            if cv2.waitKey(25) & 0xFF == ord('q'):
                break
        except KeyboardInterrupt:
            break

    # close the camera instance
    camera.release()
    
    
    

#function used to tell if there is a person or nit
def detectPerson(frame):

    # initialize the HOG descriptor/person detector
    hog = cv2.HOGDescriptor()
    hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())

    # detect people in the image
    # returns the bounding boxes for the detected objects
    boxes, weights = hog.detectMultiScale(frame, winStride=(8,8) )

    #if weights are bigger than 2 then we have a person
    print("Weights:", weights)

    if len(weights) > 0:

        print("There is a person")

    else:
        print("No Person")

    boxes = np.array([[x, y, x + w, y + h] for (x, y, w, h) in boxes])

    for (xA, yA, xB, yB) in boxes:
        # display the detected boxes in the colour picture
        cv2.rectangle(frame, (xA, yA), (xB, yB),
                          (0, 255, 0), 2)

    # Display the resulting frame
    cv2.imshow('frame',frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        #program finished
        print("Finished ")

def alarmFunction():
    outputPin = 6 # pin 38
    GPIO.setup(outputPin, GPIO.OUT, initial=GPIO.HIGH)
    # GPIO.setup(outputPin, GPIO.OUT, initial=GPIO.HIGH)
    # deadline = 100 # in miliseconds
    # ENSURE DEADLINE MET, IDK HOW TO DO THIS IN PYTHON

    while(1):
        GPIO.output(outputPin, GPIO.HIGH)
        time.sleep(.0001)  # change as necessary
        GPIO.output(outputPin, GPIO.LOW)
        time.sleep(.0001)  # change as necessary

    # while(1):
    #     # if(shouldExit): # figure this out for python
    #     #     exit();
    #     GPIO.output(outputPin, GPIO.HIGH)
    #     time.sleep(1) # change as necessary
    #     GPIO.output(outputPin, GPIO.LOW)
    #     time.sleep(1) # change as necessary
    return 1

# Handles the LED
def ledFunction():
    # From left to right the inputs of RGB LED are R, Ground, G, B
    redPin = 11 # pin 23
    GPIO.setup(redPin, GPIO.OUT, initial=GPIO.LOW)
    GreenPin = 5 # pin 29
    GPIO.setup(GreenPin, GPIO.OUT, initial=GPIO.LOW)
    bluePin = 10 # pin 19
    GPIO.setup(bluePin, GPIO.OUT, initial=GPIO.LOW)
    if( False ):
        # green condition
        GPIO.output(redPin, GPIO.LOW)
        GPIO.output(GreenPin, GPIO.HIGH)
        GPIO.output(bluePin, GPIO.LOW)
    else:
        # red condtion
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
    # GPIO.setup(output_pin, GPIO.OUT, initial=GPIO.HIGH)

    print("Starting demo now! Press CTRL+C to exit")
    # ledFunction()
    alarmFunction()

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
