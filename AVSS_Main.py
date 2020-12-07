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

# For RT_Task
import queue
import threading
import trace

exitFlag = 0
deadline = 5 # 5 seconds
last_detected = 0
lpd_deadline = 5
isPersonDetected = False
alarmGoing = False
threadID = 1

# From RT_Task.py
def process_data(threadName, q):
    success = False
    while not exitFlag:
        queueLock.acquire()
        if not workQueue.empty():
            data = q.get()
            queueLock.release()
            print("%s processing %s" % (threadName, data))
            if data == "Sanitation":
                fanFunction()
                q.task_done()
            elif data == "Led":
                ledFunction(isPersonDetected)
                q.task_done()
            elif data == "Alarm":
                alarmFunction()
                print("This is where shit will go after testing of alarm function")
                q.task_done()
        else:
            queueLock.release()
        time.sleep(1)



class myThread (threading.Thread):
   def __init__(self, threadID, name, q):
      threading.Thread.__init__(self)
      self.threadID = threadID
      self.name = name
      self.q = q
   def run(self):
      print("Starting " + self.name)
      process_data(self.name, self.q)
      print("Exiting " + self.name)


def run_camera():
# Create the Camera instance
    camera = nano.Camera(camera_type=1, device_id=0, width=640, height=480, fps=30)
    print('USB Camera ready? - ', camera.isReady())
    while camera.isReady():
        try:
            # read the camera image
            frame = camera.read()
            #detect the person now
            global isPersonDetected
            global lpd_deadline
            global last_detected

            if( detectPerson(frame) ):
                time_now = time.time()
                if( time_now - last_detected > lpd_deadline):
                    last_detected = time_now
                    isPersonDetected = True
                else:
                    isPersonDetected = False

            # display the frame
            cv2.imshow("Video Frame", frame)
            if cv2.waitKey(25) & 0xFF == ord('q'):
                break

            # Definetly want to eventually change this after testing
            # Should be running on its own thread
            # if isPersonDetected:
            #     break
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
    # print("Weights:", weights)
    #check to see if we detected a person
    if len(weights) > 0:
        print("There is a person")
        boxes = np.array([[x, y, x + w, y + h] for (x, y, w, h) in boxes])
        #create the boxes
        for (xA, yA, xB, yB) in boxes:
            # display the detected boxes in the colour picture
            cv2.rectangle(frame, (xA, yA), (xB, yB),
                          (0, 255, 0), 2)
        # Display the resulting frame
        #cv2.imshow('frame', frame)
        return True
    else:
        # print("No Person")
        return False

def alarmFunction():
    outputPin = 6 # pin 31
    GPIO.setup(outputPin, GPIO.OUT, initial=GPIO.HIGH)
    global alarmGoing
    alarmGoing = True
    # GPIO.setup(outputPin, GPIO.OUT, initial=GPIO.HIGH)
    # deadline = 100 # in miliseconds
    # ENSURE DEADLINE MET, IDK HOW TO DO THIS IN PYTHON

    # Eventually this will be run with its own thread but for testing set it on a timer
    duration = 0
    while( alarmGoing ):
        GPIO.output(outputPin, GPIO.HIGH)
        time.sleep(.003)  # change as necessary
        GPIO.output(outputPin, GPIO.LOW)
        time.sleep(.003)  # change as necessary

    return 1

# Handles the LED
def ledFunction(isPerson):
    # From left to right the inputs of RGB LED are R, Ground, G, B
    redPin = 11 # pin 23
    GPIO.setup(redPin, GPIO.OUT, initial=GPIO.LOW)
    GreenPin = 5 # pin 29
    GPIO.setup(GreenPin, GPIO.OUT, initial=GPIO.LOW)
    bluePin = 10 # pin 19
    GPIO.setup(bluePin, GPIO.OUT, initial=GPIO.LOW)
    if( isPerson ):
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

def fanFunction():
    # From left to right the inputs of RGB LED are R, Ground, G, B
    fanPin = 19 # pin 35
    GPIO.setup(fanPin, GPIO.OUT, initial=GPIO.LOW)
    print("Entering Fan")
    GPIO.output(fanPin, GPIO.HIGH)
    #deadline = 200 # in miliseconds
    # ENSURE DEADLINE MET, IDK HOW TO DO THIS IN PYTHON
    return 1



# Main for all intents and purposes
print("Starting demo now! Press CTRL+C to exit")
GPIO.setmode(GPIO.BCM)

print("Before")

# Create the never ending daemon thread
cam_task = threading.Thread(target=run_camera, daemon=False)
cam_task.start()

print("After")
# run_camera()

# This is where the bulk of the code will go
while(1):
    if (isPersonDetected):
        print("Queueing")
        threadList = ["Thread-1", "Thread-2", "Thread-3"]
        taskList = ["Sanitation", "Led", "Alarm"]
        queueLock = threading.Lock()
        workQueue = queue.PriorityQueue(10)
        threads = []

        # Create new threads
        print(threadList)
        for tName in threadList:
            thread = myThread(threadID, tName, workQueue)
            thread.start()
            starttime = time.time()
            threads.append(thread)
            threadID += 1
            print(threadID)

        # Fill the queue
        queueLock.acquire()
        for task in taskList:

            if( task == "Sanitation" ):
                workQueue.put((3, task))
            if (task == "Led" ):
                workQueue.put((2, task))
            if (task == "Alarm" ):
                workQueue.put((1, task))


        # print("Queue Size: ", workQueue.qsize())
        queueLock.release()

        print("Test4")

        # Wait for queue to empty
        print(workQueue.queue)
        while not workQueue.empty():
            pass

        workQueue.queue.clear()

        print("Test5")

        # Notify threads it's time to exit
        print("TEST")
        print(starttime)
        print("Measured Time", time.time())
        print("Measured Elapsed Time", time.time() - starttime)
        print("Deadline: ", deadline)
        while(1):
            if time.time() - starttime > deadline:
                print("Deadline reached")
                exitFlag = 1
                alarmGoing = False
                break

        # Wait for all threads to complete
        for t in threads:
            t.join()
        # for q in workQueue:
        #     q.join()

        print("All Task ended")
        print("Queue Size: ", workQueue.qsize())
        print(threading.active_count())
        exitFlag = 0

# if __name__ == '__main__':
#     #map to the bcm
#     GPIO.setmode(GPIO.BCM)
#     #run the camera
#     run_camera()
#     print("Starting demo now! Press CTRL+C to exit")
#     ledFunction()


    # alarmFunction()
    # fanFunction()

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
