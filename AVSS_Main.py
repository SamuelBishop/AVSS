"""
    Project: AVSS (Airborne Virus Sanitation System)
    Team: Jacob Alongi | Samuel Bishop | Michael Boeding
"""

import Jetson.GPIO as GPIO
import time
import nanocamera as nano
import cv2
import numpy as np
import queue
import threading

# kill flag for threads
exitFlag = 0
# deadline for threads in seconds
deadline = 5
# time stamp for last person detected
last_detected = 0
# length of time to ignore detecting new people
lpd_deadline = 5

alarmGoing = False
# initial thread ID for threads
threadID = 1
# this is used to trigger threads, ignores detection by lpd_deadline time
isPersonDetected = False
# way to get actual real time detection of person
real_time_detect_person = False

redPin = 11  # pin 23
greenPin = 5  # pin 29
bluePin = 10  # pin 19

"""
    process_data is called by thread object, run method is overrident to call this method
    - when called checks if the kill thread is set to 1
    - grabs task from the workQueue by name, and runs the coressponding method
        Sanitation:
        Led:
        Alarm:
"""
def process_data(threadName, q):
    success = False
    while not exitFlag:
        queueLock.acquire()
        if not workQueue.empty():
            data = q.get()
            queueLock.release()
            print("%s processing %s" % (threadName, data))
            # print(data[1])
            if data[1] == "Sanitation":
                fanFunction()
                q.task_done()
            elif data[1] == "Led":
                print("LED STATEMENT")
                if(real_time_detect_person):
                    print("GREEN STATEMENT")
                    ledFunction(0, 1, 0)
                else:
                    print("RED STATEMENT")
                    ledFunction(1, 0, 0)
                q.task_done()
            elif data[1] == "Alarm":
                alarmFunction()
                q.task_done()
        else:
            queueLock.release()
        time.sleep(1)


# Implementation of thread object
# needed to change the run function which is overridden
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
    global isPersonDetected
    global lpd_deadline
    global last_detected
    global real_time_detect_person

    # Create the Camera instance
    camera = nano.Camera(camera_type=1, device_id=0, width=640, height=480, fps=30)
    print('USB Camera ready? - ', camera.isReady())
    while camera.isReady():
        try:
            # read the camera image
            frame = camera.read()

            # detect the person now
            if( detectPerson(frame) ):
                time_now = time.time()
                real_time_detect_person = True
                if( time_now - last_detected > lpd_deadline):
                    last_detected = time_now
                    isPersonDetected = True
                else:
                    isPersonDetected = False
            else:
                real_time_detect_person = False
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


# Handles the LED lights
def ledFunction(red=0, green=0, blue=0):

    GPIO.output(redPin, GPIO.HIGH) if red else GPIO.output(redPin, GPIO.LOW)

    GPIO.output(greenPin, GPIO.HIGH) if green else GPIO.output(greenPin, GPIO.LOW)

    GPIO.output(bluePin, GPIO.HIGH) if blue else GPIO.output(bluePin, GPIO.LOW)
    return 1



def fanFunction():
    # From left to right the inputs of RGB LED are R, Ground, G, B
    fanPin = 19 # pin 35
    GPIO.setup(fanPin, GPIO.OUT, initial=GPIO.LOW)
    GPIO.output(fanPin, GPIO.HIGH)
    #deadline = 200 # in miliseconds
    # ENSURE DEADLINE MET, IDK HOW TO DO THIS IN PYTHON
    return 1




print("Starting demo now! Press CTRL+C to exit")
GPIO.setmode(GPIO.BCM)

# Create camera thread to run by itself
cam_task = threading.Thread(target=run_camera, daemon=False)
cam_task.start()

# initialize the lights to start with red on
ledFunction(1, 0, 0)

"""
    Dispatcher process
    if person is detected: 
        - create 3 threads, main process takes 1 thread of 4 in Jetson Nano
            when threads created, start_time is set, used for deadlines
        - create a que and load with the task to be handled when a person is detected
            using priority que to give priority to lower assigned value
            locks que while accessing to prevent race conditions
        - wait until the queue is empty
        - wait until the deadline is reached and send signal to kill threads
        - join all threads so they can exit, reset kill signal to 0
    
"""
while(1):
    if (isPersonDetected):
        # create list of threads
        threadList = ["Thread-1", "Thread-2", "Thread-3"]
        # create list of tasks
        taskList = ["Sanitation", "Led", "Alarm"]
        queueLock = threading.Lock()
        workQueue = queue.PriorityQueue(10)
        threads = []

        # Create new threads
        # print(threadList)
        for tName in threadList:
            thread = myThread(threadID, tName, workQueue)
            thread.start()
            starttime = time.time()
            threads.append(thread)
            threadID += 1
            # print(threadID)

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

        # Wait for queue to empty
        # print(workQueue.queue)
        while not workQueue.empty():
            pass


        # Notify threads it's time to exit if deadline passed
        while(1):
            if time.time() - starttime > deadline:
                print("Deadline reached")
                exitFlag = 1
                alarmGoing = False
                break

        # Wait for all threads to complete
        for t in threads:
            t.join()

        print("All Task ended")
        # print("Queue Size: ", workQueue.qsize())
        # print(threading.active_count())
        exitFlag = 0
    # person not detected, set light to red
    else:
        ledFunction(1, 0, 0)

