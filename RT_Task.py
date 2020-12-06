#!/usr/bin/python

import queue
import threading
import time

exitFlag = 0

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


def sound_alarm():
    try:
        print("Sound Alarm!")
    except Exception as e:
        print(e)
        time_now = time.time()
        # logging.error(time_now, e)
        return False
    return True


def turn_on_led():
    try:
        print("LED Turned on")
    except Exception as e:
        print(e)
        time_now = time.time()
        # logging.error(time_now, e)
        return False
    return True


def activate_disenfectant():
    try:
        print("Activating Disenfectant")
    except Exception as e:
        print(e)
        timeNow = time.time()
        # logging.error(timeNow, e)
        return False
    return True


def process_data(threadName, q):
    success = False
    while not exitFlag:
        queueLock.acquire()
        if not workQueue.empty():
            data = q.get()
            queueLock.release()
            print("%s processing %s" % (threadName, data))
            if data == "Sanitation":
                success = activate_disenfectant()
                q.task_done()

            elif data == "Led":
                success = turn_on_led()
                q.task_done()
            elif data == "Alarm":
                success = sound_alarm()
                q.task_done()
        else:
            queueLock.release()
        time.sleep(1)

object_detected = True
if(object_detected):
    threadList = ["Thread-1", "Thread-2", "Thread-3"]
    taskList = ["Sanitation", "Led", "Alarm"]
    queueLock = threading.Lock()
    workQueue = queue.Queue(10)
    threads = []
    threadID = 1

    # Create new threads
    for tName in threadList:
       thread = myThread(threadID, tName, workQueue)
       thread.start()
       threads.append(thread)
       threadID += 1

    # Fill the queue
    queueLock.acquire()
    for word in taskList:
       workQueue.put(word)
    queueLock.release()

    # Wait for queue to empty
    while not workQueue.empty():
       pass

    # Notify threads it's time to exit
    exitFlag = 1

    # Wait for all threads to complete
    for t in threads:
       t.join()
    print("Exiting Main Thread")