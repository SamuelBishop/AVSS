import logging
import threading
import time
import queue


exitFlag = 0




# Generic thread class
class myThread(threading.Thread):
    def __init__(self, threadID, name, q):
        threading.Thread.__init__(self)
        self.threadID = threadID
        self.name = name
        self.q = q
    # calls process_data
    def run(self):
        print("Starting " + self.name)
        process_data(self.name, self.q)
        print("Exiting " + self.name)


def sound_alarm():
    print("Sound Alarm!")
    pass
def turn_on_led():
    print("LED Turned on")
    pass

def activate_disenfectant():
    print("Activating Disenfectant")
    pass


def process_data(threadName, q):
   while not exitFlag:
     queueLock.acquire()
     if not workQueue.empty():
        data = q.get()
        queueLock.release()
        print("%s processing %s" % (threadName, data))
        if data == "Sanitation":
            activate_disenfectant()
        elif data == "Led":
            turn_on_led()
        elif data == "Alarm":
            sound_alarm()


     else:
        queueLock.release()
     time.sleep(1)


threadList = ["Thread-1", "Thread-2", "Thread-3", "Thread-4"]
taskList = ["Sanitation", "Led", "Alarm"]
queueLock = threading.Lock()
workQueue = queue.Queue(3)
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
for task in taskList:
   workQueue.put(task)
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




# def dispatch_thread():
#
#     print("Dispatch Thread")
#
# # asynchronous task with no deadline or return

#
# """
#
# make a daemon thread that will die when the program dies
# daemon thread: thread that runs in the background,
# is killed if the main program dies,
# normal thread will cause main program to wait until complete
# """
# # asynchronous task with no deadline or return

#
# if __name__ == "__main__":
#     # create the dispatcher thread
#     dispatcher = threading.Thread(target=dispatch_thread)
#     # start the thread
#     dispatcher.start()

