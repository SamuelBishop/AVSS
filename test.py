import threading, queue, time, logging, random
person_detected = False
dispatch = None
q = queue.PriorityQueue()
time_now = 0
last_detected = 0
deadline = 30

def dispatcher():
    threadList = ["Thread-1", "Thread-2", "Thread-3"]

    pass

def main():
    dispatch = threading.Thread(target=dispatcher, daemon=True).start()
    while(1):
        rand_num = random.randint(1, 100)
        if rand_num > 60:
            person_detected = True
        else:
            person_detected = False

        if person_detected:
            time_now = time.time()
            if time_now - last_detected > deadline:
                last_detected = time_now
                q.put("Sanitation")

            q.put("LED_ON")
            q.put("Alarm_ON")


        else:
            q.put("LED_OFF")
            q.put("Alarm_OFF")


if __name__ == '__main__':
    main()