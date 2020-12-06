import logging
import threading
import time

def dispatch_thread():
    while(1):
        print("Dispatch Thread")

def sound_alarm():
    pass

def turn_on_led():
    pass

def activate_disenfectant():
    pass

if __name__ == "__main__":
    dispather = threading.Thread()


