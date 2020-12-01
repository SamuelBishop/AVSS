"""
This project is going to be awesome. Need to research I/O in python for the jetson nano,
also need to research tensorflow lite.

So far all this does is toggle on and off an LED on pin 12
"""

import Jetson.GPIO as GPIO
import time

output_pin = 18  # BOARD pin 12, BCM pin 18

# The function that does all of the image processing
def magic(img):
    finished = 1
    # Process this image, set the relevant pin outputs, return finished
    return finished

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

    # imgFromVideoFeed = "I don't know at all what im doing"
    # magic(imgFromVideoFeed)