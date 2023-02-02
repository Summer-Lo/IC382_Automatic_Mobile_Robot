# Raspberry Pi Python program for 202223 IC382
# Top Module (Robot Arm/Fork) Control written by Summer Lo on 2023-02-02

import RPi.GPIO as GPIO
import time

pickup_pin = 6
ack_pickup_pin = 7
place_pin = 5
ack_place_pin = 8

GPIO.setmode(GPIO.BCM)
GPIO.setup(pickup_pin,GPIO.OUT)
GPIO.setup(place_pin,GPIO.OUT)
GPIO.setup(ack_pickup_pin,GPIO.IN)
GPIO.setup(ack_place_pin,GPIO.IN)
GPIO.output(pickup_pin,0)
GPIO.output(place_pin,0)


def pick_up():
    print("[INFO] ----------Starting Pick Up Process----------")
    GPIO.output(pickup_pin,1)
    print(GPIO.input(ack_pickup_pin))
    while(GPIO.input(ack_pickup_pin) == 0):
        print("[GPIO] ACK Pick Up Pin LOW now!")
        time.sleep(0.5)
        pass
    print("[GPIO] *** ACK PICK UP ", GPIO.input(ack_pickup_pin), "  ***")
    GPIO.output(pickup_pin,0)
    print("[INFO] ----------End for Pick Up Process----------")

def place():
    print("[INFO] ----------Starting Place Process----------")
    GPIO.output(place_pin,1)
    print(GPIO.input(ack_place_pin))
    while(GPIO.input(ack_place_pin) == 0):
        print("[GPIO] ACK Place Pin LOW now!")
        time.sleep(0.5)
        pass
    print("[GPIO] *** ACK PLACE ", GPIO.input(ack_place_pin), " ***")
    GPIO.output(place_pin,0)
    print("[INFO] ----------End for Place Process----------")


# Testing part

if __name__ == '__main__':
    try:
        time.sleep(5)
        pick_up()
        time.sleep(2)
        place()
    except KeyboardInterrupt:
        print("[System] Reset All GPIO Pin and EXIT now!")
        GPIO.output(pickup_pin,0)
        GPIO.output(place_pin,0)
