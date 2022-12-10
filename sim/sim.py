#!/usr/bin/python3

import RPi.GPIO as GPIO
import time

# led = 18
led = 12

GPIO.setmode(GPIO.BOARD)
GPIO.setup(led, GPIO.OUT)

sleep_time = 0.0004
# n = int(0xB9B9B9B9) 
n = int(0x00010000)

while True:
  GPIO.output(led, GPIO.HIGH)
  time.sleep(sleep_time)
  GPIO.output(led, GPIO.LOW)
  time.sleep(sleep_time)

  buf = (0x40 << 24) | n
  for i in range(32): 
    next = (buf & (1 << 31)) != 0
    buf = buf << 1
    #print(buf)
    if next == 0:
      GPIO.output(led, GPIO.LOW)
      time.sleep(sleep_time)
      GPIO.output(led, GPIO.HIGH)
      time.sleep(sleep_time)
    elif next == 1:
      GPIO.output(led, GPIO.HIGH)
      time.sleep(sleep_time)
      GPIO.output(led, GPIO.LOW)
      time.sleep(sleep_time)
    else:
      print("invalid next bit")


  GPIO.output(led, GPIO.HIGH)
  time.sleep(sleep_time)
  GPIO.output(led, GPIO.LOW)
  time.sleep(sleep_time)

  # time.sleep(1.0)
  time.sleep(0.5)
  n = n + 1

GPIO.cleanup()
