import RPi.GPIO as GPIO
GPIO.setmode(GPIO.BCM)

GPIO.setup(17,GPIO.OUT,initial=0)
GPIO.setup(27,GPIO.OUT,initial=0)

GPIO.output(17, 1)
GPIO.output(27, 1)