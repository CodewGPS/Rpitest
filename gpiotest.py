import RPi.GPIO as GPIO
import time

# Test individual components
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

# Test buzzer
BUZ_PIN = 4
GPIO.setup(BUZ_PIN, GPIO.OUT)
print("Testing buzzer...")
GPIO.output(BUZ_PIN, GPIO.LOW)  # Turn on
time.sleep(1)
GPIO.output(BUZ_PIN, GPIO.HIGH)  # Turn off

# Test pump 1
PMP1_PIN = 2
GPIO.setup(PMP1_PIN, GPIO.OUT)
print("Testing pump 1...")
GPIO.output(PMP1_PIN, GPIO.HIGH)
time.sleep(2)
GPIO.output(PMP1_PIN, GPIO.LOW)

GPIO.cleanup()
