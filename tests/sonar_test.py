import RPi.GPIO as GPIO
import time

# Определение пинов
GPIO.setmode(GPIO.BCM)
TRIG = 10
ECHO = 9

# Настройка пинов
GPIO.setup(TRIG, GPIO.OUT)
GPIO.setup(ECHO, GPIO.IN)

SLEEP_DURATION = 1 / 10
MAX_DURATION = 0.015


def measure_distance():
    time.sleep(SLEEP_DURATION)
    GPIO.output(TRIG, True)
    time.sleep(0.00001)
    GPIO.output(TRIG, False)
    absoluteStart = time.time()

    # Ожидание возврата импульса
    start_time = absoluteStart
    while GPIO.input(ECHO) == 0:
        start_time = time.time()
        if absoluteStart - start_time > MAX_DURATION:
            return None

    end_time = start_time
    while GPIO.input(ECHO) == 1:
        end_time = time.time()
        if end_time - start_time > MAX_DURATION:
            return None

    duration = end_time - start_time

    distance = (duration * 34300) / 2
    # print(f"{end_time - absoluteStart}, {duration}")

    return distance


try:
    while True:
        dist = measure_distance()
        print(dist)
except KeyboardInterrupt:
    print("Измерение остановлено пользователем")
    GPIO.cleanup()
