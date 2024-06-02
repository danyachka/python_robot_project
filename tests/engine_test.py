import RPi.GPIO as GPIO
import time

from src.execution_scripts.executor.executor import setWheelSpeed

# Настройка GPIO
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

# Пины для IN1, IN2 и ENA
IN1 = 17
IN2 = 27
ENA = 22

# Установка пинов как выходы
GPIO.setup(IN1, GPIO.OUT)
GPIO.setup(IN2, GPIO.OUT)
GPIO.setup(ENA, GPIO.OUT)

# Настройка ШИМ на пине ENA
pwm = GPIO.PWM(ENA, 100)  # Частота 100 Гц
pwm.start(0)  # Начальная скорость 0%



try:
    while True:
        # Пример: двигать мотор вперед на 50% скорости
        setWheelSpeed(50, IN1, IN2, ENA)
        time.sleep(5)

        # Пример: остановить мотор
        setWheelSpeed(0, IN1, IN2, ENA)
        time.sleep(2)

        # Пример: двигать мотор назад на 75% скорости
        setWheelSpeed(-75, IN1, IN2, ENA)
        time.sleep(5)

except KeyboardInterrupt:
    pass

# Остановка ШИМ и очистка GPIO
pwm.stop()
GPIO.cleanup()
