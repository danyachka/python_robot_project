import RPi.GPIO as GPIO
import time
import numpy as np
import cv2

# Настройка GPIO
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)
XCLK = 4
PCLK = 18
VSYNC = 24
HREF = 25
DATA_PINS = [17, 27, 22, 23, 5, 6, 13, 19]  # Используемые пины данных (D0-D7)

GPIO.setup(XCLK, GPIO.OUT)
GPIO.setup(PCLK, GPIO.IN)
GPIO.setup(VSYNC, GPIO.IN)
GPIO.setup(HREF, GPIO.IN)
for pin in DATA_PINS:
    GPIO.setup(pin, GPIO.IN)


# Генерация сигнала XCLK (пример с частотой ~8 MHz)
def generate_xclk():
    pwm = GPIO.PWM(XCLK, 8000000)
    pwm.start(50)  # 50% скважность
    return pwm


# Функция для чтения данных с пинов
def read_data_pins():
    value = 0
    for i, pin in enumerate(DATA_PINS):
        if GPIO.input(pin):
            value |= (1 << i)
    return value


# Захват изображения
def capture_image(width, height):
    image = np.zeros((height, width, 3), dtype=np.uint8)

    capturing = False
    row = 0

    while row < height:
        if GPIO.input(VSYNC) == GPIO.HIGH:
            capturing = True
            row = 0
            while GPIO.input(VSYNC) == GPIO.HIGH:
                pass  # Ожидание окончания сигнала VSYNC

        if capturing and GPIO.input(HREF) == GPIO.HIGH:
            col = 0
            while GPIO.input(HREF) == GPIO.HIGH and col < width:
                if GPIO.input(PCLK) == GPIO.HIGH:
                    pixel_value = read_data_pins()
                    red = (pixel_value & 0xF800) >> 8
                    green = (pixel_value & 0x07E0) >> 3
                    blue = (pixel_value & 0x001F) << 3
                    pixel = image[row, col]
                    pixel[0] = blue
                    pixel[1] = red
                    pixel[2] = green
                    col += 1
                    while GPIO.input(PCLK) == GPIO.HIGH:
                        pass  # Ожидание окончания сигнала PCLK
            row += 1
    return image

try:
    pwm = generate_xclk()
    print("XCLK сигнал генерируется...")

    width, height = 640, 480  # Разрешение QVGA
    print("Захват изображения...")
    raw_image = capture_image(width, height)
    print("Изображение захвачено...")

    # Использование OpenCV для отображения изображения
    cv2.imshow('Captured Image', raw_image)
    cv2.waitKey(0)  # Ожидание нажатия любой клавиши для закрытия окна
    print("Изображение отображено...")

    # Сохранение изображения в файл
    cv2.imwrite('captured_image.png', raw_image)
    print("Изображение сохранено как captured_image.png")

except KeyboardInterrupt:
    print("Прерывание программы")
finally:
    pwm.stop()
    GPIO.cleanup()
