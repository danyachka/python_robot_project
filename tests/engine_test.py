from src.execution_scripts.executor.executor import HardwareExecutor

from time import sleep

executor = HardwareExecutor()

try:
    speed = 25
    print('Forward with speed: 100')
    executor.setSpeed(speed)
    sleep(5)
    print('Back with speed: 100')
    executor.setSpeed(-speed)
    sleep(5)
    print('0%')
    executor.setSpeed(0)
    sleep(5)
    print('left')
    executor.setRightSpeed(speed)
    executor.setLeftSpeed(-speed)
    sleep(5)
    print('right')
    executor.setRightSpeed(-speed)
    executor.setLeftSpeed(speed)
    sleep(5)

except KeyboardInterrupt:
    print('Keyboard Interrupt')
    executor.onDestroy()

executor.onDestroy()
