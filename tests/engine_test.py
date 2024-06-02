from src.execution_scripts.executor.executor import HardwareExecutor

from time import sleep

executor = HardwareExecutor()

try:
    print('Forward with speed: 100')
    executor.setSpeed(50)
    sleep(5)
    print('Back with speed: 100')
    executor.setSpeed(-50)
    sleep(5)
    print('0%')
    executor.setSpeed(0)
    sleep(5)
    print('left')
    executor.setRightSpeed(50)
    executor.setLeftSpeed(-50)
    sleep(5)
    print('right')
    executor.setRightSpeed(-50)
    executor.setLeftSpeed(50)
    sleep(5)

except KeyboardInterrupt:
    print('Keyboard Interrupt')
    executor.onDestroy()

executor.onDestroy()
