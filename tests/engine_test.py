from src.execution_scripts.executor.executor import DCMotor, HardwareExecutor

from time import sleep

executor = HardwareExecutor()


try:
    print('Forward with speed: 50%')
    executor.setSpeed(100)
    sleep(5)
    executor.setSpeed(-100)
    sleep(5)
    print('Backwards with speed: 100%')
    executor.setSpeed(0)
    sleep(5)
    print('Forward with speed: 5%')
    executor.setRightSpeed(50)
    executor.setLeftSpeed(-50)
    sleep(5)
    executor.setRightSpeed(-50)
    executor.setLeftSpeed(50)
    sleep(5)

except KeyboardInterrupt:
    print('Keyboard Interrupt')
    executor.onDestroy()