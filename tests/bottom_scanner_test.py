from src.execution_scripts.executor.executor import HardwareExecutor

executor = HardwareExecutor()


try:

    while True:
        print(f'data = {executor.readInfraScannerData()}')

except KeyboardInterrupt:
    executor.onDestroy()
