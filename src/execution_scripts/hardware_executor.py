import numpy as np
import sim
import cv2 as cv
from src.execution_scripts.emulation_tools import GyroscopeEmulator


# How to create interfaces (abstract classes)?
class HardwareExecutorModel:

    def __init__(self):
        pass

    def readImage(self) -> np.ndarray:
        pass

    def readGyro(self) -> [float]:
        pass

    def readSonarData(self):
        pass

    def setSpeed(self, speed) -> None:
        pass

    def setRightSpeed(self, speed) -> None:
        pass

    def setLeftSpeed(self, speed) -> None:
        pass

    def rotate(self, angle) -> None:
        pass


# Should execute commands in simulation
class HardwareExecutorEmulator(HardwareExecutorModel):
    clientId = 0

    frontLeftWheel = None
    frontRightWheel = None
    backLeftWheel = None
    backRightWheel = None

    camera_handle = None

    robot_handle = None
    gyroscopeEmulator: GyroscopeEmulator

    def __init__(self, clientId):
        super().__init__()
        self.clientId = clientId

        res, self.frontLeftWheel = sim.simxGetObjectHandle(
            clientId, './front_left_wheel', sim.simx_opmode_oneshot_wait)
        print(f'frontLeftWheel handel - {res is sim.simx_return_ok}')

        res, self.frontRightWheel = sim.simxGetObjectHandle(
            clientId, './front_right_wheel', sim.simx_opmode_oneshot_wait)
        print(f'frontRightWheel handel - {res is sim.simx_return_ok}')

        res, self.backLeftWheel = sim.simxGetObjectHandle(
            clientId, './back_right_wheel', sim.simx_opmode_oneshot_wait)
        print(f'backLeftWheel handel - {res is sim.simx_return_ok}')

        res, self.backRightWheel = sim.simxGetObjectHandle(
            clientId, './back_left_wheel', sim.simx_opmode_oneshot_wait)
        print(f'backRightWheel handel - {res is sim.simx_return_ok}')

        res, self.camera_handle = sim.simxGetObjectHandle(clientId, './camera', sim.simx_opmode_oneshot_wait)
        print(f'Camera handel - {res is sim.simx_return_ok}')

        res, self.robot_handle = sim.simxGetObjectHandle(clientId, './robot', sim.simx_opmode_blocking)
        print(f'Robot handel - {res is sim.simx_return_ok}')

        self.gyroscopeEmulator = GyroscopeEmulator()

    def readImage(self) -> np.ndarray:
        res, resolution, image = sim.simxGetVisionSensorImage(
            self.clientId, self.camera_handle, 0, sim.simx_opmode_blocking)

        if res == sim.simx_return_ok:
            # Convert the image to a format usable by OpenCV
            image = np.array(image, dtype=np.uint8)
            image = image.reshape([resolution[1], resolution[0], 3])
            image = np.flip(image, 0)
            image = cv.cvtColor(image, cv.COLOR_RGB2BGR)
            return image
        else:
            print("Failed to capture image.")
            return None

    def readGyro(self) -> [float]:
        res, euler_angles = (
            sim.simxGetObjectOrientation(self.clientId, self.robot_handle, -1, sim.simx_opmode_blocking))

        return self.gyroscopeEmulator.update(euler_angles)

    def readSonarData(self):
        pass

    def setRightSpeed(self, speed) -> None:
        error = sim.simxSetJointTargetVelocity(
            self.clientId, self.frontRightWheel, -speed, sim.simx_opmode_oneshot_wait)
        print(f'frontRightWheel: {error is sim.simx_return_ok}')

        error = sim.simxSetJointTargetVelocity(
            self.clientId, self.backRightWheel, -speed, sim.simx_opmode_oneshot_wait)
        print(f'backRightWheel: {error is sim.simx_return_ok}')

    def setLeftSpeed(self, speed) -> None:
        error = (sim.simxSetJointTargetVelocity
                 (self.clientId, self.frontLeftWheel, speed, sim.simx_opmode_oneshot_wait))
        print(f'frontLeftWheel: {error is sim.simx_return_ok}')

        error = sim.simxSetJointTargetVelocity(
            self.clientId, self.backLeftWheel, speed, sim.simx_opmode_oneshot_wait)
        print(f'backLeftWheel: {error is sim.simx_return_ok}')

    def setSpeed(self, speed) -> None:
        self.setLeftSpeed(speed)
        self.setRightSpeed(speed)

    def rotate(self, angle):
        pass


# Should execute commands with actual hardware
class HardwareExecutor(HardwareExecutorModel):

    def __init__(self):
        super().__init__()
        pass
