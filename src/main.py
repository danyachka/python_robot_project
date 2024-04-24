import sys
import sim
import execution_scripts.scene_1 as scene_1


sim.simxFinish(-1)  # just in case, close all opened connections

# В главном скрипте адрес и порт должны быть такими же
clientID = sim.simxStart('127.0.0.1', 19999, True,
                         True, 5000, 1)

if clientID != -1:  # check if client connection successful
    print('Connected to remote API server')

else:
    print('Connection not successful')
    sys.exit('Could not connect')


scene_1.main(clientID)
