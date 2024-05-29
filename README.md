# In CoppeliaSim scene
#### In main script of CoppelliaSim project:
```Lua
require('defaultMainScript')

sim.setThreadSwitchTiming(2)
simRemoteApi.start(19999)
```

# Local stream
#### If flask is used stream can be found:
##### http://127.0.0.1:5000
##### http://192.168.0.43:5000

# Configuration
#### Configuration file should be named as configuration.json.
#### It must be located in data folder.
#### Example of configuration.json:
```Json
{
 "IS_EMULATION": true,
 "PRINT_ONLY_IMPORTANT_LOGS": true,
 "BUILD_PLOT": false,
 "USE_FLASK": true,
 "FLASK_DELAY": 0.2,
 "ARUCO_DICTIONARY": "DICT_4X4_1000"
}
```

# Dictionary
#### Dictionary file also must be located in data folder.
#### Example of dictionary.json:
``` Json
{
 "ANGLES":
 {
 "0": -97,
 "20": 49,
 "15": -19,
 "6": -4
 },
 "FINISH": 13
}
```
