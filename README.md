# Vehicle Sim

### Compiling from source
#### Requirements
- CMake version 3.20+
- C++ compiler (compiles with MSVC or Clang and probably many others)

#### Clone repository
````bash
>> git clone --recurse-submodules https://github.com/doddm...
````

#### Unix 
Build out of source. From the root directory of the project:

```bash
>> mkdir cmake_build
>> cd cmake_build
>> cmake ..
>> make VehicleSim
```

#### Windows

### Vehicle controls:
- Accelerate:        Forward cursor 
- Brake:             Back cursor
- Steer left:        Left cursor
- Steer right:       Right cursor
- Quit application:  Esc

| Action      |   Key input   |
|-------------|:-------------:|
| Accelerate  | Forward cursor |
| Brake       |    Back cursor  |
| Steer left |    Left cursor   |
| Steer right |    Right cursor   |
| Quit application |    Esc    |

### Camera controls:
| Action           |          Input           |
|------------------|:------------------------:|
| Pitch/Yaw camera | Ctrl + Left click mouse  |
| Zoom in/out      | Ctrl + Right click mouse |
| Zoom in/out        |      Mouse wheel         |


