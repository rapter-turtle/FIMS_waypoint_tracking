# bada_msg

ROS 2 message definitions for the Bada project. This package contains custom message types used across the Bada autonomous vessel system.

## Message Types

The package contains the following message types:

### Mode

```plaintext
int8 mode
```

Represents the operational mode of the system.

### ActuatorOutputs

```plaintext
uint32 active
float32[32] actuator
```

Represents the state of actuators in the system.

### ControlParameters

```plaintext
float64 acceptance_radius  
float64 kp
float64 kd 
float64 desired_velocity 
float64 max_steer
float64 max_thrust_diff
float64 max_steer_diff
float64 max_thrust
float64 kup 
float64 kud
float64 kui
```

Control parameters for the vessel navigation system.

### PlcStatus

```plaintext
int32 auto_control_status
int32 emergency_stop_status
int32 engine_rpm_status
int32 clutch_status
int32 steering_angle_status 
int32 trim_angle_status
int32 engine_running_status 
int32 bow_thruster_power_status 
int32 bow_thruster_rev_status
```

Status information from the vessel's PLC (Programmable Logic Controller).

### Attitude

```plaintext
float32 roll
float32 pitch
float32 yaw
```

Vessel orientation in 3D space.

### GlobalPosition

```plaintext
float64 lat
float64 lon
float64 alt
```

Global position in latitude, longitude, and altitude.

### Speed

```plaintext
float32 vx
float32 vy
float32 vz
```

Linear velocities along the three axes.

## Usage

To use these messages in your ROS 2 package, add the following to your `package.xml`:

```xml
<depend>bada_msg</depend>
```

And in your CMakeLists.txt:

```cmake
find_package(bada_msg REQUIRED)
```

Then in your code:

```cpp
#include "bada_msg/msg/attitude.hpp"
// Or any other message type you need
```

## Usage

To use these messages in your ROS 2 package, add the following to your `package.xml`:

```xml
<depend>bada_msg</depend>
```

And in your CMakeLists.txt:

```cmake
find_package(bada_msg REQUIRED)
```

Then in your code:

```cpp
#include "bada_msg/msg/attitude.hpp"
// Or any other message type you need
```