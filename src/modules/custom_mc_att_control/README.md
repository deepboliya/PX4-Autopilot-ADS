# Custom Minimal Attitude Controller

This is a simplified attitude controller designed for research and educational purposes. It provides basic PID attitude control without the complexity of the full `mc_att_control` module.

## Features

- **Minimal Code**: Easy to read and understand
- **Simple PID Control**: Basic proportional-integral-derivative controller
- **Research Ready**: Easy to modify for custom control algorithms
- **Safe**: Built-in rate limiting and safety checks

## Architecture

```
Input:  vehicle_attitude_setpoint (desired attitude quaternion)
        ↓
Controller: Simple PID on quaternion error
        ↓
Output: vehicle_rates_setpoint (desired angular rates)
```

## Usage

### Build and Run

```bash
# Build PX4 with the custom controller
make px4_sitl

# Start PX4 simulation
make px4_sitl gz_x500

# In PX4 shell, stop the default attitude controller
mc_att_control stop

# Start the custom attitude controller
custom_mc_att_control start

# Check status
custom_mc_att_control status

# Monitor output
listener vehicle_rates_setpoint
```

### Parameters

The controller uses standard PX4 attitude parameters:
- `MC_ROLL_P`: Roll proportional gain
- `MC_PITCH_P`: Pitch proportional gain
- `MC_YAW_P`: Yaw proportional gain

You can tune these in QGroundControl or via the PX4 shell:
```bash
param set MC_ROLL_P 6.5
param set MC_PITCH_P 6.5
param set MC_YAW_P 2.8
```

## Customization

The controller is designed to be easily modified for research:

### 1. Replace PID Controller
Edit the `pid_controller()` function in `custom_mc_att_control.cpp`:

```cpp
Vector3f CustomAttitudeControl::pid_controller(const Vector3f &error, float dt)
{
    // Your custom control algorithm here
    // e.g., sliding mode control, LQR, nonlinear control, etc.
    return your_control_output;
}
```

### 2. Add Custom Parameters
Add parameters in the header file:

```cpp
DEFINE_PARAMETERS(
    (ParamFloat<px4::params::CUSTOM_GAIN_1>) _param_custom_gain_1,
    (ParamFloat<px4::params::CUSTOM_GAIN_2>) _param_custom_gain_2
)
```

### 3. Modify Control Structure
The main control loop is in the `Run()` function - easy to modify for different control approaches.

## Safety Notes

- Always test in simulation first (`make px4_sitl gz_x500`)
- The controller has built-in rate limiting for safety
- Disable the default `mc_att_control` before starting this module
- Use conservative parameters initially

## Debugging

```bash
# Monitor performance
perf

# Check uORB topics
listener vehicle_attitude
listener vehicle_attitude_setpoint
listener vehicle_rates_setpoint

# View controller logs
dmesg | grep custom_mc_att_control
```

## Research Applications

This minimal controller is perfect for:
- Algorithm development and testing
- Control theory research
- Educational demonstrations
- Baseline comparisons
- Custom control law implementation
