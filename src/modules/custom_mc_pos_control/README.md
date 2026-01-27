# Custom MC Position Control

A minimal position controller optimized for the Gazebo X500 quadcopter.

## Overview

This module provides a simplified implementation of multicopter position control, specifically designed for the X500 quadcopter in Gazebo simulation. It removes all VTOL-related complexity, goto control, and other overhead not needed for basic quadcopter operation.

## Architecture

The controller uses a cascaded control structure:

1. **Position P-loop**: Takes position setpoints and generates velocity setpoints
2. **Velocity PID-loop**: Takes velocity setpoints and generates acceleration setpoints  
3. **Acceleration Control**: Converts acceleration setpoints to thrust vectors
4. **Attitude Generation**: Converts thrust vectors to attitude setpoints

## X500 Specific Configuration

The module is pre-configured with X500-optimized defaults:

| Parameter | Value | Description |
|-----------|-------|-------------|
| Hover Thrust | 0.5 | Normalized hover thrust |
| Max Tilt | 35° | Maximum tilt angle |
| Max XY Velocity | 12 m/s | Horizontal speed limit |
| Max Z Velocity Up | 3 m/s | Upward speed limit |
| Max Z Velocity Down | 1.5 m/s | Downward speed limit |
| Min Thrust | 0.12 | Minimum normalized thrust |
| Max Thrust | 0.9 | Maximum normalized thrust |

## Parameters Used

The controller uses standard PX4 MPC parameters:

### Position Gains
- `MPC_XY_P` - Horizontal position P gain
- `MPC_Z_P` - Vertical position P gain

### Velocity Gains  
- `MPC_XY_VEL_P_ACC` - Horizontal velocity P gain
- `MPC_XY_VEL_I_ACC` - Horizontal velocity I gain
- `MPC_XY_VEL_D_ACC` - Horizontal velocity D gain
- `MPC_Z_VEL_P_ACC` - Vertical velocity P gain
- `MPC_Z_VEL_I_ACC` - Vertical velocity I gain
- `MPC_Z_VEL_D_ACC` - Vertical velocity D gain

### Limits
- `MPC_XY_VEL_MAX` - Maximum horizontal velocity
- `MPC_Z_VEL_MAX_UP` - Maximum upward velocity
- `MPC_Z_VEL_MAX_DN` - Maximum downward velocity
- `MPC_THR_HOVER` - Hover thrust
- `MPC_THR_MIN` - Minimum thrust
- `MPC_THR_MAX` - Maximum thrust
- `MPC_TILTMAX_AIR` - Maximum tilt in air

### Options
- `MPC_USE_HTE` - Use hover thrust estimator

## What's Removed (vs mc_pos_control)

- VTOL support
- GotoControl module
- Takeoff state machine
- Complex parameter validation
- Velocity notch filters
- Manual mode handling
- Position mode complexity
- Cruise speed handling
- Landing speed adjustments
- Multi-vehicle responsiveness tuning

## Usage

```bash
# Start the module
custom_mc_pos_control start

# Stop the module
custom_mc_pos_control stop

# Check status
custom_mc_pos_control status
```

## Integration

This module publishes:
- `vehicle_attitude_setpoint` - Attitude commands for the attitude controller
- `vehicle_local_position_setpoint` - Position/velocity/acceleration setpoints

This module subscribes to:
- `vehicle_local_position` - Current position/velocity estimates
- `trajectory_setpoint` - Desired position/velocity/acceleration
- `vehicle_control_mode` - Control mode flags
- `vehicle_land_detected` - Landing state
- `hover_thrust_estimate` - Estimated hover thrust (optional)

## Building

Add the module to your board configuration or use:

```cmake
set(CONFIG_MODULES_CUSTOM_MC_POS_CONTROL yes)
```

## Notes

- This module is designed for research and educational purposes
- It assumes a pure quadcopter configuration (no VTOL)
- It works with the standard PX4 attitude and rate controllers
- For production use, consider the full `mc_pos_control` module
