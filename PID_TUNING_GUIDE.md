# Deposit Motor PID Tuning Guide

## Overview
The DepositTuner is simplified to just control the deposit target velocity. PID tuning is done manually in Panels.

## Default Settings
- Target Velocity: 850 ticks/second
- kP (Proportional): 0.0002
- kI (Integral): 0.00000050
- kD (Derivative): 0.0001
- kFF (Feedforward): 0.00018

## Problem You're Solving
The motors were speeding up too fast, then overshooting, coming back down, undershooting, and slowly settling. This is a classic overshoot problem that requires tuning the PID gains via Panels.

## DepositTuner Controls

### Simple Controls
1. Press gamepad2 **X** to toggle the deposit on/off
2. Use gamepad2 **Dpad Up/Down** to adjust the target velocity (in 10 tick increments)
3. Observe motor velocities on telemetry

### Tuning in Panels

The PID values (kP, kI, kD) will be adjusted in Panels. Here's the strategy:

#### If motors overshoot and oscillate:
1. **Decrease kP** in Panels (reduce proportional gain) - too much causes overshoot
2. **Increase kD** in Panels (increase derivative gain) - dampens oscillation
3. Consider slightly increasing kI to help settle faster

#### If motors are too slow to reach target:
1. **Increase kP** in Panels (increase proportional gain)
2. **Increase kFF** in Panels (feedforward) - proportional push toward target
3. If it takes too long, increase kI slightly

#### If motors oscillate around the target:
1. **Increase kD** in Panels (derivative helps prevent oscillation)
2. **Decrease kP** in Panels (reduce aggressive response)
3. **Decrease kI** in Panels (integral can cause overshoot)

#### If motors never quite reach target (steady-state error):
1. **Increase kI** in Panels (integral eliminates steady-state error)
2. Make sure kFF is appropriate for your motor specs


### Telemetry Information
- **Target Velocity**: Current target in ticks/sec
- **Status**: ACTIVE or inactive
- **Deposit Velocities**: Current motor velocity (L and R motors) in ticks/sec

## Testing Your Tuning
1. Set a target velocity (e.g., 850 ticks/sec)
2. Press X to activate
3. Observe the motor ramp up on telemetry
4. Look for:
   - Smooth acceleration to target
   - Minimal overshoot (error should peak <20%)
   - Settling within 0.5-1.5 seconds
   - Stable hold at target velocity

## Advanced Notes
- Feedforward (kFF) is multiplied by the target velocity, so it provides proportional power
- The PID controller resets when target reaches 0, preventing integral windup
- Output is clamped to -1.0 to 1.0 (max motor power)
- Integral limits prevent windup at -1000 to 1000

## Next Steps
Once you achieve good tuning:
1. Note the final kP, kI, kD, kFF values
2. Update them in the DepositTuner code as defaults
3. Copy the same values to PedroAutonomous for autonomous use

