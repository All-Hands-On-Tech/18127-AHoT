# Heading Diagnostic Summary

## Issue Description
The robot's heading appears to be "stuck between 0 and 1" instead of properly ranging in radians (0 to 2π ≈ 6.28) or degrees (0 to 360).

## Potential Causes

### 1. Pinpoint Localizer Configuration
The GoBilda Pinpoint localizer uses odometry pods to calculate heading. If there's an issue with:
- Encoder direction configuration
- Pod offset measurements  
- Encoder resolution settings

This could cause incorrect heading calculations.

### 2. Heading Normalization Issue
Some possibilities:
- Heading is being normalized to 0-1 range instead of 0-2π
- IMU offset is incorrectly applied
- Unit conversion error (degrees vs radians vs normalized)

### 3. PID Output Clamping
The heading PID controller might be outputting values that are clamped to 0-1, preventing proper rotation control.

## Changes Made for Diagnostics

### PedroAutonomous.java
Added enhanced telemetry to display:
- Current heading in both degrees (normalized to 0-360) and radians
- Robot position (X, Y)
- Path following state (isBusy)
- Path state machine status

This will help identify if the heading values are correct but just displayed oddly, or if there's an actual calculation error.

## Testing Steps

### Step 1: Check Raw Heading Values
1. Run the PedroAuto OpMode
2. Watch the telemetry for "Heading (rad)" and "Heading (deg)"
3. **Expected**: 
   - Heading (rad) should range from 0 to ~6.28
   - Heading (deg) should range from 0 to 360
4. **If stuck at 0-1**: Heading calculation is broken

### Step 2: Manual Rotation Test
1. Place robot on field
2. Manually rotate the robot 90 degrees
3. Check if heading changes by π/2 (1.57 rad) or 90°
4. **If heading doesn't change proportionally**: Localizer issue

### Step 3: Check Starting Pose
Current starting pose: `(120.316, 128.707, 38°)` or `(120.316, 128.707, 0.663 rad)`

If the telemetry shows heading starting at a value between 0-1 that's NOT 0.663, then there's a unit conversion problem.

## Possible Fixes

### Fix 1: Check Pinpoint IMU Offset
The Pinpoint device might have an IMU that needs proper configuration. Check if there's an IMU offset setting in the Pinpoint initialization.

```java
// In Constants.java, after localizerConstants
// You may need to add:
.imuOffset(0.0)  // or whatever the correct method is
```

### Fix 2: Encoder Resolution Check
Verify the encoder resolution matches your actual pods:
```java
.encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
```

If you're using different pods (e.g., `goBILDA_SWINGARM_POD`), this needs to change.

### Fix 3: Pod Offsets
Current offsets:
- Forward pod Y: 4 inches
- Strafe pod X: 1.6 inches

If these are incorrect, heading calculations will be wrong. Measure from the center of rotation to each pod.

### Fix 4: Heading Scale Factor
If the Pinpoint is configured to return heading in rotations (0-1) instead of radians, you may need to add a scale factor:

```java
// This would need to be added in the follower update or pose retrieval
double heading = follower.getPose().getHeading() * 2 * Math.PI;
```

But this should be handled by the library, not manually.

## Next Steps

1. **Run the OpMode** and observe the telemetry values
2. **Report back** the actual values you see for:
   - Heading (rad): ___
   - Heading (deg): ___
   - Does heading change when you rotate the robot? ___

3. Based on those values, we can determine if this is:
   - A display issue (heading is correct but shown incorrectly)
   - A calculation issue (heading is calculated incorrectly)
   - A configuration issue (Pinpoint settings are wrong)

