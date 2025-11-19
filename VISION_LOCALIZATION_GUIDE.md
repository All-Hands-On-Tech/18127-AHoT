# Vision Localization Guide

## Overview

The `VisionLocalization` class provides AprilTag-based position tracking for your robot. It uses the camera to detect AprilTags on the field and calculates the robot's precise position and orientation.

## Files Created

1. **VisionLocalization.java** - Core vision localization class
2. **VisionLocalizationTest.java** - Demo TeleOp showing how to use the system

## Quick Start

### 1. Basic Setup

```java
// In your OpMode's runOpMode() method:
VisionLocalization vision = new VisionLocalization(hardwareMap);

// In your main loop:
while (opModeIsActive()) {
    vision.update();  // Call this every loop!
    
    // Get robot position:
    double x = vision.getRobotX();        // inches
    double y = vision.getRobotY();        // inches
    double heading = vision.getRobotHeading();  // degrees
    
    // Your other code...
}

// When done:
vision.close();
```

### 2. Custom Camera Position

If your camera is NOT at the center of your robot, you need to specify its offset:

```java
// VisionLocalization(hardwareMap, camX, camY, camZ, camYaw, camPitch, camRoll)
VisionLocalization vision = new VisionLocalization(
    hardwareMap,
    0,      // camX: 0 inches right of center
    6,      // camY: 6 inches forward of center
    2,      // camZ: 2 inches above robot center
    0,      // camYaw: 0° (facing forward)
    -90,    // camPitch: -90° (horizontal)
    0       // camRoll: 0° (upright)
);
```

#### Camera Position Reference:
- **X**: Distance right (+) or left (-) from robot center (inches)
- **Y**: Distance forward (+) or backward (-) from robot center (inches)
- **Z**: Distance up (+) or down (-) from robot center (inches)

#### Camera Orientation Reference:
- **Yaw**: 0° = forward, +90° = left, -90° = right, 180° = backward
- **Pitch**: -90° = horizontal forward (typical), 0° = pointing up
- **Roll**: 0° = upright, 90° = rotated 90° clockwise

## Key Methods

### Position Tracking
- `update()` - **MUST call every loop** to process new camera frames
- `getRobotX()` - Get X position in inches
- `getRobotY()` - Get Y position in inches
- `getRobotHeading()` - Get heading in degrees

### Detection Info
- `getDetectedTagCount()` - Number of tags currently visible
- `getConfidence()` - Confidence score 0.0-1.0 (higher = more reliable)
- `hasRecentDetection()` - True if tags detected in last 1 second
- `getDetections()` - Get list of all detected AprilTags

### Camera Control
- `stopStreaming()` - Stop camera to save CPU (when not needed)
- `resumeStreaming()` - Resume camera streaming
- `setDecimation(int)` - Trade detection range for speed:
  - `1` = Best range, slower FPS (~10 FPS)
  - `2` = Balanced (recommended, ~22 FPS)
  - `3` = Fastest, shorter range (~30 FPS)

### Status Checks
- `isReady()` - True when camera is streaming and ready
- `getCameraState()` - Get current camera state as string

## Field Coordinate System

The vision system uses the FTC field coordinate system:
- **Origin**: Center of the field
- **X-axis**: Left (-) to Right (+) when facing away from driver station
- **Y-axis**: Toward driver station (-) to Away from driver station (+)
- **Heading**: 0° = facing away from driver station, increases counter-clockwise

## Integration with Odometry

You can combine vision localization with your existing odometry for robust positioning:

```java
// In your main loop:
vision.update();
odometry.update();

// When you have high confidence vision data, you can correct odometry:
if (vision.getConfidence() > 0.7 && vision.hasRecentDetection()) {
    // Use vision position
    double x = vision.getRobotX();
    double y = vision.getRobotY();
    double heading = vision.getRobotHeading();
    
    // Optionally: Reset odometry to match vision
    // odometry.setPosition(x, y, heading);
} else {
    // Use odometry position when vision is unavailable
    Odometry.Position pos = odometry.getPosition();
    // Use pos.getXmm(), pos.getYmm(), pos.getHeadingDeg()
}
```

## Example: Autonomous with Vision

```java
@Autonomous(name = "Vision Auto Example")
public class VisionAutoExample extends LinearOpMode {
    @Override
    public void runOpMode() {
        RobotHardware hw = new RobotHardware(hardwareMap);
        VisionLocalization vision = new VisionLocalization(hardwareMap);
        
        waitForStart();
        
        while (opModeIsActive()) {
            vision.update();
            
            // Get current position
            double currentX = vision.getRobotX();
            double currentY = vision.getRobotY();
            
            // Navigate to target (example: x=24, y=36)
            double targetX = 24.0;
            double targetY = 36.0;
            
            double errorX = targetX - currentX;
            double errorY = targetY - currentY;
            
            // Simple proportional control
            double forwardPower = errorY * 0.05;  // Adjust gain
            double strafePower = errorX * 0.05;
            
            // Calculate wheel powers for mecanum drive
            double fl = forwardPower + strafePower;
            double fr = forwardPower - strafePower;
            double bl = forwardPower - strafePower;
            double br = forwardPower + strafePower;
            
            hw.setDrivePowers(fl, fr, bl, br);
            
            // Check if at target
            double distance = Math.hypot(errorX, errorY);
            if (distance < 2.0) {  // Within 2 inches
                hw.setDrivePowers(0, 0, 0, 0);
                break;
            }
            
            sleep(20);
        }
        
        vision.close();
    }
}
```

## Troubleshooting

### "Camera State: NOT_INITIALIZED"
- Check that webcam is plugged in and configured in Robot Configuration
- Verify webcam name is "Webcam 1" or update code to match your name
- Try power cycling the Control Hub

### "Tags Detected: 0"
- Make sure AprilTags are visible to camera
- Check lighting conditions (too bright/dark can affect detection)
- Adjust decimation: Lower decimation (1) = better range
- Verify camera is pointed at tags

### Poor Detection / Low Confidence
- Increase lighting in the room
- Clean camera lens
- Get closer to AprilTags
- Reduce decimation value
- Make sure tags are not at extreme angles

### Position Jumps Around
- This is normal with single tag detection
- Multiple visible tags will improve stability
- Consider filtering: Average position over several frames
- Use sensor fusion with odometry for smooth position

## Camera Configuration

Make sure your webcam is configured in the Robot Controller:
1. Connect to Robot Controller via Driver Station
2. Go to Settings → Configure Robot
3. Add Webcam with name "Webcam 1" (or update code to match)

## Performance Tips

1. **Use appropriate decimation**: Start with 2, adjust based on needs
2. **Stop streaming when not needed**: Save CPU for other tasks
3. **Limit telemetry updates**: Update at reasonable rate (not every loop)
4. **Use confidence scores**: Don't trust low-confidence detections
5. **Combine with odometry**: Vision for absolute position, odometry for smooth tracking

## Next Steps

1. Run `VisionLocalizationTest` TeleOp to verify setup
2. Adjust camera position parameters to match your robot
3. Test detection range at different decimation levels
4. Integrate with your existing autonomous code
5. Implement sensor fusion with odometry for best results

## Additional Resources

- FTC AprilTag Documentation: https://ftc-docs.firstinspires.org/en/latest/apriltag/vision_portal/apriltag_intro/apriltag-intro.html
- Field Coordinate System: https://ftc-docs.firstinspires.org/en/latest/game_specific_resources/field_coordinate_system/field-coordinate-system.html

