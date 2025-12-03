# Vision Localization in Deposit Tuner

## Overview

The Deposit Tuner now includes AprilTag vision localization to display the robot's field position in real-time using the FTC Field Coordinate System.

## What Was Added

### Vision Integration Features:
1. **AprilTag Detection** - Automatically detects DECODE season AprilTags
2. **Field Coordinates** - Displays position in millimeters (X, Y coordinates)
3. **Heading Display** - Shows robot orientation in degrees
4. **Tag Information** - Lists detected tag IDs and names
5. **No Driver Input Required** - Vision runs automatically in the background

### Display Format:
```
=== VISION LOCALIZATION ===
Tags Detected: 2
Field X: 1487.3 mm
Field Y: -2543.8 mm
Heading: 45.2°
Tag IDs: 11 (Blue Observation Zone), 15 (Red Submersible)
```

## Field Coordinate System (DECODE)

The DECODE field uses an **inverted** square field configuration:

### Origin:
- **Center of field** where the four center tiles meet
- Z = 0 at the top surface of the floor mat

### Axes (from Red Alliance perspective):
- **X-axis**: Negative (away from audience) to Positive (toward audience)
  - Red Wall is on the **left** as seen from audience
  - X increases toward the audience
  
- **Y-axis**: Negative (toward Red Alliance) to Positive (toward Blue Alliance)
  - Y increases across the field from Red to Blue
  
- **Z-axis**: Up from the floor
  - Z increases upward

### Heading:
- **0°** = Facing away from Red Alliance (toward Blue)
- Increases counter-clockwise when viewed from above
- **90°** = Facing toward audience
- **180°/-180°** = Facing toward Red Alliance
- **-90°/270°** = Facing away from audience

## Camera Configuration

Adjust these constants at the top of DepositTuner.java to match your camera mounting:

```java
private static final double CAMERA_X_OFFSET_MM = 0.0;    // mm right from robot center
private static final double CAMERA_Y_OFFSET_MM = 0.0;    // mm forward from robot center
private static final double CAMERA_Z_OFFSET_MM = 152.4;  // mm up (6 inches default)
private static final double CAMERA_YAW = 0.0;            // degrees (0 = forward)
private static final double CAMERA_PITCH = -90.0;        // degrees (-90 = horizontal)
private static final double CAMERA_ROLL = 0.0;           // degrees (0 = upright)
```

### Camera Position Measurement:
- Measure from robot's center of rotation
- Positive X = Right, Positive Y = Forward, Positive Z = Up
- Yaw: 0° = forward, 90° = left, -90° = right
- Pitch: -90° = horizontal pointing forward (typical)
- Roll: 0° = camera upright

## Coordinate Units

All vision coordinates are displayed in **millimeters** (mm):
- 1 inch = 25.4 mm
- Field dimensions: approximately 3580 mm x 3580 mm

This matches the FTC documentation which uses mm for field coordinates.

## How It Works

1. **AprilTag Detection**: Camera continuously scans for AprilTags
2. **Pose Calculation**: SDK calculates robot position relative to each detected tag
3. **Tag Library**: Uses official DECODE season tag positions
4. **Averaging**: When multiple tags are visible, positions are averaged for accuracy
5. **Coordinate Conversion**: SDK provides inches, converted to mm for display

## Using Vision Data

### In TeleOp:
- Monitor position while driving for awareness
- Check alignment with game elements
- Verify robot location on field

### In Autonomous:
```java
// Vision data is available in the main loop
if (visionTagCount > 0) {
    // Use visionX_mm, visionY_mm, visionHeading
    // Navigate to target position
    double errorX = targetX_mm - visionX_mm;
    double errorY = targetY_mm - visionY_mm;
    // ... drive control logic
}
```

## Sensor Fusion (Future Enhancement)

Vision localization works alongside odometry:
- **Odometry**: Continuous, smooth position tracking
- **Vision**: Absolute position from tags, corrects drift
- **Combined**: Use vision to reset/correct odometry periodically

Example fusion logic:
```java
if (visionTagCount >= 2) {  // High confidence with 2+ tags
    // Trust vision more
    double fusedX = 0.7 * visionX_mm + 0.3 * pos.getXmm();
    double fusedY = 0.7 * visionY_mm + 0.3 * pos.getYmm();
} else if (visionTagCount == 1) {  // Lower confidence
    // Balance vision and odometry
    double fusedX = 0.4 * visionX_mm + 0.6 * pos.getXmm();
    double fusedY = 0.4 * visionY_mm + 0.6 * pos.getYmm();
} else {
    // No vision, use odometry only
    double fusedX = pos.getXmm();
    double fusedY = pos.getYmm();
}
```

## Troubleshooting

### "Tags Detected: 0"
- Check camera is connected and configured as "Webcam 1"
- Ensure AprilTags are visible in camera view
- Verify adequate lighting
- Tags must be from DECODE season tag library

### Wrong Coordinates
- Verify camera offset constants match your mounting
- Check camera orientation (yaw, pitch, roll)
- Ensure tags are flat and not damaged
- Calibrate camera if possible

### Poor Performance
- Vision runs at decimation = 2 (balanced)
- Reduces to decimation = 3 for more FPS if needed
- Vision runs in background, doesn't affect driving

### Position Jumps
- Normal with single tag detection
- More stable with multiple visible tags
- Use sensor fusion with odometry for smooth tracking

## DECODE Season Tag Positions

AprilTags in the DECODE season are located at:
- Goals (Red and Blue)
- Observation Zones
- Other field elements

The SDK automatically knows these positions from the tag library.

## Additional Resources

- FTC Field Coordinate System: https://ftc-docs.firstinspires.org/field-coordinate-system
- AprilTag Documentation: https://ftc-docs.firstinspires.org/apriltag
- DECODE Game Manual: Check for official tag positions

## Performance Notes

- Vision processing runs every loop cycle (~50 Hz)
- Decimation = 2 provides ~22 FPS camera processing
- Minimal impact on drive performance
- Camera can be disabled by modifying code if not needed

---

**Vision is now integrated!** Drive around and watch the field coordinates update in real-time.

