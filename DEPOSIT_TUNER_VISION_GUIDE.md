# Vision Localization in Deposit Tuner

## Overview

The Deposit Tuner now includes AprilTag vision localization with **sensor fusion** to display the robot's field position in real-time using the FTC Field Coordinate System. The system combines odometry and vision data for optimal position tracking.

## What Was Added

### Vision Integration Features:
1. **AprilTag Detection** - Automatically detects DECODE season AprilTags
2. **Sensor Fusion** - Intelligently combines odometry and vision data
3. **Three Position Sources** - Displays fused, odometry, and vision positions separately
4. **Field Coordinates** - All positions in millimeters (X, Y coordinates)
5. **Heading Display** - Shows robot orientation in degrees
6. **Tag Information** - Lists detected tag IDs and names
7. **No Driver Input Required** - Vision runs automatically in the background

### Display Format:
```
=== ROBOT LOCALIZATION ===
FUSED POSITION (Primary):
  X: 1487.3 mm
  Y: -2543.8 mm
  Heading: 45.2°
  Fusion: 70% vision, 30% odo

ODOMETRY:
  X: 1502.1 mm
  Y: -2531.4 mm
  Heading: 44.8°

VISION (AprilTag):
  Tags Detected: 2
  X: 1481.5 mm
  Y: -2549.7 mm
  Heading: 45.4°
  Tag IDs: 11 (Blue Observation Zone), 15 (Red Submersible)
```

## Sensor Fusion Algorithm

The system uses **weighted averaging** to combine odometry and vision:

### Fusion Weights:
- **2+ Tags Detected**: 70% vision, 30% odometry (high confidence)
- **1 Tag Detected**: 40% vision, 60% odometry (moderate confidence)
- **0 Tags Detected**: 0% vision, 100% odometry (fallback)

### Why Sensor Fusion?
- **Odometry Strengths**: Smooth, continuous, high frequency
- **Odometry Weaknesses**: Accumulates drift over time
- **Vision Strengths**: Absolute position, corrects drift
- **Vision Weaknesses**: Can be noisy, depends on tag visibility
- **Fused Result**: Best of both worlds - smooth and accurate

### Position Sources:

1. **FUSED POSITION (Primary)** 
   - Weighted average of odometry and vision
   - Use this for navigation and autonomous
   - Most accurate and stable position estimate

2. **ODOMETRY**
   - Raw odometry from Pinpoint sensors
   - Continuous, smooth tracking
   - May drift over time

3. **VISION (AprilTag)**
   - Raw vision from AprilTag detection
   - Absolute field position
   - Only available when tags are visible

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

## Using Position Data

### In TeleOp:
- **Monitor FUSED position** for most accurate robot location
- **Check ODOMETRY** to see raw sensor data
- **Check VISION** to verify AprilTag detection
- Use for alignment with game elements
- Verify robot location on field

### In Autonomous (Using Fused Position):
```java
// In your main loop, after sensor updates and fusion calculation:

// Use the fused position for navigation (most accurate)
double currentX = fusedX_mm;
double currentY = fusedY_mm;
double currentHeading = fusedHeading;

// Navigate to target position
double targetX = 1500.0;  // mm
double targetY = -2000.0; // mm

double errorX = targetX - currentX;
double errorY = targetY - currentY;

// Simple proportional control
double forwardPower = errorY * 0.0005;  // Adjust gain as needed
double strafePower = errorX * 0.0005;

// Check if at target
double distance = Math.hypot(errorX, errorY);
if (distance < 50.0) {  // Within 50mm (5cm) of target
    // Reached destination
}
```

### Accessing Individual Sources:
```java
// Fused position (recommended for navigation)
double x = fusedX_mm;
double y = fusedY_mm;
double heading = fusedHeading;

// Odometry only
double odoX = odoX_mm;
double odoY = odoY_mm;
double odoHeading = odoHeading;

// Vision only (when available)
if (visionValid) {
    double visionX = visionX_mm;
    double visionY = visionY_mm;
    double visionHeading = visionHeading;
}
```

## Comparing Position Sources

### When to Trust Each Source:

**Use FUSED Position when:**
- Navigating autonomously
- Need most accurate position
- Tags are occasionally visible

**Use ODOMETRY when:**
- Need smooth, continuous tracking
- Vision is temporarily unavailable
- Short-duration movements

**Use VISION when:**
- Need absolute field position
- Correcting long-term drift
- Multiple tags are visible (high confidence)

### Monitoring Fusion Quality:
```java
// Check fusion weight to see confidence level
if (visionWeight >= 0.7) {
    // High confidence - 2+ tags visible
    // Safe to make precise movements
} else if (visionWeight >= 0.4) {
    // Moderate confidence - 1 tag visible
    // Acceptable for most operations
} else {
    // Low confidence - no tags
    // Relying on odometry only
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

