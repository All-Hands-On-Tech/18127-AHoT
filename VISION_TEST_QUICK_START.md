# Vision Localization Test - Quick Start Guide

## What Was Created

A new **VisionLocalizationTest** TeleOp that demonstrates AprilTag detection and field positioning without requiring a robot.

## Key Features

1. **Self-Contained**: All vision code is in one file, no external dependencies on RobotHardware
2. **Field Coordinates**: Uses standard FTC Field Coordinate System
3. **AprilTag Detection**: Shows detected tag IDs and positions
4. **Robot Position**: Calculates and displays robot position on field based on AprilTag library
5. **No Drive Required**: Template for testing vision without robot hardware

## What It Shows

For each detected AprilTag:

### If Tag is in Library (has metadata):
- **Tag Name** (e.g., "Blue Observation Zone")
- **Robot Field Position**: X, Y, Z coordinates on the field
- **Robot Orientation**: Yaw (heading), Pitch, Roll
- **Navigation Data**: Range, Bearing, Elevation
- **Tag Field Position** (when detailed info is ON)

### If Tag is NOT in Library:
- **Tag ID** number
- **Pixel position** on camera
- **Camera-relative data** (when detailed info is ON)

## Field Coordinate System

The OpMode uses the standard FTC field coordinates:
- **Origin**: Center of the field
- **X-axis**: Negative (left) to Positive (right) from driver station
- **Y-axis**: Negative (toward driver station) to Positive (away)
- **Heading**: 0° = facing away from driver station

## Controls

- **DPAD UP**: Resume camera streaming
- **DPAD DOWN**: Stop streaming (saves CPU)
- **DPAD LEFT**: Decrease decimation (better range, slower FPS)
- **DPAD RIGHT**: Increase decimation (faster FPS, shorter range)
- **A Button**: Toggle detailed info ON/OFF

## Camera Configuration

Adjust these constants at the top of the file for your camera mounting:

```java
private static final double CAMERA_X_OFFSET = 0.0;  // inches right from robot center
private static final double CAMERA_Y_OFFSET = 0.0;  // inches forward from robot center
private static final double CAMERA_Z_OFFSET = 6.0;  // inches up from robot center
private static final double CAMERA_YAW = 0.0;       // degrees (0 = forward)
private static final double CAMERA_PITCH = -90.0;   // degrees (-90 = horizontal forward)
private static final double CAMERA_ROLL = 0.0;      // degrees (0 = upright)
```

## How to Use

1. **Connect webcam** named "Webcam 1" in Robot Configuration
   - Or it will fall back to phone camera if no webcam found

2. **Run the OpMode** from Driver Station

3. **Point camera** at DECODE AprilTags

4. **Read telemetry** to see:
   - Tag IDs detected
   - Robot position on field
   - Navigation data

5. **Use controls** to adjust decimation and toggle details

## Decimation Settings

- **1**: Best detection range, ~10 FPS (slower)
- **2**: Balanced (recommended), ~22 FPS
- **3**: Fastest, ~30 FPS, shorter range

## Next Steps

1. **Test with DECODE tags** to verify detection
2. **Adjust camera constants** to match your mounting
3. **Integrate position data** into autonomous code
4. **Combine with odometry** for sensor fusion

## Example Output

```
=== APRILTAG LOCALIZATION ===
Camera State: STREAMING
Decimation: 2 (DPAD L/R)
Tags Detected: 1

--- TAG ID 11 ---
Name: Blue Observation Zone

ROBOT FIELD POSITION:
  X (Right): 24.5 inches
  Y (Forward): 48.2 inches
  Z (Up): 0.8 inches

ROBOT ORIENTATION:
  Yaw (Heading): 15.3°
  Pitch: -1.2°
  Roll: 0.5°

NAVIGATION DATA:
  Range: 36.7 inches
  Bearing: 12.4°
  Elevation: -5.2°

--- CONTROLS ---
DPAD UP: Resume | DOWN: Stop
DPAD L/R: Decimation
A: Details OFF
```

## Files

- **VisionLocalizationTest.java** - Main test OpMode (self-contained)
- **VisionLocalization.java** - Reusable class for robot integration (future use)
- **VISION_LOCALIZATION_GUIDE.md** - Comprehensive documentation

## Troubleshooting

- **No camera found**: Check webcam is plugged in and configured
- **No tags detected**: Ensure AprilTags are visible and well-lit
- **Wrong positions**: Verify camera offset constants match your setup
- **Low FPS**: Increase decimation or reduce resolution

## For DECODE Season

This test OpMode is designed for the DECODE season's AprilTag library. Tags are automatically recognized if they're in the official season library. Custom tags can be added by modifying the tag library in the code.

---

**Ready to test!** Run "Vision Localization Test" from Driver Station.

