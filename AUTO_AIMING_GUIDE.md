# Auto-Aiming Feature Guide

## Overview

The DepositTuner now includes **alliance selection** and **auto-aiming** functionality that uses AprilTag vision to automatically rotate the robot to face the alliance goal.

## Features Added

### 1. Alliance Selection (Initialization)
During the initialization phase (before pressing START):
- **Driver 1 presses A** → Sets robot to **BLUE** alliance
- **Driver 1 presses B** → Sets robot to **RED** alliance
- Selection can be changed any time before START
- Current alliance is displayed on screen

### 2. Auto-Aiming (During TeleOp)
When the OpMode is running:
- **Driver 1 holds X** → Activates auto-aiming
- Robot automatically rotates to face alliance goal
- Rotation direction depends on alliance:
  - **BLUE**: Rotates LEFT (counterclockwise) to heading 0°
  - **RED**: Rotates RIGHT (clockwise) to heading 0°
- Rotation speed: **50%** (configurable)
- Stops when within **2°** of target heading

## How It Works

### Alliance Selection Phase:
```
=== ALLIANCE SELECTION ===
Driver 1: Press A for BLUE
Driver 1: Press B for RED

Current Alliance: BLUE

---
Init complete - waiting start
Gamepad1: Hold X for auto-aim to alliance goal
```

### Auto-Aiming Logic:
1. **Detect Alliance Tag**: Vision searches for AprilTags with "blue" or "red" in the name
2. **Check Heading**: Compares current robot heading to target (0°)
3. **Rotate**: Automatically rotates in the correct direction
4. **Stop**: When heading is within 2° of target

### Rotation Directions:
- **BLUE Alliance**: Rotates **LEFT** (positive rotation, counterclockwise)
- **RED Alliance**: Rotates **RIGHT** (negative rotation, clockwise)
- **Target**: Both aim for **0° heading**

## Usage

### Initialization:
1. Start the OpMode
2. **Press A** for Blue Alliance or **Press B** for Red Alliance
3. Wait for alliance selection to appear
4. Press START when ready

### During TeleOp:
1. Drive normally with joysticks
2. **Hold X** to activate auto-aiming
3. Robot will rotate automatically toward 0° heading
4. Release X to regain manual control
5. Driver can still strafe/move forward while auto-aiming

## Telemetry Display

### During Auto-Aim:
```
Alliance: BLUE
Auto-Aim: ACTIVE - Rotating to 0°
```

### Searching for Tag:
```
Alliance: RED
Auto-Aim: Searching for alliance tag...
```

### Target Aligned:
```
Alliance: BLUE
Auto-Aim: Target aligned!
```

## Configuration Constants

At the top of DepositTuner.java:

```java
// Auto-aiming constants
private static final double AUTO_AIM_ROTATION_SPEED = 0.5;  // 50% speed
private static final double AUTO_AIM_HEADING_TOLERANCE = 2.0;  // degrees
```

### Adjusting Settings:
- **AUTO_AIM_ROTATION_SPEED**: 0.0 to 1.0 (0.5 = 50% speed)
- **AUTO_AIM_HEADING_TOLERANCE**: Degrees within target to stop (2.0° default)

## Technical Details

### Tag Detection:
- Searches for tags with metadata containing "blue" or "red"
- Uses `detection.metadata.name.toLowerCase().contains()`
- Examples: "Blue Observation Zone", "Red Goal", etc.

### Heading Calculation:
```java
double headingError = -fusedHeading;  // Target is 0°
if (Math.abs(headingError) > AUTO_AIM_HEADING_TOLERANCE) {
    // Rotate toward target
}
```

### Rotation Control:
```java
if (robotAlliance == Alliance.BLUE) {
    rotate = AUTO_AIM_ROTATION_SPEED;  // Left (positive)
} else {
    rotate = -AUTO_AIM_ROTATION_SPEED; // Right (negative)
}
```

### Manual Override:
- Driver can still control forward/strafe while auto-aiming
- Manual rotation is disabled while X is held
- Release X to regain full manual control

## Examples

### Scenario 1: Blue Alliance Scoring
1. Select BLUE during init
2. Drive near the blue goal
3. Hold X - robot rotates left to 0°
4. When aligned, release X and drive forward
5. Score in blue goal

### Scenario 2: Red Alliance Scoring
1. Select RED during init
2. Drive near the red goal
3. Hold X - robot rotates right to 0°
4. When aligned, release X and drive forward
5. Score in red goal

## Troubleshooting

### "Searching for alliance tag..."
- **Cause**: Alliance tag not visible to camera
- **Solution**: 
  - Drive closer to alliance goal
  - Adjust camera angle
  - Ensure lighting is adequate
  - Verify correct alliance is selected

### Robot rotates wrong direction
- **Check**: Verify correct alliance selected (A=Blue, B=Red)
- **Check**: Ensure tag is detected (watch telemetry)
- **Solution**: Reselect alliance if needed

### Auto-aim doesn't activate
- **Check**: X button is held (not just tapped)
- **Check**: Alliance is selected (not NONE)
- **Check**: Alliance tag is visible
- **Check**: Vision system is initialized

### Robot oscillates around target
- **Cause**: Heading tolerance too tight
- **Solution**: Increase `AUTO_AIM_HEADING_TOLERANCE` to 3.0 or 4.0

### Rotation too fast/slow
- **Solution**: Adjust `AUTO_AIM_ROTATION_SPEED`
  - Too fast: Reduce to 0.3 or 0.4
  - Too slow: Increase to 0.6 or 0.7

## Code Flow

```
INIT PHASE:
├─ Driver presses A or B
├─ robotAlliance set to BLUE or RED
└─ Display alliance selection

MAIN LOOP:
├─ Update sensors (odometry + vision)
├─ Calculate fused position
├─ Check if X is held
├─ IF X held AND alliance set:
│  ├─ Search for alliance tag
│  ├─ IF tag found:
│  │  ├─ Calculate heading error
│  │  ├─ IF error > tolerance:
│  │  │  └─ Set rotation (LEFT for Blue, RIGHT for Red)
│  │  └─ ELSE: Stop rotation (aligned)
│  └─ ELSE: Continue searching
└─ Apply drive powers
```

## Safety Features

1. **No Alliance = No Auto-Aim**: Must select alliance first
2. **Manual Override**: Release X to regain control immediately
3. **Tolerance Window**: Stops rotating within 2° to prevent oscillation
4. **Vision Dependent**: Only rotates when alliance tag is visible
5. **Speed Limited**: Capped at 50% rotation speed for safety

## Future Enhancements

Possible improvements:
- Auto-approach to goal (forward/strafe automation)
- Multiple target positions (high/low goal)
- PID control for smoother rotation
- Distance-based scoring automation
- Multiple alliance tag support

## Testing Checklist

- [ ] Select Blue alliance - verify displayed correctly
- [ ] Select Red alliance - verify displayed correctly
- [ ] Hold X with Blue - robot rotates left
- [ ] Hold X with Red - robot rotates right
- [ ] Robot stops at 0° heading
- [ ] Release X - manual control restored
- [ ] Auto-aim works while driving forward
- [ ] No auto-aim when alliance = NONE
- [ ] Telemetry shows correct status

---

**Auto-aiming is complete!** Drivers can now automatically aim at the alliance goal by holding X.

