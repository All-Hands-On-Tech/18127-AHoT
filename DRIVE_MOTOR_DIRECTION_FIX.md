# Drive Motor Direction Fix

## Date: November 30, 2025

## Change Made

**File:** `/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/common/RobotHardware.java`

**Lines Modified:** 61-64

### Before (All motors inverted):
```java
if (frontLeft != null) frontLeft.setDirection(DcMotor.Direction.REVERSE);
if (frontRight != null) frontRight.setDirection(DcMotor.Direction.FORWARD);
if (backLeft != null) backLeft.setDirection(DcMotor.Direction.REVERSE);
if (backRight != null) backRight.setDirection(DcMotor.Direction.FORWARD);
```

### After (All motors reversed):
```java
if (frontLeft != null) frontLeft.setDirection(DcMotor.Direction.FORWARD);
if (frontRight != null) frontRight.setDirection(DcMotor.Direction.REVERSE);
if (backLeft != null) backLeft.setDirection(DcMotor.Direction.FORWARD);
if (backRight != null) backRight.setDirection(DcMotor.Direction.REVERSE);
```

## What Changed

- **frontLeft**: REVERSE → FORWARD
- **frontRight**: FORWARD → REVERSE
- **backLeft**: REVERSE → FORWARD
- **backRight**: FORWARD → REVERSE

All four drive motors have been flipped to the opposite direction.

## Impact

This change will affect:
- ✅ **TeleOp**: Robot movement will be reversed from before
- ✅ **Autonomous (PedroAutonomous)**: Robot will drive in opposite direction
- ✅ **Any code using RobotHardware.setDrivePowers()**

## Note

The Pedro Pathing Constants.java **also** has motor directions configured. If the robot still drives incorrectly, you may need to update those as well.

**Location:** `/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pedroPathing/Constants.java`

Current Pedro Pathing motor directions:
```java
.leftFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
.leftRearMotorDirection(DcMotorSimple.Direction.FORWARD)
.rightFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
.rightRearMotorDirection(DcMotorSimple.Direction.REVERSE)
```

If the robot **still** drives backward after this change, it's because Pedro Pathing uses its own motor directions during autonomous. In that case, those would need to be flipped as well.

## Testing

1. **Test TeleOp first** - Verify robot drives in expected direction
2. **Test Autonomous** - Run PedroAuto and verify path following
3. **If autonomous is still wrong** - Update Constants.java motor directions

## Status

✅ **Change applied successfully**  
✅ **No compilation errors**  
⚠️ **Needs testing to confirm correct direction**

