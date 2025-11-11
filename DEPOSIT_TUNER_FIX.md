# DepositTuner Fix Summary

## Issues Fixed

### Issue 1: PID Only Affecting One Motor
**Problem:** The PID controller was only reading velocity from `hw.depositMotorL` (left motor), so it was only controlling one motor properly.

**Solution:** Changed the velocity reading to average both motors:
```java
// Read actual velocity from BOTH motors (average them for PID feedback)
double actual = 0.0;
if (hw.depositMotorL != null && hw.depositMotorR != null) {
    double vL = hw.depositMotorL.getVelocity();
    double vR = hw.depositMotorR.getVelocity();
    actual = (vL + vR) / 2.0;  // Average both motors
} else if (hw.depositMotorL != null) {
    actual = hw.depositMotorL.getVelocity();
}
```

Now the PID uses the **average velocity** of both motors for feedback, ensuring both motors are controlled together.

---

### Issue 2: Oscillating Speeds
**Problem:** The PID gains were too aggressive, causing the motors to oscillate around the target speed.

**Old PID Values:**
```java
KP: 0.0002
KI: 0.00000050
KD: 0.0001
```

**New PID Values (Conservative):**
```java
KP: 0.00008    (reduced by 60% - less aggressive response)
KI: 0.000003   (increased 6x - helps reach target steadily)
KD: 0.0002     (doubled - more damping, smoother approach)
```

These values prioritize **smooth, non-oscillating** behavior over fast response time.

---

## What This Fixes

✅ **Both motors now controlled by PID** - Average velocity used for feedback
✅ **Reduced oscillation** - Lower KP prevents overshooting
✅ **Better damping** - Higher KD smooths the response
✅ **Steady-state accuracy** - Higher KI helps reach exact target

---

## Expected Behavior After Fix

**Before:**
- Only one motor responded to PID
- Motors oscillated/bounced around target speed
- Inconsistent shooting velocity

**After:**
- Both motors controlled together (averaged feedback)
- Smooth ramp-up to target speed
- Minimal oscillation
- More consistent shooting

---

## Testing

1. **Start DepositTuner**
2. **Press GP2 X** to activate motors
3. **Use GP2 D-Pad Up/Down** to set target velocity (try 180 for 400 RPM)
4. **Watch telemetry:**
   - Both L and R velocities should be similar
   - Should smoothly approach target without bouncing
   - Should hold steady at target

---

## If Still Oscillating

If you still see oscillation after this fix, try these adjustments:

1. **Reduce KP further:**
   ```java
   PIDController depositPid = new PIDController(0.00005, 0.000003, 0.0002);
   ```

2. **Increase KD more:**
   ```java
   PIDController depositPid = new PIDController(0.00008, 0.000003, 0.0003);
   ```

3. **Check motor directions** - If motors are fighting each other, one might need direction reversed

---

## Files Modified

- `/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/TeleOp/DepositTuner.java`
  - Fixed velocity reading to average both motors
  - Updated PID gains to prevent oscillation

---

**Changes are ready to test!** 🎯

