# Odometry Test - Quick Start

## What You Need
- Robot with Pinpoint odometry configured
- Driver Station connected
- Robot placed on field or flat surface

## Step-by-Step

### 1. Select OpMode
1. Open Driver Station
2. Select **"Odometry Test"** from Testing group
3. Press **INIT**

### 2. Check Initialization
Look for these messages:
- ✅ `✓ Pinpoint device initialized` = Perfect!
- ⚠️ `Warning: Could not access Pinpoint directly` = OK, follower still works
- ❌ `INITIALIZATION FAILED` = Fix needed (see guide)

### 3. Press START

### 4. Quick Tests

#### Test A: Does Heading Work?
1. **Look at telemetry**: Find "Heading (deg)" and "Heading (rad)"
2. **Rotate robot 90°** clockwise (turn right)
3. **Check**: Heading should increase by ~90° (1.57 radians)

**If heading is stuck between 0-1:** You found the bug! See troubleshooting.

#### Test B: Does Position Work?
1. **Push robot forward** about 12 inches
2. **Check**: "Y (inches)" should increase
3. **Push robot sideways** (strafe)
4. **Check**: "X (inches)" should change

**If position doesn't change:** Check encoder connections (press X)

#### Test C: Are Encoders Connected?
1. **Press X button** (toggle raw encoders)
2. **Look for**: "Forward Enc" and "Strafe Enc"
3. **Push robot**: Values should change

**If both show zero:** Encoders not connected!

## What The Display Means

### Good Odometry Looks Like:
```
=== POSITION ===
X (inches): 12.45
Y (inches): 67.89
Heading (deg): 45.0°      ← Should be 0-360
Heading (rad): 0.785      ← Should be 0-6.28

=== STATUS ===
Update Rate: 65.2 Hz      ← Should be 50+
✓ Heading Range: Normal (radians)
```

### Bad Odometry Looks Like:
```
Heading (deg): 0.5°       ← Stuck near 0-1
Heading (rad): 0.008      ← Should be bigger when rotated
⚠ Heading may be normalized 0-1!
```

## Quick Fixes

### "Encoders show zero"
1. Check cables are plugged into Pinpoint
2. Try pushing robot while watching encoder values
3. If still zero, cables may be damaged

### "Heading stuck 0-1"
1. Press **A** to reset position
2. Rotate robot and check again
3. If still stuck, see ODOMETRY_TEST_GUIDE.md

### "Position doesn't change"
1. Press **X** to show encoders
2. If encoders change but position doesn't:
   - Pod offsets may be wrong
   - Encoder directions may be reversed

## Controls Reference

| Button | What It Does |
|--------|-------------|
| **A** | Reset to (0, 0, 0°) |
| **B** | Set to field start (120.316, 128.707, 38°) |
| **X** | Show/hide raw encoder values |
| **Y** | Show/hide Pinpoint device status |
| **DPAD** | Adjust heading offset for testing |

## Next Steps

✅ **If everything looks good:**
1. Your odometry is working!
2. Run your autonomous (PedroAuto)
3. Watch telemetry during autonomous

❌ **If something is wrong:**
1. Read full guide: **ODOMETRY_TEST_GUIDE.md**
2. Check Constants.java configuration
3. Verify hardware connections

## Common Results

### ✅ WORKING
- Heading: 0-360° or 0-6.28 rad
- Update Rate: 50+ Hz
- Position changes when robot moves
- Encoders show non-zero values

### ❌ NOT WORKING
- Heading stuck at 0-1
- Encoders always zero
- Position never changes
- Update rate < 30 Hz
- Device Status: NOT_READY

---

**Full documentation:** ODOMETRY_TEST_GUIDE.md

