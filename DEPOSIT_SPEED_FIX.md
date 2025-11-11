# Deposit Speed Fix - Summary

## What I Fixed

### 1. **Corrected Target Velocity Sign**
- Changed from `-820.0` to `+850.0` (matches DepositTuner exactly)
- Negative target was confusing the PID controller

### 2. **Matched DepositTuner Control Logic Exactly**
- Same clamping method (`if` statements instead of `Math.max/min`)
- Same power application (left motor gets `power`, right gets `-power`)
- Same feedforward calculation

### 3. **Enhanced Telemetry**
- Shows actual velocity vs target (e.g., "850/850")
- Shows motor power output (e.g., "@0.13")
- Path selection visible at all times

## What to Check When You Test

### Expected Behavior:
- **DepositL**: Should show `850/850 @0.12-0.15` (velocity at target, low power)
- **DepositR**: Should show `850 @-0.12` (same velocity, opposite power)
- Motors should ramp up smoothly over ~1 second
- Speed should hold steady at 850 ticks/s

### If Still Going Too Fast:
1. Check telemetry - what velocity does it actually show?
2. Is the velocity reading correct or is it showing 2000+?
3. Check power output - is it maxed at 1.0 or reasonable (0.1-0.2)?

## Most Likely Issues

### If motors spin at full speed:
- **Motor mode wrong** - Check that motors are in `RUN_USING_ENCODER` mode
- **Encoder not connected** - Velocity reads as 0, PID maxes out power

### If velocity reads correctly but ignores target:
- **PID not updating** - Check that `runDepositAtVelocity(dt)` is being called
- **dt too large** - Check loop timing

### If velocity oscillates wildly:
- **Gains too high** - Reduce kP
- **dt calculation wrong** - Motors getting huge time steps

## Quick Test
Run autonomous and watch telemetry:
- Does "DepositL" show a reasonable velocity number?
- Does it reach 850 or keep climbing?
- What does the power (@X.XX) settle to?

**Tell me what the telemetry shows and I can diagnose further!**

