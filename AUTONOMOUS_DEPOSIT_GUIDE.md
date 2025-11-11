# Autonomous Deposit Control - Quick Reference

## What It Does Now
✅ Uses **same PID velocity control** as DepositTuner  
✅ Runs deposit at **850 ticks/sec** (same as your tuned value)  
✅ Automatically maintains target velocity throughout autonomous  

## Tunable Values (lines 38-43)

```java
DEPOSIT_TARGET_VELOCITY = 850.0  // Target speed in ticks/sec
DEPOSIT_KP = 0.0008              // Proportional gain
DEPOSIT_KI = 0.0000015           // Integral gain  
DEPOSIT_KD = 0.00005             // Derivative gain
DEPOSIT_KFF = 0.00015            // Feedforward coefficient
```

## How It Works

1. **STATE_START_PATH1**: Resets PID, starts path following
2. **STATE_PATH_AND_DEPOSIT**: Runs deposit at 850 ticks/sec while driving
3. **STATE_WAIT_BEFORE_INTAKE**: Keeps deposit running, waits 2 seconds
4. **STATE_INTAKE**: Runs both deposit and intake for 1.5 seconds
5. **STATE_DONE**: Stops everything cleanly

## Telemetry to Watch

- **DepositL Vel**: Left motor actual velocity (should reach ~850)
- **DepositR Vel**: Right motor actual velocity (should reach ~850)
- **Path State**: Current state (0→1→2→3→-1)

## Tuning If Needed

### Speed too slow/fast?
Change `DEPOSIT_TARGET_VELOCITY` (line 38)

### Not reaching target speed?
1. Increase `DEPOSIT_KP` (faster response)
2. Increase `DEPOSIT_KFF` (more initial power)

### Oscillating/overshooting?
1. Decrease `DEPOSIT_KP`
2. Increase `DEPOSIT_KD` (damping)

### Steady-state error?
Increase `DEPOSIT_KI` (eliminates persistent error)

## Expected Behavior

- Motors ramp up smoothly to 850 ticks/sec
- Hold steady at target throughout autonomous
- No oscillation or jerky movement
- Stop cleanly when done

**This matches your DepositTuner behavior exactly!**

