# PID Tuning Guide for Consistent Shooting

This guide explains how to tune the PID controller for your deposit (shooting) mechanism to achieve consistent velocity control and reduce overshooting.

## Understanding PID Control

The PID controller has three main components:

- **P (Proportional)**: Responds to current error. Higher values = faster response but more overshoot.
- **I (Integral)**: Eliminates steady-state error over time. Helps reach the exact target velocity.
- **D (Derivative)**: Dampens the response. Reduces overshoot and oscillation.
- **FF (Feedforward)**: Provides an initial power boost to help reach target velocity faster.

## Current Configuration

```java
private static final double DEPOSIT_SPEED_TARGET = 850.0; // encoder ticks per second
private static final double DEPOSIT_KP = 0.0002; // Proportional gain
private static final double DEPOSIT_KI = 0.00001; // Integral gain
private static final double DEPOSIT_KD = 0.0001; // Derivative gain
private static final double DEPOSIT_KFF = 0.00035; // Feedforward
```

## Common Issues and Solutions

### Issue 1: Motor is Overshooting (Speeding Up Too Much)

**Symptoms:**
- Motor velocity spikes above the target
- Shooting power is inconsistent and too strong
- Motor velocity oscillates wildly

**Solutions (in order of effectiveness):**

1. **Reduce KP (Proportional Gain)** ← Best first step
   - Start with current value and divide by 2
   - If `DEPOSIT_KP = 0.0004`, try `0.0002`
   - This reduces the aggressive response to error
   - Trade-off: Motor may take longer to reach target speed

2. **Increase KD (Derivative Gain)**
   - Adds damping to smooth out the response
   - Try multiplying current value by 1.5-2x
   - If `DEPOSIT_KD = 0.00005`, try `0.0001`
   - This helps prevent overshooting

3. **Decrease KFF (Feedforward)**
   - Reduces the initial power boost
   - If `DEPOSIT_KFF = 0.0005`, try `0.00035`
   - Only if reducing KP doesn't help enough

### Issue 2: Motor Not Reaching Target Speed

**Symptoms:**
- Motor speed stays below target (e.g., reaching 750 instead of 850)
- Inconsistent shooting power
- Takes too long to spin up

**Solutions:**

1. **Increase KFF (Feedforward)**
   - Provides more initial push to reach target
   - If `DEPOSIT_KFF = 0.00025`, try `0.0003`
   - This is the quickest way to reach target speed

2. **Increase KI (Integral Gain)**
   - Slowly reduces steady-state error
   - If `DEPOSIT_KI = 0.000005`, try `0.00001`
   - Takes longer to work but fixes persistent errors

3. **Increase KP (Proportional Gain)**
   - More responsive to error
   - If `DEPOSIT_KP = 0.0002`, try `0.00025`
   - Be careful not to introduce overshoot

### Issue 3: Motor Oscillating Around Target

**Symptoms:**
- Motor velocity bounces above and below target
- Shooting power is jerky and inconsistent

**Solutions:**

1. **Increase KD (Derivative Gain)**
   - Adds damping to smooth oscillations
   - If `DEPOSIT_KD = 0.00005`, try `0.0001` or higher

2. **Reduce KP (Proportional Gain)**
   - Lower responsiveness reduces oscillation
   - If `DEPOSIT_KP = 0.0003`, try `0.0002`

## Tuning Process

### Step 1: Gather Baseline Data
Before tuning, run your autonomous for several times and note:
- What velocity do you actually reach? (Check telemetry)
- Is it overshooting or undershooting?
- Does it oscillate?

### Step 2: Make One Change at a Time
- Change only ONE parameter value per test
- Test multiple times (at least 3) to verify consistency
- Log the results in telemetry
- If it gets worse, revert and try a different parameter

### Step 3: Fine-Tune Each Parameter

**For Overshoot:**
```
KP: Divide by 2-3 first, then fine-tune
KD: Multiply by 1.5, then add more if needed
KFF: Only reduce if overshooting severely
```

**For Undershoot:**
```
KFF: Multiply by 1.2-1.5 first
KI: Increase slightly if persistent error
KP: Increase only if KFF doesn't help
```

**For Oscillation:**
```
KD: Increase significantly (2-3x)
KP: Reduce by 20-30%
```

### Step 4: Validate Performance
Once tuned, verify:
- Motor reaches 850 ± 10 ticks/sec consistently
- No overshoot (velocity shouldn't go above 900)
- Minimal oscillation (smooth, steady velocity)
- Consistent shooting accuracy over multiple shots

## Recommended Tuning Values to Try

If current values aren't working, here are progressive starting points:

### Aggressive (Fast Response, More Overshoot):
```java
DEPOSIT_KP = 0.0004
DEPOSIT_KI = 0.00002
DEPOSIT_KD = 0.00005
DEPOSIT_KFF = 0.0004
```

### Conservative (Smooth, No Overshoot):
```java
DEPOSIT_KP = 0.00015
DEPOSIT_KI = 0.000008
DEPOSIT_KD = 0.00015
DEPOSIT_KFF = 0.0003
```

### Balanced (Recommended Starting Point):
```java
DEPOSIT_KP = 0.0002
DEPOSIT_KI = 0.00001
DEPOSIT_KD = 0.0001
DEPOSIT_KFF = 0.00035
```

## Telemetry Monitoring

Always check these values in telemetry during tuning:
- **DepositMotorL Vel**: Left motor velocity (should match target)
- **DepositMotorR Vel**: Right motor velocity (should match left motor)
- **DEPOSIT_SPEED_TARGET**: Your target velocity (850 in this case)

If left and right velocities differ significantly, you may have a motor power imbalance.

## Motor Direction Fix

Both motors should have the same power applied:
```java
depositMotorL.setPower(totalPower);
depositMotorR.setPower(totalPower);
```

If motors spin in opposite directions, invert one in the hardware configuration or add a negative sign.

## Tips for Consistency

1. **Run multiple tests**: Velocity can vary based on battery voltage
2. **Test at similar battery levels**: Fresh batteries may behave differently
3. **Monitor temperature**: Motors heat up after repeated shots
4. **Use telemetry**: Don't rely on feel; measure actual velocity
5. **Log results**: Keep track of which values work best
6. **Start conservative**: Begin with lower gains and increase gradually

## When to Stop Tuning

Your tuning is complete when:
- ✓ Motor consistently reaches target velocity within ±10 ticks/sec
- ✓ No overshoot (velocity doesn't exceed target by more than 5%)
- ✓ Minimal oscillation (smooth, steady velocity)
- ✓ Consistent shooting accuracy across multiple cycles
- ✓ Performance remains stable across multiple autonomous runs

Good luck with your shooting tuning!

