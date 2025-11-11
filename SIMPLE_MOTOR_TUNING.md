# Simple Motor Tuning Guide

## What Changed
- Removed all PID/position control complexity
- Motors now run at constant power (simple and predictable)
- You control exact power for each motor independently

## Tunable Values (lines 38-39)
```java
DEPOSIT_POWER_L = -0.5  // Left motor (-1.0 to 1.0)
DEPOSIT_POWER_R = 0.5   // Right motor (-1.0 to 1.0)
```

## Quick Diagnosis Steps

### 1. Check Direction (5 seconds)
- Run autonomous
- **Watch telemetry**: DepositMotorL Pos & DepositMotorR Pos
- Both should increase in SAME direction
- **If opposite**: One motor is wired backwards

### 2. Fix Wrong Direction
- **Option A**: Flip the sign (0.5 → -0.5 or vice versa)
- **Option B**: Swap motor wires physically

### 3. Tune Speed
- **Too slow**: Increase absolute value (0.5 → 0.7)
- **Too fast**: Decrease absolute value (0.5 → 0.3)
- **Both motors must have SAME absolute value** (e.g., -0.6 and 0.6)

### 4. Common Issues
| Problem | Solution |
|---------|----------|
| Only one motor spins | Check wiring/config names |
| Motors fight each other | Wrong direction on one motor |
| Not enough power | Increase both values equally |
| Too jerky | Lower both values equally |

## Example Settings
```java
// Slow and steady
DEPOSIT_POWER_L = -0.3
DEPOSIT_POWER_R = 0.3

// Medium speed
DEPOSIT_POWER_L = -0.5
DEPOSIT_POWER_R = 0.5

// Full speed
DEPOSIT_POWER_L = -0.8
DEPOSIT_POWER_R = 0.8
```

## Testing Process
1. Set both to 0.3 (safe starting point)
2. Run for 5 seconds
3. Check telemetry - encoders should increase
4. Adjust power if needed
5. Test again

**Total tuning time: ~2 minutes**

