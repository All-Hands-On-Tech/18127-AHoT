# IMMEDIATE ACTION: Fix Your Oscillation

## Your Current Problem
- Overshoots to 1000
- Crashes to 700
- Oscillates between 650-750

## The Fix (Do This Now)

### In DepositPIDTuner, set EXACTLY these values:

```
Target Speed: 850 ticks/sec
KP: 0.00005
KI: 0.0
KD: 0.0003
KFF: 0.0003
```

### Then:
1. Press **D-Pad Right** to start motors
2. Watch telemetry for 10 seconds
3. Look for smooth ramp-up to 850 with minimal overshoot

---

## Expected Result After Fix

✅ **Good:** Smoothly reaches 850, minor overshoot (< 50 ticks), settles in 2-3 seconds

❌ **Still Bad:** Still overshooting past 900

---

## If Still Overshooting After First Try

Try these progressively:

### Round 2: More Aggressive Damping
```
KP: 0.00003
KD: 0.0005
KI: 0.0
KFF: 0.0003
```

### Round 3: Even More Conservative
```
KP: 0.00002
KD: 0.0007
KI: 0.0
KFF: 0.0003
```

### Round 4: Check Feedforward
```
KP: 0.00005
KD: 0.0003
KI: 0.0
KFF: 0.00025  (Reduced from 0.0003)
```

---

## Why This Fixes It

| Parameter | Current | New | Why |
|-----------|---------|-----|-----|
| KP | Too High | 0.00005 | Was causing massive overshoot response |
| KI | Active | 0.0 | Was making oscillations worse |
| KD | Too Low | 0.0003 | Now dampens the overshoot |
| KFF | Same | 0.0003 | Helps reach target without PID crashing |

---

## Once It's Stable

Once you get smooth behavior at 850:

1. Test at 300, 600, 1200 ticks/sec
2. If all good, press Y to print values
3. Copy values to PedroAutonomous.java
4. Test in autonomous mode

---

## Test Script (5 minutes)

1. Set values above
2. Press D-Pad Right
3. Count to 10 slowly
4. Is velocity smooth at 850? YES ✅ → Continue
5. Is velocity smooth at 850? NO ❌ → Try Round 2 values

Repeat until YES.

---

**Do this NOW and report back what velocity you get!**

