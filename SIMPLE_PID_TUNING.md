# Simple PID Tuning Guide

## Your Setup
- **Target Speed:** 180 ticks/sec (≈ 400 RPM)
- **Time to Reach:** ~6 seconds (NO RUSH)
- **Priority:** NO OVERSHOOT (smooth and safe)

---

## Conservative PID Values (Anti-Overshoot)

```java
private static final double DEPOSIT_SPEED_TARGET = 180.0;    // 400 RPM
private static final double DEPOSIT_KP = 0.00008;            // Very low - no aggressive response
private static final double DEPOSIT_KI = 0.000003;           // Tiny - for fine adjustments
private static final double DEPOSIT_KD = 0.0002;             // High - smooth damping
private static final double DEPOSIT_KFF = 0.00018;           // Low feedforward
```

**What these do:**
- **KP (0.00008):** Very gentle response, prevents overshoot
- **KI (0.000003):** Slowly creeps to target if needed
- **KD (0.0002):** Smooth damping, prevents any bouncing
- **KFF (0.00018):** Just enough power to help it reach 180

---

## How to Tune (3 Steps)

### Step 1: Test Current Values
1. Start DepositPIDTuner
2. Gamepad 1: D-Pad Up ~18 times (set to 180)
3. Gamepad 1: D-Pad Right (START)
4. Watch for 10 seconds
5. **Does it smoothly reach 180 without overshooting?**

**If YES:** ✅ Done! Copy values to PedroAutonomous
**If NO:** Go to Step 2

---

### Step 2: If Still Overshooting
- Gamepad 2: D-Pad Down 5 times (reduce KP more)
- Test again
- Keep reducing KP until smooth

---

### Step 3: If Not Reaching 180
- Gamepad 2: LT 2-3 times (increase KD for slower ramp)
- Or Gamepad 2: D-Pad Left 1-2 times (increase KI slightly)
- Test again

---

## Copy These Values to PedroAutonomous.java

When done, replace this section in **PedroAutonomous.java** (around line 35):

```java
private static final double DEPOSIT_SPEED_TARGET = 180.0;
private static final double DEPOSIT_KP = 0.00008;
private static final double DEPOSIT_KI = 0.000003;
private static final double DEPOSIT_KD = 0.0002;
private static final double DEPOSIT_KFF = 0.00018;
```

---

## Quick Reference

| Problem | Fix |
|---------|-----|
| Overshooting | Reduce KP (Gamepad 2: D-Pad Down) |
| Too slow | Increase KD (Gamepad 2: LT) |
| Bouncing | Increase KD more (Gamepad 2: LT) |
| Won't reach 180 | Increase KI (Gamepad 2: D-Pad Left) |

---

**That's it! Simple and smooth.** 🎯

