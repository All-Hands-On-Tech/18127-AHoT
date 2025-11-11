# 🎮 Deposit PID Tuning Guide - Complete Edition

---

## 📋 Table of Contents

1. [Tuner Setup](#tuner-setup)
2. [Tuner Controls](#tuner-controls)
3. [Quick Start](#quick-start)
4. [Your Current Problem](#your-current-problem)
5. [Detailed Tuning Steps](#detailed-tuning-steps)
6. [Testing & Validation](#testing--validation)
7. [Common Issues](#common-issues)
8. [Final Steps](#final-steps)

---

## 🎮 Tuner Setup

### What You Need
- **2 Wireless Gamepads** (controllers)
- **FTC Robot with Deposit Motors**
- **DepositPIDTuner OpMode** loaded
- **Driver Station**

### Setup Steps
1. Connect both gamepads to the Driver Station
2. Select **"Deposit PID Tuner"** from TeleOp list
3. Press **INIT**
4. Verify Panels telemetry is displaying
5. Both gamepads should vibrate/register (optional test)

**Status Check:** You should see the welcome message and Panels display with control mappings

---

## 🎮 Tuner Controls

### GAMEPAD 1: Motor Control
Used for **speed selection** and **starting/stopping motors**

| Button | Action | Details |
|--------|--------|---------|
| **D-Pad ↑** | Increase Target Speed | +10 ticks/sec per press |
| **D-Pad ↓** | Decrease Target Speed | -10 ticks/sec per press |
| **D-Pad ←** | **RESET EVERYTHING** | Speed→0, Stop motors, Reset PID |
| **D-Pad →** | **START MOTORS** | Begin test run |
| **A Button** | STOP (soft stop) | Stops motors WITHOUT resetting values |

### GAMEPAD 2: PID Tuning
Used for **real-time PID adjustment** while motors are running

| Button | Action | Details |
|--------|--------|---------|
| **D-Pad ↑** | Increase KP | By current increment size |
| **D-Pad ↓** | Decrease KP | By current increment size |
| **D-Pad ←** | Increase KI | By current increment size |
| **D-Pad →** | Decrease KI | By current increment size |
| **LT (Left Trigger)** | Increase KD | By current increment size |
| **RT (Right Trigger)** | Decrease KD | By current increment size |
| **X + D-Pad ↑/↓** | Adjust KFF | Fine control for feedforward |
| **B Button** | Toggle Increment Size | Switch between COARSE & FINE |
| **Y Button** | Print Values | Display copy-paste format |

### Telemetry Display (Panels)

Shows in real-time:
- **Status**: 🟢 RUNNING or 🔴 STOPPED
- **Target Speed**: Your set speed (ticks/sec)
- **Motor L/R Velocity**: Actual motor speeds
- **Average Velocity**: Both motors combined
- **Error**: Difference from target (ticks & %)
- **All PID Values**: KP, KI, KD, KFF in real-time

---

## ⚡ Quick Start (2 minutes)

### Step-by-Step
1. **Start OpMode** with both controllers ready
2. **Gamepad 1**: Press D-Pad ↑ several times (set speed to ~300)
3. **Gamepad 1**: Press D-Pad → (START MOTORS)
4. **Wait 2 seconds** - Observe telemetry
5. **Gamepad 2**: Press D-Pad ↓ (decrease KP if overshooting)
6. **Gamepad 1**: Press A (STOP)
7. **Gamepad 2**: Press Y (Print values)

**That's it!** You're now tuning the PID live.

---

## 🔴 Your Current Problem

### Symptoms
- **Overshoots to ~1000**
- **Crashes down to ~700**
- **Oscillates between 650-750**

### Root Cause Analysis
| Problem | Cause |
|---------|-------|
| Overshoot to 1000 | **KP is WAY too high** |
| Crash to 700 | **KD is too low** (no damping) |
| Oscillation 650-750 | **KI accumulating error** |

### Immediate Fix (Do This NOW)

**Set these exact values:**
```
Target Speed: 850 ticks/sec
KP: 0.00005       ← DRASTICALLY reduced
KI: 0.0           ← TURNED OFF
KD: 0.0003        ← SIGNIFICANTLY increased
KFF: 0.0003       ← Keep reasonable
```

**How to Set These:**
1. Start OpMode
2. Gamepad 1: D-Pad ← to RESET
3. Gamepad 1: D-Pad ↑ press 85 times (or set to 850 ticks)
4. Gamepad 2: Use D-Pad to adjust KP, KI, KD to above values
5. Gamepad 1: D-Pad → to START
6. **Watch for 10 seconds** - Should smooth out nicely

**Expected Result:**
- ✅ Smooth ramp to 850 ticks/sec
- ✅ No crazy overshoot
- ✅ Settles in 2-3 seconds
- ✅ No oscillation

---

## 📚 Detailed Tuning Steps

### STEP 1: Diagnose Motor Direction (5 min)

**Goal:** Verify motors spin FORWARD correctly

**Settings:**
```
Target Speed: 100 ticks/sec
KP: 0.00001
KI: 0.0
KD: 0.0
KFF: 0.0001
```

**Test:**
1. Set values above using Gamepad 2
2. Gamepad 1: Set speed to 100
3. Gamepad 1: Press D-Pad → (START)
4. Watch telemetry

**Result Check:**
- ✅ Motors spin FORWARD
- ✅ Reach ~100 ticks/sec smoothly
- ✅ No oscillation

**If motors spin backward:**
- Check hardware config for motor polarity
- Or check the `runDepositAtSpeed()` method for sign inversion

---

### STEP 2: Find Feedforward Baseline (10 min)

**Goal:** Establish how much power is needed to reach target

**Settings:**
```
Target Speed: 850 ticks/sec
KP: 0.0
KI: 0.0
KD: 0.0
KFF: 0.0002  ← START HERE
```

**Test Sequence:**
1. Set all values above
2. Gamepad 1: D-Pad ← (RESET)
3. Gamepad 1: Press D-Pad ↑ until speed = 850
4. Gamepad 1: D-Pad → (START)
5. Let run for 5 seconds
6. Record: What velocity do you reach?

**Adjust KFF based on results:**

| Velocity Reached | Action | New KFF |
|------------------|--------|---------|
| < 700 | Too low | Increase by 0.00005 |
| 700-800 | Getting close | Increase by 0.00002 |
| 800-850 | Good! | Keep it |
| 851-900 | Too high | Decrease by 0.00002 |
| > 900 | Way too high | Decrease by 0.00005 |

**Goal:** Reach 800-850 smoothly without oscillation

**Write down your final KFF value** ← This is your baseline!

---

### STEP 3: Add Proportional Control (15 min)

**Goal:** Make motor respond intelligently to error

**Starting Point:**
```
Target Speed: 850
KP: 0.00002  ← VERY LOW to start
KI: 0.0      ← OFF
KD: 0.0      ← OFF
KFF: [YOUR VALUE FROM STEP 2]
```

**Test & Adjust:**

| Iteration | KP Value | What to Watch | Next Action |
|-----------|----------|---------------|-------------|
| 1 | 0.00002 | Reaches target? | Increase KP |
| 2 | 0.00005 | Faster? Any overshoot? | Increase if OK |
| 3 | 0.0001 | Still smooth? | Increase if OK |
| 4 | 0.00015 | Slight overshoot? | Stop or add KD |
| 5 | 0.0002 | More overshoot? | Move to STEP 4 |

**When to Stop:**
- ✅ Reaches 850 in 1-2 seconds
- ✅ Overshoot < 50 ticks (under 900)
- ✅ No wild oscillation

**Write down your final KP value**

---

### STEP 4: Add Derivative for Smoothing (10 min)

**Goal:** Dampen overshoot and smooth the response

**Starting Point:**
```
Target Speed: 850
KP: [YOUR VALUE FROM STEP 3]
KI: 0.0
KD: 0.00002  ← VERY LOW to start
KFF: [YOUR VALUE FROM STEP 2]
```

**Test & Adjust:**

| KD Value | Expected Behavior | Notes |
|----------|-------------------|-------|
| 0.00002 | Minor overshoot reduction | Increase more |
| 0.00005 | Moderate damping | Keep going |
| 0.0001 | Good smoothing | Often the sweet spot |
| 0.00015 | More damping | If still overshooting |
| 0.0002 | Heavy damping | Might be sluggish now |

**When to Stop:**
- ✅ Overshoot is minimal (< 3%, not exceeding ~877)
- ✅ Still reaches target in < 2 seconds
- ✅ Smooth response without bouncing

**Write down your final KD value**

---

### STEP 5: Add Integral for Accuracy (10 min)

**ONLY DO THIS IF:**
- Motor settles BELOW target (e.g., 840 instead of 850)
- You need perfect accuracy

**Starting Point:**
```
Target Speed: 850
KP: [YOUR VALUE]
KI: 0.000002  ← VERY TINY to start
KD: [YOUR VALUE]
KFF: [YOUR VALUE]
```

**Test & Adjust:**

| KI Value | Expected Behavior | Notes |
|----------|-------------------|-------|
| 0.000002 | Tiny improvement | Might need more |
| 0.000005 | Noticeable creep toward target | Good progress |
| 0.00001 | Should reach exactly 850 | Often the target |
| 0.000015 | Risk of new oscillation | Watch for this |
| 0.00002 | Too aggressive | Causes bouncing |

**When to Stop:**
- ✅ Motor reaches AND maintains 850 ± 5 ticks/sec
- ✅ No new oscillation introduced
- ✅ Settles in < 2 seconds

**Write down your final KI value**

---

## 🧪 Testing & Validation

### Test Matrix (15 min)

Once you have KP, KI, KD, KFF tuned, test at multiple speeds:

**Test 1: Low Speed (300 ticks/sec)**
```
1. Gamepad 1: Set target to 300
2. Gamepad 1: START
3. Watch for smooth response, no overshoot
4. Record: Actual velocity reached?
```

**Test 2: Medium Speed (600 ticks/sec)**
```
Same as Test 1, but with 600 target
```

**Test 3: High Speed (850 ticks/sec)**
```
Same as Test 1, but with 850 target (your tuned speed)
```

**Test 4: Very High Speed (1200 ticks/sec)**
```
Same as Test 1, but with 1200 target
```

**Passing Criteria:**
All 4 tests should show:
- ✅ Reaches target smoothly
- ✅ Overshoot < 5%
- ✅ Settles within 2 seconds
- ✅ No oscillation

**If any test fails:**
- Go back to appropriate STEP
- Make small adjustments
- Re-test all 4 speeds

---

### Direction Consistency Check (5 min)

**Absolute Critical Test:**

```
1. Set target to 850
2. START motors
3. Watch for 10 full seconds
4. Record:
   - Does velocity ever go NEGATIVE?
   - Does it ever REVERSE direction?
   - Any weird jumps or glitches?
```

**MUST be TRUE:**
- ✅ Velocity stays positive throughout
- ✅ Never goes backward
- ✅ Smooth monotonic approach

**If velocity ever goes negative:**
- ❌ Motor polarity problem
- Check `runDepositAtSpeed()` for sign issue
- Fix before proceeding

---

## ⚠️ Common Issues

### Issue 1: Still Overshooting (> 900)

**Cause:** KP still too high OR KD too low

**Fix (in order):**
1. Reduce KP by 30% (e.g., 0.0001 → 0.00007)
2. Increase KD by 50% (e.g., 0.0001 → 0.00015)
3. Test again

### Issue 2: Won't Reach Target (stuck at 700)

**Cause:** KFF too low OR KP too low

**Fix (in order):**
1. Increase KFF by 0.00005
2. Or increase KP by 30%
3. Test again

### Issue 3: Oscillates Around Target (bounces ±100)

**Cause:** KI too aggressive OR KP/KD imbalanced

**Fix (in order):**
1. Reduce KI by 50% or turn it OFF (set to 0)
2. Increase KD by 50%
3. Test again

### Issue 4: Left/Right Motors Different

**Cause:** Motor hardware mismatch or encoder issue

**Fix:**
1. Check motor connections
2. Verify encoders are reading
3. May need individual motor tuning

---

## 🏁 Final Steps

### Step 1: Print Your Final Values

1. **Gamepad 2: Press Y button**
2. **Look for output like:**
   ```
   Copy these values into PedroAutonomous.java:
   private static final double DEPOSIT_SPEED_TARGET = 850.0;
   private static final double DEPOSIT_KP = 0.00005;
   private static final double DEPOSIT_KI = 0.000003;
   private static final double DEPOSIT_KD = 0.0001;
   private static final double DEPOSIT_KFF = 0.0003;
   ```

### Step 2: Update PedroAutonomous.java

1. **Open** `PedroAutonomous.java`
2. **Find** the TUNING VARIABLES section (around line 35)
3. **Replace** these lines:
   ```java
   private static final double DEPOSIT_SPEED_TARGET = 850.0;
   private static final double DEPOSIT_KP = [YOUR_VALUE];
   private static final double DEPOSIT_KI = [YOUR_VALUE];
   private static final double DEPOSIT_KD = [YOUR_VALUE];
   private static final double DEPOSIT_KFF = [YOUR_VALUE];
   ```

### Step 3: Rebuild and Test in Autonomous

1. **Build project** (Android Studio)
2. **Deploy** to robot
3. **Run autonomous**
4. **Verify** shooting is consistent and accurate

---

## ✅ Completion Checklist

Before you're done, verify:

- [ ] Motor spins forward (never backward)
- [ ] Reaches 850 ± 10 ticks/sec
- [ ] Overshoot < 5% (< 893)
- [ ] Settles in < 2 seconds
- [ ] No oscillation
- [ ] Works at 300, 600, 850, 1200 ticks/sec
- [ ] Left and right motors have similar velocity
- [ ] Values printed and copied to code
- [ ] Autonomous code rebuilt
- [ ] Tested in autonomous mode

---

## 💡 Pro Tips

1. **Test Multiple Times** - Run each speed 3+ times for consistency
2. **Battery Level** - Tune with fresh batteries; old batteries behave differently
3. **Small Changes** - Don't jump 0.00005 to 0.0002; go gradually
4. **Temperature** - Motors heat up; performance may change mid-session
5. **Keep Notes** - Write down what you tried and results
6. **Increment Modes** - Use COARSE for rough tuning, FINE for precise
7. **Press A, Not Reset** - Use A button between tests if only changing one param
8. **Watch Telemetry** - Don't guess; measure actual values

---

## 🎯 Expected Timeline

| Step | Time | Cumulative |
|------|------|-----------|
| Diagnose Direction | 5 min | 5 min |
| Feedforward Baseline | 10 min | 15 min |
| Proportional Control | 15 min | 30 min |
| Derivative Smoothing | 10 min | 40 min |
| Integral Accuracy | 10 min | 50 min |
| Multi-Speed Testing | 15 min | 65 min |
| Final Validation | 5 min | 70 min |
| Update Code | 5 min | **75 min total** |

**Expected Total Time: ~1 hour 15 minutes**

---

## 📞 Need Help?

If you're stuck:

1. **Check the Quick Reference** table below
2. **Verify motor connections** aren't loose
3. **Check battery voltage** (might be too low)
4. **Verify encoder readings** in telemetry
5. **Try resetting PID** (D-Pad Left) and starting over

---

## Quick Reference Table

| Symptom | Likely Cause | Solution |
|---------|-------------|----------|
| Overshoot to 1000, crash to 700, oscillate 650-750 | **KP way too high** | Reduce KP to 0.00005, increase KD to 0.0003, turn off KI |
| Motors spin backward | Motor polarity wrong | Check hardware config or code signs |
| Won't reach 850 | KFF or KP too low | Increase KFF by 0.00005 or KP by 30% |
| Bounces around target | KI too high or KP/KD imbalanced | Reduce KI by 50% or turn off, increase KD |
| Takes forever to respond | All gains too low | Increase KP and KFF |
| Jerky, not smooth | Missing derivative | Increase KD significantly |
| Left/right motors different | Hardware issue | Check connections and encoders |

---

**Good luck tuning! You've got this! 🚀**


