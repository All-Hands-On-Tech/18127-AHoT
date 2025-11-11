# Deposit Tuner RPM - Diagnostic Logging Guide

## What Was Added

Your DepositTunerRPM now has **comprehensive diagnostic logging** to help identify the motor inconsistency problem!

---

## How to Use

### Step 1: Run the Tuner
1. Select **"Deposit Tuner - RPM"** from TeleOp
2. Press **START**

### Step 2: Set Your Target Speed
1. Use **GP2 D-Pad Up/Down** to adjust RPM (±10 RPM)
2. Use **GP2 D-Pad Left/Right** for fine adjustment (±1 RPM)
3. Set to your desired RPM (e.g., 400 RPM)

### Step 3: Start Motors
1. Press **GP2 X** to toggle motors ON
2. Wait for motors to stabilize (or see them oscillate)

### Step 4: Log Data Points
1. **Press GP2 Y button** repeatedly to capture snapshots
2. Press Y every ~0.5-1 second while motors are running
3. Capture at least **10-20 snapshots** showing the problem
4. Make sure to capture:
   - When motors first start (ramp-up)
   - When they're oscillating/inconsistent
   - When they're supposed to be stable

### Step 5: View the Log
1. **Hold GP2 B button** to display all logged data
2. The screen will show the last 20 logged entries
3. Each line contains:
   - **Timestamp** (seconds since start)
   - **Target RPM** and ticks/sec
   - **Motor L velocity** (left motor ticks/sec)
   - **Motor R velocity** (right motor ticks/sec)
   - **Average velocity**
   - **Error** (ticks and %)
   - **Imbalance** (difference between L and R)
   - **Power** (motor power applied)
   - **Loop frequency** (Hz)

### Step 6: Copy the Data
1. While holding **GP2 B**, take a **screenshot** or **photo** of the Driver Station screen
2. Or manually copy the log entries from the screen
3. Send me the logged data!

---

## Example Log Entry

```
[5] T=12.45s | Target: 400 RPM (179 ticks) | L: 250 | R: 110 | Avg: 180 | Err: -1 ticks (-0.6%) | Imbal: 140 | Power: 0.456 | Hz: 62.5
```

**What this tells us:**
- At 12.45 seconds into the run
- Target was 400 RPM (179 ticks/sec)
- Left motor: 250 ticks/sec
- Right motor: 110 ticks/sec ← **BIG PROBLEM!**
- Average: 180 ticks/sec (on target but motors unbalanced)
- Error: -1 tick (basically on target)
- Imbalance: 140 ticks difference between motors ← **HUGE IMBALANCE!**
- Power: 0.456 (45.6%)
- Loop running at 62.5 Hz

---

## What to Look For

### Good Signs ✓
- **Imbalance < 30 ticks** - Motors balanced
- **Error % < 5%** - On target
- **Motor L and R similar** - Both motors working together
- **Stable?** shows "✓ YES"

### Bad Signs ✗
- **Imbalance > 50 ticks** - Motors NOT synchronized
- **Error % > 10%** - Way off target
- **Motor L >> Motor R** or vice versa - One motor lagging
- **Oscillating!** marker shows up
- **Range > 100** - Wild oscillation

---

## Telemetry Display

While the tuner is running, you'll see:

```
╔═══════════════════════════════╗
║ DEPOSIT TUNER - DEBUG         ║
╚═══════════════════════════════╝

⚡ Status: 🟢 RUNNING

─── TARGET ───
  RPM: 400.0
  Ticks/sec: 179.0

─── ACTUAL VELOCITIES ───
  Motor L: 175.0 ticks/s (390 RPM)
  Motor R: 180.0 ticks/s (402 RPM)
  Average: 177.5 ticks/s (396 RPM)

─── ERROR ───
  Error: 1.5 ticks (3.4 RPM)
  Error %: 0.8%
  On Target? ✓ YES

─── MOTOR BALANCE ───
  L - R Diff: -5.0 ticks/s
  Balanced? ✓ YES

─── STABILITY ───
  Motor L Range: 25.0 ticks/s
  Motor R Range: 30.0 ticks/s
  Stable? ✓ YES

─── PID INTERNALS ───
  Power Output: 0.325
  KP: 0.00003000
  KI: 0.00000100
  KD: 0.00040000
  KFF: 0.00012

─── LOOP PERFORMANCE ───
  Frequency: 65.5 Hz
  Period: 15.3 ms

─── CONTROLS ───
  X: Toggle ON/OFF
  D-Pad ↑/↓: ±10 RPM
  D-Pad ←/→: ±1 RPM
  Y: Log current data
  B: Print all logged data

─── DIAGNOSTIC LOG ───
  Snapshots Logged: 15
  Last Log Time: 34.56s
```

---

## Quick Test Procedure

### 5-Minute Test
1. **Start**: Launch tuner, set to 400 RPM
2. **Activate**: Press X to start motors
3. **Log startup**: Press Y 5 times in first 2 seconds
4. **Log running**: Press Y 10 times over next 3-5 seconds
5. **View**: Hold B to see all logs
6. **Screenshot**: Capture the log dump
7. **Send to me**: Share the data!

---

## What Happens Next

Once you send me the logged data, I'll analyze:
1. **Motor imbalance** - Are both motors getting same power but producing different speeds?
2. **Oscillation pattern** - Is PID causing the bouncing?
3. **Power levels** - Is power too high/low?
4. **Error trends** - Is it converging or diverging?
5. **Hardware issues** - One motor slower/faster consistently?

Then I'll create the **exact code fix** needed!

---

## Tips

✅ **Log during the problem** - Capture when it's oscillating/inconsistent
✅ **Multiple snapshots** - More data = better diagnosis
✅ **Hold B to view** - Don't just tap it, hold it
✅ **Screenshot clearly** - Make sure text is readable
✅ **Test at different RPMs** - Try 200, 400, 600 RPM

---

**Ready to diagnose! Run the tuner and send me the logs!** 🔍

