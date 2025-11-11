# 🎮 Tuner Quick Reference Card

## Controller Cheat Sheet

### GAMEPAD 1 - Motor Control (Left Hand)
```
╔═════════════════════╗
║  D-Pad Up/Down      ║  Set target speed (±10)
║  D-Pad Left  ←      ║  🛑 RESET ALL
║  D-Pad Right →      ║  ▶️  START motors
║  A Button           ║  ⏹️  STOP motors
╚═════════════════════╝
```

### GAMEPAD 2 - PID Tuning (Right Hand)
```
╔════════════════════════════╗
║  D-Pad Up/Down    ↑↓       ║  Adjust KP
║  D-Pad Left/Right ←→       ║  Adjust KI
║  LT (Left Trig)            ║  Increase KD
║  RT (Right Trig)           ║  Decrease KD
║  X + D-Pad                 ║  Adjust KFF
║  B Button                  ║  Toggle Increment
║  Y Button                  ║  Print values
╚════════════════════════════╝
```

---

## 1-Minute Startup

```
1. Start OpMode (2 controllers connected)
2. Gamepad 1: D-Pad Up × 85 (target = 850)
3. Gamepad 1: D-Pad Right (START)
4. Watch telemetry on Panels
5. Gamepad 1: A Button (STOP)
Done!
```

---

## Emergency Fix (Your Problem)

**Symptoms:** Overshoot to 1000, crash to 700, oscillate 650-750

**Fix in 30 seconds:**
```
Gamepad 2:
1. D-Pad Down × 65 times → KP = 0.00005
2. D-Pad Left × 100 times → KI = 0.0
3. LT × 10 times → KD = 0.0003

Then Gamepad 1:
4. D-Pad Up × 85 times → Speed = 850
5. D-Pad Right → START

Result: Smooth ramp to 850! ✅
```

---

## Tuning Checklist

- [ ] Motors spin FORWARD (not backward)
- [ ] Reaches 850 ticks/sec
- [ ] Overshoot < 5% (< 893)
- [ ] Settles in < 2 seconds
- [ ] No oscillation
- [ ] Works at 300, 600, 850, 1200 ticks/sec
- [ ] Left & Right motors match
- [ ] Values printed and copied to code
- [ ] Autonomous rebuilt & tested

---

## Status Light Guide

| Display | Meaning |
|---------|---------|
| 🟢 RUNNING | Motors are active |
| 🔴 STOPPED | Motors are off |
| COARSE | Large increments (0.0001) |
| FINE | Small increments (0.00001) |

---

## Common Issues Quick Fix

| Problem | Fix |
|---------|-----|
| Still overshooting | Gamepad 2: D-Pad Down more |
| Won't reach 850 | Gamepad 2: LT more (increase KD) |
| Oscillating | Gamepad 2: D-Pad Down + LT more |
| Backward spin | Check motor hardware config |
| Both motors different | Check encoder connections |

---

## Print Tuned Values

When done tuning:
```
Gamepad 2: Press Y Button

You'll see output like:
private static final double DEPOSIT_KP = 0.00005;
private static final double DEPOSIT_KI = 0.000003;
private static final double DEPOSIT_KD = 0.0001;
private static final double DEPOSIT_KFF = 0.0003;

Copy these into PedroAutonomous.java!
```

---

## Color Guide for Telemetry

```
🟢 Green/Ready  = Good status
🔴 Red/Stop     = Motors off
─── Lines       = Section headers
════╗ Boxes     = Control groups
```

---

## Key Values to Monitor

| Value | Target | Range |
|-------|--------|-------|
| Motor L/R Vel | ~850 | 840-860 ✅ |
| Error | < 10 | 0 ± 10 ✅ |
| Overshoot | < 50 | 0-50 ✅ |
| KP | 0.0001-0.0002 | Small values |
| KI | 0-0.00001 | Usually tiny |
| KD | 0.0001-0.0002 | Small-medium |
| KFF | 0.0003-0.0004 | Medium |

---

**Tip:** Keep this card next to you while tuning! Print it out! 🖨️

