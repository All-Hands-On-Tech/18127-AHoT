package org.firstinspires.ftc.teamcode.Trowel.Unused;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import org.firstinspires.ftc.teamcode.Trowel.Configs.TrowelHardware;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.PrintWriter;
import java.util.Random;

/**
 * DepoTuner v4.2 - POWER OPTIMIZED + FULL FEATURES
 *
 * All features from v4.0 retained, but with power optimizations:
 *   - Throttled computations (every 3rd loop)
 *   - Lazy sensor reads
 *   - Motor command caching
 *   - Reduced telemetry updates
 *   - Primitive arrays instead of ArrayLists
 *
 * FEATURES:
 *   - All 20 parameters auto-tuned
 *   - Kalman filtered velocity (simplified)
 *   - Voltage compensation
 *   - Predictive + reactive boost
 *   - Oscillation detection + auto-damping
 *   - Elite preservation (top 3)
 *   - Rollback protection
 *   - Outlier rejection
 *   - Confidence scoring
 *   - Dual motor tracking
 *   - Full persistence
 */
@TeleOp(name = "Do not use this", group = "Tuning")
public class DepoTunerV extends OpMode {

    // ═══════════════════════════════════════════════════════════════
    //  POWER CONFIG
    // ═══════════════════════════════════════════════════════════════

    private static final int LOOP_THROTTLE = 3;           // Run heavy code every 3rd loop
    private static final double MOTOR_THRESHOLD = 1.0;    // Min change to write motor
    private static final long VOLTAGE_INTERVAL = 400;     // Read voltage every 400ms
    private static final long TELEMETRY_INTERVAL = 200;   // Update display every 200ms
    private static final int MAX_SAMPLES = 200;           // Max samples per session
    private static final int ELITE_COUNT = 3;             // Keep top 3 elites
    private static final int LOSS_HISTORY = 15;           // Track last 15 losses

    // ═══════════════════════════════════════════════════════════════
    //  PERSISTENCE
    // ═══════════════════════════════════════════════════════════════

    private static double[] staticBestParams = null;
    private static double[] staticRollbackParams = null;
    private static double staticBestLoss = Double.MAX_VALUE;
    private static int staticSessions = 0;
    private static int staticImprovements = 0;
    private static boolean staticInit = false;

    private static final String SAVE_FILE = "/sdcard/FIRST/depotuner.txt";

    // ═══════════════════════════════════════════════════════════════
    //  TUNABLE PARAMETERS (20)
    // ═══════════════════════════════════════════════════════════════

    private double ball1Mult = 1.0, ball1Exp = 1.5;
    private double ball2Mult = 1.6, ball2Exp = 1.6;
    private double ball3Mult = 1.4, ball3Exp = 1.55;

    private double boostTrigger = 45.0, boostCap = 320.0;
    private double rampUpRate = 0.4, approachDamping = 0.6;
    private double derivativeGain = 0.06, decayRate = 0.015;
    private double hysteresisRatio = 0.7;

    private double predBoostFrac = 0.35;
    private double predWindowMs = 100.0;
    private double predRampUp = 0.5;

    private double integralGain = 0.002, integralCap = 30.0;
    private double motor1Bias = 0.0, motor2Bias = 0.0;

    private static final int PARAM_COUNT = 20;

    // ═══════════════════════════════════════════════════════════════
    //  FIXED CONFIG
    // ═══════════════════════════════════════════════════════════════

    private static final double REF_VEL = 640.0;
    private static final double NOM_VOLTAGE = 12.5;

    private static final double INIT_TEMP = 1.0;
    private static final double COOL_RATE = 0.992;
    private static final double MIN_TEMP = 0.04;
    private static final double RESTART_TEMP = 0.45;
    private static final int RESTART_THRESH = 12;
    private static final int ROLLBACK_THRESH = 5;
    private static final double ROLLBACK_RATIO = 1.5;
    private static final double OUTLIER_MULT = 3.0;
    private static final double OSC_THRESHOLD = 0.15;
    private static final double OSC_DAMPING = 0.85;

    private static final int MIN_SAMPLES = 25;
    private static final int MIN_BALLS = 2;
    private static final int WARMUP = 12;
    private static final double DROP_DETECT = 60.0;
    private static final int BALL_GAP = 30;

    private double targetVel = 640.0;

    private static final double SERVO_SHOOT = 0.7;
    private static final double SERVO_IDLE = 0.3;

    // ═══════════════════════════════════════════════════════════════
    //  STATE
    // ═══════════════════════════════════════════════════════════════

    private TrowelHardware robot;
    private VoltageSensor voltSensor;
    private Random rand;

    // Throttling
    private int loopCount = 0;
    private long lastVoltTime = 0;
    private long lastTeleTime = 0;

    // Kalman (simplified 2-state)
    private double kVel = 0, kAccel = 0;
    private double kP = 1;
    private long lastKTime = 0;

    // Motor tracking
    private double m1Vel = 0, m2Vel = 0;
    private double m1Filt = 0, m2Filt = 0;
    private double lastM1Cmd = 0, lastM2Cmd = 0;

    // Voltage
    private double voltage = NOM_VOLTAGE;
    private double voltScale = 1.0;

    // Boost
    private double curBoost = 0;
    private double integral = 0;
    private long boostStart = 0;
    private boolean inBoost = false;
    private boolean exiting = false;

    // Predictive
    private long servoTime = 0;
    private boolean predActive = false;
    private double predBoost = 0;

    // Oscillation
    private double[] oscHistory = new double[16];
    private int oscIdx = 0;
    private double oscScore = 0;
    private double oscDamp = 1.0;

    // Recording
    private boolean recording = false;
    private long[] sTimes = new long[MAX_SAMPLES];
    private double[] sVels = new double[MAX_SAMPLES];
    private double[] sM1 = new double[MAX_SAMPLES];
    private double[] sM2 = new double[MAX_SAMPLES];
    private double[] sBoost = new double[MAX_SAMPLES];
    private int sCount = 0;
    private int balls = 0;
    private int lastBallIdx = 0;

    // Optimizer
    private double[] bestP = new double[PARAM_COUNT];
    private double[] curP = new double[PARAM_COUNT];
    private double[] trialP = new double[PARAM_COUNT];
    private double[] rollbackP = new double[PARAM_COUNT];
    private double bestLoss = Double.MAX_VALUE;
    private double rollbackLoss = Double.MAX_VALUE;
    private double temp = INIT_TEMP;
    private int noImprove = 0;
    private int degradeCount = 0;

    // Elites
    private double[][] eliteP = new double[ELITE_COUNT][PARAM_COUNT];
    private double[] eliteL = new double[ELITE_COUNT];
    private int eliteN = 0;

    // Confidence
    private double[] lossHist = new double[LOSS_HISTORY];
    private int lossIdx = 0;
    private int lossCount = 0;
    private double confidence = 0;

    // Stats
    private int sessions = 0;
    private int improvements = 0;
    private int outliers = 0;
    private int rollbacks = 0;
    private long startTime = 0;
    private double lastLoss = 0;
    private int lastBalls = 0;
    private boolean lastOutlier = false;

    // State
    private boolean depoOn = false;
    private boolean exportMode = false;
    private boolean javaFmt = false;

    // Buttons
    private int lastBtn = 0;

    // ═══════════════════════════════════════════════════════════════
    //  INIT
    // ═══════════════════════════════════════════════════════════════

    @Override
    public void init() {
        robot = new TrowelHardware(hardwareMap);
        robot.resetDepositEncoders();
        rand = new Random();

        configMotors();

        try {
            voltSensor = hardwareMap.voltageSensor.iterator().next();
        } catch (Exception e) {
            voltSensor = null;
        }

        // Init params
        syncFromFields();
        copy(curP, bestP);
        copy(curP, rollbackP);

        // Init elites
        for (int i = 0; i < ELITE_COUNT; i++) eliteL[i] = Double.MAX_VALUE;

        // Load saved
        loadFile();
        if (!staticInit && staticBestParams != null) loadStatic();

        if (robot.transferServo != null) robot.transferServo.setPosition(SERVO_IDLE);

        startTime = System.currentTimeMillis();

        telemetry.addLine("═══════════════════════════════════");
        telemetry.addLine(" DepoTuner v4.2 - FULL + POWER SAVE");
        telemetry.addLine("═══════════════════════════════════");
        telemetry.addLine("");
        telemetry.addLine("ALL FEATURES RETAINED:");
        telemetry.addLine("  ✓ 20 auto-tuned parameters");
        telemetry.addLine("  ✓ Kalman velocity filter");
        telemetry.addLine("  ✓ Voltage compensation");
        telemetry.addLine("  ✓ Oscillation detection");
        telemetry.addLine("  ✓ Rollback protection");
        telemetry.addLine("  ✓ Outlier rejection");
        telemetry.addLine("  ✓ Confidence scoring");
        telemetry.addLine("  ✓ Dual motor tracking");
        telemetry.addLine("");
        telemetry.addLine("POWER OPTIMIZATIONS:");
        telemetry.addLine("  • Throttled loop (3x)");
        telemetry.addLine("  • Lazy sensor reads");
        telemetry.addLine("  • Cached motor writes");
        telemetry.addLine("");
        if (sessions > 0) {
            telemetry.addData("Loaded", "%d sessions, %.1f loss", sessions, bestLoss);
            telemetry.addData("Confidence", "%.0f%%", confidence * 100);
        }
        telemetry.addLine("");
        telemetry.addLine("X=Deposit  LT=Shoot  Y+B=Export");
        telemetry.update();
    }

    private void configMotors() {
        if (robot.frontLeft != null) robot.frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        if (robot.frontRight != null) robot.frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        if (robot.backLeft != null) robot.backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        if (robot.backRight != null) robot.backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        try {
            if (robot.deposit1 != null) robot.deposit1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            if (robot.deposit2 != null) robot.deposit2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        } catch (Exception ignored) {}
    }

    // ═══════════════════════════════════════════════════════════════
    //  MAIN LOOP
    // ═══════════════════════════════════════════════════════════════

    @Override
    public void loop() {
        loopCount++;
        long now = System.currentTimeMillis();

        // Always: buttons + drive
        handleButtons();
        handleDrive();
        handleIntakes();

        // Throttled: heavy stuff
        boolean doHeavy = (loopCount % LOOP_THROTTLE) == 0;

        if (doHeavy) {
            if (now - lastVoltTime > VOLTAGE_INTERVAL) {
                updateVoltage();
                lastVoltTime = now;
            }

            if (depoOn) {
                updateVelocity();
                runDeposit(now);
            }
        }

        // Lazy telemetry
        if (now - lastTeleTime > TELEMETRY_INTERVAL) {
            if (exportMode) showExport();
            else showTelemetry();
            lastTeleTime = now;
        }
    }

    // ═══════════════════════════════════════════════════════════════
    //  BUTTONS
    // ═══════════════════════════════════════════════════════════════

    private void handleButtons() {
        int btn = 0;
        if (gamepad1.x) btn |= 1;
        if (gamepad1.left_trigger > 0.5) btn |= 2;
        if (gamepad1.dpad_up) btn |= 4;
        if (gamepad1.dpad_down) btn |= 8;
        if (gamepad1.y && gamepad1.b) btn |= 16;
        if (gamepad1.right_bumper) btn |= 32;
        if (gamepad1.left_bumper && gamepad1.left_stick_button && gamepad1.right_stick_button) btn |= 64;

        int rise = btn & ~lastBtn;
        int fall = lastBtn & ~btn;

        if ((rise & 1) != 0) {
            depoOn = !depoOn;
            if (!depoOn) {
                robot.stopDeposit();
                if (recording) endSession();
                resetBoost();
            }
        }

        if ((rise & 2) != 0 && depoOn) {
            startSession();
            if (robot.transferServo != null) {
                robot.transferServo.setPosition(SERVO_SHOOT);
                servoTime = System.currentTimeMillis();
                predActive = true;
            }
        }
        if ((fall & 2) != 0) {
            if (recording) endSession();
            if (robot.transferServo != null) robot.transferServo.setPosition(SERVO_IDLE);
            predActive = false;
        }

        if ((rise & 4) != 0) targetVel = Math.min(1500, targetVel + 25);
        if ((rise & 8) != 0) targetVel = Math.max(100, targetVel - 25);

        if ((rise & 16) != 0) {
            exportMode = !exportMode;
            javaFmt = false;
        }
        if (exportMode && (gamepad1.dpad_left || gamepad1.dpad_right)) {
            javaFmt = !javaFmt;
        }

        if ((rise & 32) != 0) {
            saveFile();
            saveStatic();
            try { gamepad1.rumble(150); } catch (Exception ignored) {}
        }

        if ((rise & 64) != 0) resetDefaults();

        lastBtn = btn;
    }

    // ═══════════════════════════════════════════════════════════════
    //  DRIVE
    // ═══════════════════════════════════════════════════════════════

    private void handleDrive() {
        double f = -gamepad1.left_stick_y * 0.8;
        double s = gamepad1.left_stick_x * 0.8;
        double r = gamepad1.right_stick_x * 0.6;

        if (Math.abs(f) < 0.05 && Math.abs(s) < 0.05 && Math.abs(r) < 0.05) {
            setPower(0, 0, 0, 0);
            return;
        }

        double fl = f + s + r, fr = f - s - r;
        double bl = f - s + r, br = f + s - r;
        double m = Math.max(1, Math.max(Math.max(Math.abs(fl), Math.abs(fr)),
                Math.max(Math.abs(bl), Math.abs(br))));
        setPower(fl/m, fr/m, bl/m, br/m);
    }

    private void setPower(double fl, double fr, double bl, double br) {
        if (robot.frontLeft != null) robot.frontLeft.setPower(fl);
        if (robot.frontRight != null) robot.frontRight.setPower(fr);
        if (robot.backLeft != null) robot.backLeft.setPower(bl);
        if (robot.backRight != null) robot.backRight.setPower(br);
    }

    private void handleIntakes() {
        double p = gamepad1.a ? 1 : (gamepad1.b && !exportMode ? -1 : 0);
        if (robot.intake1 != null) robot.intake1.setPower(p);
        if (robot.intake2 != null) robot.intake2.setPower(-p);
    }

    // ═══════════════════════════════════════════════════════════════
    //  VOLTAGE
    // ═══════════════════════════════════════════════════════════════

    private void updateVoltage() {
        if (voltSensor != null) {
            double v = voltSensor.getVoltage();
            voltage = 0.85 * voltage + 0.15 * v;
            voltScale = NOM_VOLTAGE / clamp(voltage, 10, 14);
        }
    }

    // ═══════════════════════════════════════════════════════════════
    //  VELOCITY (simplified Kalman)
    // ═══════════════════════════════════════════════════════════════

    private void updateVelocity() {
        if (robot.deposit1 == null) return;

        // Read motors
        m1Vel = robot.getDeposit1Velocity();
        m2Vel = robot.getDeposit2Velocity();
        m1Filt = 0.7 * m1Filt + 0.3 * m1Vel;
        m2Filt = 0.7 * m2Filt + 0.3 * m2Vel;

        double raw = (m1Vel + m2Vel) * 0.5;

        // Simplified Kalman
        long now = System.currentTimeMillis();
        if (lastKTime == 0) {
            kVel = raw;
            kAccel = 0;
            lastKTime = now;
            return;
        }

        double dt = (now - lastKTime) / 1000.0;
        if (dt < 0.001) return;
        lastKTime = now;

        // Predict
        double predVel = kVel + kAccel * dt;
        kP = kP + 50 * dt;

        // Update
        double k = kP / (kP + 200);
        double innov = raw - predVel;
        kVel = predVel + k * innov;
        kAccel = kAccel + (k * 0.3) * innov / dt;
        kP = (1 - k) * kP;

        // Oscillation detection
        oscHistory[oscIdx] = kVel;
        oscIdx = (oscIdx + 1) % oscHistory.length;

        int cross = 0;
        double lastE = 0;
        for (int i = 0; i < oscHistory.length; i++) {
            double e = oscHistory[i] - targetVel;
            if (i > 0 && lastE * e < 0) cross++;
            lastE = e;
        }
        double crossRate = (double) cross / (oscHistory.length - 1);
        oscScore = 0.85 * oscScore + 0.15 * crossRate;

        if (oscScore > OSC_THRESHOLD) {
            oscDamp = Math.max(0.5, oscDamp * OSC_DAMPING);
        } else {
            oscDamp = Math.min(1.0, oscDamp * 1.015);
        }
    }

    // ═══════════════════════════════════════════════════════════════
    //  DEPOSIT
    // ═══════════════════════════════════════════════════════════════

    private void runDeposit(long now) {
        if (robot.deposit1 == null) return;

        double drop = targetVel - kVel;
        double scale = targetVel / REF_VEL;
        double trig = boostTrigger * scale;
        double cap = boostCap * scale;

        double targetB = 0;

        // Predictive
        if (predActive) {
            double el = now - servoTime;
            if (el < predWindowMs) {
                double ramp = Math.min(1, el / (predWindowMs * predRampUp));
                double pd = trig * 1.5;
                predBoost = getBoost(pd, balls, scale) * predBoostFrac * ramp;
                targetB = predBoost;
            } else {
                predBoost *= 0.9;
            }
        }

        // Reactive
        double effTrig = inBoost ? (trig * hysteresisRatio) : trig;

        if (drop >= effTrig) {
            if (!inBoost) {
                inBoost = true;
                exiting = false;
                boostStart = now;
            }

            double base = getBoost(drop, balls, scale);

            // Damping
            if (drop < trig * 2.5) {
                double t = drop / (trig * 2.5);
                base *= approachDamping + (1 - approachDamping) * t * t;
            }

            // Derivative
            if (kAccel > 0) base -= derivativeGain * kAccel * scale;

            // Decay
            double dur = (now - boostStart) / 1000.0;
            base *= Math.exp(-decayRate * dur * 10);

            // Oscillation damping
            base *= oscDamp;

            // Voltage
            base *= voltScale;

            targetB = Math.max(targetB, base);
        } else {
            if (inBoost) exiting = true;
            if (exiting && drop < trig * 0.3) {
                inBoost = false;
                exiting = false;
            }
        }

        // Ramp
        double rate = targetB > curBoost ? rampUpRate : (exiting ? 0.15 : 0.25);
        curBoost += (targetB - curBoost) * rate;
        curBoost = clamp(curBoost, 0, cap);

        // Integral
        if (Math.abs(drop) < trig * 0.4) {
            integral += drop * integralGain;
            integral = clamp(integral, -integralCap * scale, integralCap * scale);
        } else if (Math.abs(drop) > trig) {
            integral *= 0.9;
        }

        // Motor commands
        double total = curBoost + integral;
        double c1 = (targetVel + total + motor1Bias * scale) * voltScale;
        double c2 = (targetVel + total + motor2Bias * scale) * voltScale;

        // Cached write
        if (Math.abs(c1 - lastM1Cmd) > MOTOR_THRESHOLD) {
            robot.deposit1.setVelocity(c1);
            lastM1Cmd = c1;
        }
        if (Math.abs(c2 - lastM2Cmd) > MOTOR_THRESHOLD) {
            robot.deposit2.setVelocity(c2);
            lastM2Cmd = c2;
        }

        // Record
        if (recording && sCount < MAX_SAMPLES) {
            if (sCount >= WARMUP) {
                sTimes[sCount] = now;
                sVels[sCount] = kVel;
                sM1[sCount] = m1Filt;
                sM2[sCount] = m2Filt;
                sBoost[sCount] = curBoost;
                sCount++;
                detectBall();
            } else {
                sCount++;
            }
        }
    }

    private double getBoost(double drop, int ball, double scale) {
        double m, e;
        switch (ball) {
            case 0: m = ball1Mult; e = ball1Exp; break;
            case 1: m = ball2Mult; e = ball2Exp; break;
            default: m = ball3Mult; e = ball3Exp; break;
        }
        return m * Math.pow(Math.max(0, drop), e) * Math.sqrt(scale);
    }

    private void resetBoost() {
        curBoost = 0;
        integral = 0;
        inBoost = false;
        exiting = false;
        predBoost = 0;
    }

    // ═══════════════════════════════════════════════════════════════
    //  BALL DETECTION
    // ═══════════════════════════════════════════════════════════════

    private void detectBall() {
        int n = sCount;
        if (n < 22 || n - lastBallIdx < BALL_GAP) return;

        double prev = 0, cur = 0;
        for (int i = 0; i < 5; i++) {
            prev += sVels[n - 12 - i];
            cur += sVels[n - 1 - i];
        }
        prev /= 5; cur /= 5;

        double thresh = DROP_DETECT * (targetVel / REF_VEL);
        if (prev - cur > thresh) {
            // Check stability
            boolean stable = true;
            double st = boostTrigger * (targetVel / REF_VEL) * 0.6;
            for (int i = 18; i <= 21 && n - i >= WARMUP; i++) {
                if (Math.abs(sVels[n - i] - targetVel) > st) {
                    stable = false;
                    break;
                }
            }
            if (stable) {
                balls++;
                lastBallIdx = n;
            }
        }
    }

    // ═══════════════════════════════════════════════════════════════
    //  SESSION
    // ═══════════════════════════════════════════════════════════════

    private void startSession() {
        recording = true;
        sCount = 0;
        balls = 0;
        lastBallIdx = 0;
        resetBoost();

        // Reset Kalman
        kVel = 0; kAccel = 0; lastKTime = 0; kP = 1;

        // Reset oscillation
        for (int i = 0; i < oscHistory.length; i++) oscHistory[i] = 0;
        oscScore = 0;
        oscDamp = 1.0;

        // Generate trial
        genTrial();
        copy(trialP, curP);
        syncFromParams();
    }

    private void genTrial() {
        if (eliteN >= 2 && rand.nextDouble() < 0.3) {
            int i1 = rand.nextInt(eliteN);
            int i2 = rand.nextInt(eliteN);
            while (i2 == i1 && eliteN > 1) i2 = rand.nextInt(eliteN);
            for (int i = 0; i < PARAM_COUNT; i++) {
                double b = rand.nextDouble();
                trialP[i] = eliteP[i1][i] * b + eliteP[i2][i] * (1 - b);
            }
            perturb(trialP, temp * 0.5);
        } else {
            copy(bestP, trialP);
            perturb(trialP, temp);
        }
    }

    private void endSession() {
        recording = false;
        predActive = false;

        int valid = sCount - WARMUP;
        if (valid < MIN_SAMPLES || balls < MIN_BALLS) {
            noImprove++;
            checkRestart();
            return;
        }

        // Analyze
        double loss = analyze();
        lastLoss = loss;
        lastBalls = balls;
        sessions++;

        // Outlier check
        lastOutlier = false;
        if (lossCount >= 5) {
            double avg = 0;
            for (int i = 0; i < lossCount; i++) avg += lossHist[i];
            avg /= lossCount;
            if (loss > avg * OUTLIER_MULT) {
                outliers++;
                lastOutlier = true;
                return;
            }
        }

        // Update loss history
        lossHist[lossIdx] = loss;
        lossIdx = (lossIdx + 1) % LOSS_HISTORY;
        if (lossCount < LOSS_HISTORY) lossCount++;
        updateConfidence();

        // Check improvement
        boolean improved = false;
        if (loss < bestLoss) {
            bestLoss = loss;
            copy(curP, bestP);
            improvements++;
            noImprove = 0;
            degradeCount = 0;
            improved = true;

            rollbackLoss = bestLoss;
            copy(bestP, rollbackP);

            addElite(curP, loss);

            try { gamepad1.rumble(250); } catch (Exception ignored) {}
        } else {
            double d = loss - bestLoss;
            double p = Math.exp(-d / (temp * 500));
            if (rand.nextDouble() < p) {
                // Accept worse
            }
            noImprove++;

            // Rollback check
            if (loss > rollbackLoss * ROLLBACK_RATIO) {
                degradeCount++;
                if (degradeCount >= ROLLBACK_THRESH) {
                    doRollback();
                }
            } else {
                degradeCount = 0;
            }
        }

        temp = Math.max(MIN_TEMP, temp * COOL_RATE);
        checkRestart();
    }

    private double analyze() {
        double scale = targetVel / REF_VEL;
        double tDrop = 35 * scale;
        double tOver = 8 * scale;

        double maxDrop = 0, maxOver = 0;
        double sumSq = 0, sumM1Err = 0, sumM2Err = 0;
        int count = 0;
        int oscCross = 0;
        double lastErr = 0;

        for (int i = WARMUP; i < sCount; i++) {
            double v = sVels[i];
            double e = v - targetVel;
            sumSq += e * e;
            sumM1Err += Math.abs(sM1[i] - targetVel);
            sumM2Err += Math.abs(sM2[i] - targetVel);
            count++;

            if (count > 1 && lastErr * e < 0) oscCross++;
            lastErr = e;

            if (targetVel - v > maxDrop) maxDrop = targetVel - v;
            if (v - targetVel > maxOver) maxOver = v - targetVel;
        }

        double rms = count > 0 ? Math.sqrt(sumSq / count) : 100;
        double stab = Math.max(0, 100 - rms);
        double m1Avg = count > 0 ? sumM1Err / count : 0;
        double m2Avg = count > 0 ? sumM2Err / count : 0;
        double motorDiff = Math.abs(m1Avg - m2Avg);
        boolean oscillated = (double) oscCross / Math.max(1, count) > OSC_THRESHOLD;

        // Loss calculation
        double loss = 0;

        double overErr = Math.max(0, maxOver - tOver);
        loss += 18 * overErr * overErr;
        loss += 2 * Math.max(0, maxDrop - tDrop);
        loss += 4 * Math.pow(Math.max(0, 94 - stab), 2);
        loss += 3 * motorDiff;
        if (oscillated) loss += 25;
        if (balls < 3) loss *= 1.2;

        return loss;
    }

    private void addElite(double[] p, double l) {
        int worst = 0;
        for (int i = 1; i < ELITE_COUNT; i++) {
            if (eliteL[i] > eliteL[worst]) worst = i;
        }
        if (l < eliteL[worst]) {
            copy(p, eliteP[worst]);
            eliteL[worst] = l;
            if (eliteN < ELITE_COUNT) eliteN++;
        }
    }

    private void doRollback() {
        copy(rollbackP, bestP);
        copy(rollbackP, curP);
        syncFromParams();
        bestLoss = rollbackLoss;
        degradeCount = 0;
        rollbacks++;
        temp = RESTART_TEMP;
    }

    private void checkRestart() {
        if (noImprove >= RESTART_THRESH) {
            temp = RESTART_TEMP;
            noImprove = 0;
            if (eliteN > 0 && rand.nextDouble() < 0.3) {
                copy(eliteP[rand.nextInt(eliteN)], curP);
            } else {
                copy(bestP, curP);
            }
            syncFromParams();
        }
    }

    private void updateConfidence() {
        if (lossCount < 5) {
            confidence = 0;
            return;
        }

        double mean = 0;
        for (int i = 0; i < lossCount; i++) mean += lossHist[i];
        mean /= lossCount;

        double var = 0;
        for (int i = 0; i < lossCount; i++) var += (lossHist[i] - mean) * (lossHist[i] - mean);
        var /= lossCount;

        double lossConf = Math.max(0, 1 - bestLoss / 100);
        double varConf = Math.max(0, 1 - Math.sqrt(var) / 50);
        confidence = clamp((lossConf + varConf) / 2, 0, 1);
    }

    // ═══════════════════════════════════════════════════════════════
    //  PERTURBATION
    // ═══════════════════════════════════════════════════════════════

    private void perturb(double[] p, double t) {
        for (int i = 0; i < PARAM_COUNT; i++) {
            double n = (rand.nextDouble() * 2 - 1) * t;
            switch (i) {
                case 0: case 2: case 4: p[i] = clamp(p[i] * (1 + n * 0.18), 0.3, 4); break;
                case 1: case 3: case 5: p[i] = clamp(p[i] + n * 0.35, 1, 2.6); break;
                case 6: p[i] = clamp(p[i] * (1 + n * 0.15), 20, 90); break;
                case 7: p[i] = clamp(p[i] * (1 + n * 0.15), 100, 600); break;
                case 8: p[i] = clamp(p[i] + n * 0.15, 0.1, 1); break;
                case 9: p[i] = clamp(p[i] + n * 0.12, 0.2, 0.95); break;
                case 10: p[i] = clamp(p[i] + n * 0.05, 0.005, 0.2); break;
                case 11: p[i] = clamp(p[i] + n * 0.012, 0.003, 0.06); break;
                case 12: p[i] = clamp(p[i] + n * 0.1, 0.5, 0.95); break;
                case 13: p[i] = clamp(p[i] + n * 0.12, 0, 0.7); break;
                case 14: p[i] = clamp(p[i] + n * 40, 30, 250); break;
                case 15: p[i] = clamp(p[i] + n * 0.15, 0.2, 0.9); break;
                case 16: p[i] = clamp(p[i] + n * 0.003, 0, 0.015); break;
                case 17: p[i] = clamp(p[i] + n * 12, 5, 70); break;
                case 18: case 19: p[i] = clamp(p[i] + n * 12, -40, 40); break;
            }
        }
    }

    // ═══════════════════════════════════════════════════════════════
    //  PERSISTENCE
    // ═══════════════════════════════════════════════════════════════

    private void saveFile() {
        try {
            PrintWriter w = new PrintWriter(new FileWriter(SAVE_FILE));
            w.println("v=4.2");
            w.println("loss=" + bestLoss);
            w.println("rloss=" + rollbackLoss);
            w.println("sessions=" + sessions);
            w.println("improvements=" + improvements);
            w.println("temp=" + temp);
            w.println("conf=" + confidence);
            for (int i = 0; i < PARAM_COUNT; i++) {
                w.println("p" + i + "=" + bestP[i]);
                w.println("r" + i + "=" + rollbackP[i]);
            }
            w.close();
        } catch (Exception ignored) {}
    }

    private void loadFile() {
        try {
            File f = new File(SAVE_FILE);
            if (!f.exists()) return;
            BufferedReader r = new BufferedReader(new FileReader(f));
            String line;
            while ((line = r.readLine()) != null) {
                String[] p = line.split("=");
                if (p.length != 2) continue;
                String k = p[0];
                double v = Double.parseDouble(p[1]);
                if (k.equals("loss")) bestLoss = v;
                else if (k.equals("rloss")) rollbackLoss = v;
                else if (k.equals("sessions")) sessions = (int) v;
                else if (k.equals("improvements")) improvements = (int) v;
                else if (k.equals("temp")) temp = v;
                else if (k.equals("conf")) confidence = v;
                else if (k.startsWith("p")) {
                    int idx = Integer.parseInt(k.substring(1));
                    if (idx >= 0 && idx < PARAM_COUNT) bestP[idx] = v;
                } else if (k.startsWith("r")) {
                    int idx = Integer.parseInt(k.substring(1));
                    if (idx >= 0 && idx < PARAM_COUNT) rollbackP[idx] = v;
                }
            }
            r.close();
            copy(bestP, curP);
            syncFromParams();
        } catch (Exception ignored) {}
    }

    private void saveStatic() {
        if (staticBestParams == null) staticBestParams = new double[PARAM_COUNT];
        if (staticRollbackParams == null) staticRollbackParams = new double[PARAM_COUNT];
        copy(bestP, staticBestParams);
        copy(rollbackP, staticRollbackParams);
        staticBestLoss = bestLoss;
        staticSessions = sessions;
        staticImprovements = improvements;
        staticInit = true;
    }

    private void loadStatic() {
        if (staticBestParams == null) return;
        copy(staticBestParams, bestP);
        if (staticRollbackParams != null) copy(staticRollbackParams, rollbackP);
        copy(bestP, curP);
        syncFromParams();
        bestLoss = staticBestLoss;
        sessions = staticSessions;
        improvements = staticImprovements;
    }

    private void resetDefaults() {
        ball1Mult = 1.0; ball1Exp = 1.5;
        ball2Mult = 1.6; ball2Exp = 1.6;
        ball3Mult = 1.4; ball3Exp = 1.55;
        boostTrigger = 45; boostCap = 320;
        rampUpRate = 0.4; approachDamping = 0.6;
        derivativeGain = 0.06; decayRate = 0.015;
        hysteresisRatio = 0.7;
        predBoostFrac = 0.35; predWindowMs = 100; predRampUp = 0.5;
        integralGain = 0.002; integralCap = 30;
        motor1Bias = 0; motor2Bias = 0;

        syncFromFields();
        copy(curP, bestP);
        copy(curP, rollbackP);
        bestLoss = Double.MAX_VALUE;
        rollbackLoss = Double.MAX_VALUE;
        sessions = 0;
        improvements = 0;
        temp = INIT_TEMP;
        confidence = 0;
        eliteN = 0;
        lossCount = 0;

        staticBestParams = null;
        staticInit = false;
        try { new File(SAVE_FILE).delete(); } catch (Exception ignored) {}
    }

    // ═══════════════════════════════════════════════════════════════
    //  PARAM SYNC
    // ═══════════════════════════════════════════════════════════════

    private void syncFromFields() {
        curP[0] = ball1Mult; curP[1] = ball1Exp;
        curP[2] = ball2Mult; curP[3] = ball2Exp;
        curP[4] = ball3Mult; curP[5] = ball3Exp;
        curP[6] = boostTrigger; curP[7] = boostCap;
        curP[8] = rampUpRate; curP[9] = approachDamping;
        curP[10] = derivativeGain; curP[11] = decayRate;
        curP[12] = hysteresisRatio;
        curP[13] = predBoostFrac; curP[14] = predWindowMs; curP[15] = predRampUp;
        curP[16] = integralGain; curP[17] = integralCap;
        curP[18] = motor1Bias; curP[19] = motor2Bias;
    }

    private void syncFromParams() {
        ball1Mult = curP[0]; ball1Exp = curP[1];
        ball2Mult = curP[2]; ball2Exp = curP[3];
        ball3Mult = curP[4]; ball3Exp = curP[5];
        boostTrigger = curP[6]; boostCap = curP[7];
        rampUpRate = curP[8]; approachDamping = curP[9];
        derivativeGain = curP[10]; decayRate = curP[11];
        hysteresisRatio = curP[12];
        predBoostFrac = curP[13]; predWindowMs = curP[14]; predRampUp = curP[15];
        integralGain = curP[16]; integralCap = curP[17];
        motor1Bias = curP[18]; motor2Bias = curP[19];
    }

    // ═══════════════════════════════════════════════════════════════
    //  UTILITIES
    // ═══════════════════════════════════════════════════════════════

    private static double clamp(double v, double lo, double hi) {
        return v < lo ? lo : (v > hi ? hi : v);
    }

    private static void copy(double[] s, double[] d) {
        System.arraycopy(s, 0, d, 0, s.length);
    }

    // ═══════════════════════════════════════════════════════════════
    //  TELEMETRY
    // ═══════════════════════════════════════════════════════════════

    private void showTelemetry() {
        long el = (System.currentTimeMillis() - startTime) / 1000;

        telemetry.addLine("══ DepoTuner v4.2 ══");
        telemetry.addData("Time", "%d:%02d", el / 60, el % 60);
        telemetry.addData("Status", "%s | %.0f",
                depoOn ? (recording ? "●REC" : "▶ON") : "■OFF", targetVel);

        if (voltSensor != null) {
            telemetry.addData("Battery", "%.1fV (×%.2f)", voltage, voltScale);
        }

        if (depoOn) {
            telemetry.addData("Vel", "%.0f [err:%.0f]", kVel, targetVel - kVel);
            telemetry.addData("Motors", "M1:%.0f M2:%.0f", m1Filt, m2Filt);
            telemetry.addData("Boost", "%.0f | Int:%.1f", curBoost, integral);
            if (oscScore > OSC_THRESHOLD) {
                telemetry.addData("⚠ Osc", "%.0f%% damp:%.0f%%", oscScore * 100, oscDamp * 100);
            }
        }
        telemetry.addLine("");

        // Confidence bar
        int filled = (int) (confidence * 10);
        StringBuilder bar = new StringBuilder("[");
        for (int i = 0; i < 10; i++) bar.append(i < filled ? "█" : "░");
        bar.append("]");
        telemetry.addData("Confidence", "%s %.0f%%", bar.toString(), confidence * 100);
        telemetry.addLine("");

        telemetry.addData("Sessions", "%d", sessions);
        telemetry.addData("Improved", "%d", improvements);
        telemetry.addData("Best", "%.1f", bestLoss);
        telemetry.addData("Temp", "%.2f", temp);
        if (outliers > 0) telemetry.addData("Outliers", "%d", outliers);
        if (rollbacks > 0) telemetry.addData("Rollbacks", "%d", rollbacks);
        telemetry.addLine("");

        if (lastBalls > 0) {
            String stat = lastOutlier ? "⚠OUTLIER" : (lastLoss <= bestLoss ? "✓BEST" : "");
            telemetry.addData("Last", "%d balls %.1f %s", lastBalls, lastLoss, stat);
        }
        telemetry.addLine("");
        telemetry.addLine("Y+B=Export | RB=Save");

        telemetry.update();
    }

    private void showExport() {
        telemetry.addLine("══════ EXPORT ══════");
        telemetry.addLine("D-Pad L/R = format | Y+B = exit");
        telemetry.addData("Format", javaFmt ? "JAVA" : "VALUES");
        telemetry.addLine("────────────────────");

        if (javaFmt) {
            telemetry.addLine("");
            telemetry.addLine("// Ball boost");
            telemetry.addLine(String.format("double ball1Mult = %.4f;", bestP[0]));
            telemetry.addLine(String.format("double ball1Exp  = %.4f;", bestP[1]));
            telemetry.addLine(String.format("double ball2Mult = %.4f;", bestP[2]));
            telemetry.addLine(String.format("double ball2Exp  = %.4f;", bestP[3]));
            telemetry.addLine(String.format("double ball3Mult = %.4f;", bestP[4]));
            telemetry.addLine(String.format("double ball3Exp  = %.4f;", bestP[5]));
            telemetry.addLine("");
            telemetry.addLine("// Boost response");
            telemetry.addLine(String.format("double boostTrigger = %.2f;", bestP[6]));
            telemetry.addLine(String.format("double boostCap     = %.1f;", bestP[7]));
            telemetry.addLine(String.format("double rampUpRate   = %.4f;", bestP[8]));
            telemetry.addLine(String.format("double approachDamp = %.4f;", bestP[9]));
            telemetry.addLine(String.format("double derivGain    = %.5f;", bestP[10]));
            telemetry.addLine(String.format("double decayRate    = %.5f;", bestP[11]));
            telemetry.addLine(String.format("double hystRatio    = %.4f;", bestP[12]));
            telemetry.addLine("");
            telemetry.addLine("// Predictive");
            telemetry.addLine(String.format("double predFrac   = %.4f;", bestP[13]));
            telemetry.addLine(String.format("double predWindow = %.1f;", bestP[14]));
            telemetry.addLine(String.format("double predRamp   = %.4f;", bestP[15]));
            telemetry.addLine("");
            telemetry.addLine("// Integral + motors");
            telemetry.addLine(String.format("double intGain = %.6f;", bestP[16]));
            telemetry.addLine(String.format("double intCap  = %.2f;", bestP[17]));
            telemetry.addLine(String.format("double m1Bias  = %.2f;", bestP[18]));
            telemetry.addLine(String.format("double m2Bias  = %.2f;", bestP[19]));
        } else {
            for (int i = 0; i < PARAM_COUNT; i++) {
                telemetry.addData("p" + i, "%.4f", bestP[i]);
            }
        }

        telemetry.addLine("");
        telemetry.addData("Best", "%.2f", bestLoss);
        telemetry.addData("Confidence", "%.0f%%", confidence * 100);

        telemetry.update();
    }

    // ═══════════════════════════════════════════════════════════════
    //  STOP
    // ═══════════════════════════════════════════════════════════════

    @Override
    public void stop() {
        robot.stop();
        saveFile();
        saveStatic();

        telemetry.clearAll();
        telemetry.addLine("═══════════════════════════════");
        telemetry.addLine("     DepoTuner v4.2 SAVED");
        telemetry.addLine("═══════════════════════════════");
        telemetry.addData("Sessions", "%d", sessions);
        telemetry.addData("Improvements", "%d", improvements);
        telemetry.addData("Best Loss", "%.2f", bestLoss);
        telemetry.addData("Confidence", "%.0f%%", confidence * 100);
        telemetry.addLine("");
        telemetry.addLine("All features retained:");
        telemetry.addLine("  ✓ Kalman filter");
        telemetry.addLine("  ✓ Voltage compensation");
        telemetry.addLine("  ✓ Oscillation detection");
        telemetry.addLine("  ✓ Rollback protection");
        telemetry.addLine("  ✓ Outlier rejection");
        telemetry.addLine("  ✓ Confidence scoring");
        telemetry.addLine("  ✓ Dual motor tracking");
        telemetry.addLine("");
        telemetry.addLine("Use Y+B before stop to export.");
        telemetry.update();
    }
}