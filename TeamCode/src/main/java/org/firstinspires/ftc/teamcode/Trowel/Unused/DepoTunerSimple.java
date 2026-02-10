package org.firstinspires.ftc.teamcode.Trowel.Unused;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import org.firstinspires.ftc.teamcode.Trowel.Configs.TrowelHardware;

import java.util.Random;

/**
 * DepoTuner v6.0 - SIMPLIFIED CONSISTENCY TUNER
 *
 * PHILOSOPHY:
 *   - Only 6 parameters (down from 20)
 *   - ALL shots must be EQUAL (variation is heavily punished)
 *   - Simple P-D control with feed-forward
 *   - Driver feedback matters most
 *
 * THE 6 PARAMETERS:
 *   1. feedForward   - Base power to maintain speed
 *   2. kP            - Proportional gain (react to error)
 *   3. kD            - Derivative gain (dampen oscillation)
 *   4. boostStrength - Extra power when ball hits
 *   5. boostDecay    - How fast boost fades
 *   6. firstBallScale - Multiplier for first ball (usually <1)
 *
 * SCORING:
 *   - Consistency is 70% of score (all balls same)
 *   - No overshoot is 20% of score
 *   - Recovery speed is 10% of score
 *
 * CONTROLS:
 *   GP1: Drive | LT=Shoot | LB=45% | RB=70% | Back=Reset
 *   GP2: X=Depo | A/B=Intake | DPad=Rate | Stick=Speed
 */
@TeleOp(name = "Do NOT use 3", group = "Tuning")
public class DepoTunerSimple extends OpMode {

    // ═══════════════════════════════════════════════════════════════
    //  THE 6 TUNABLE PARAMETERS
    // ═══════════════════════════════════════════════════════════════

    private double feedForward = 0.95;      // Base multiplier (0.8 - 1.1)
    private double kP = 0.008;              // Proportional gain (0.001 - 0.02)
    private double kD = 0.002;              // Derivative gain (0.0005 - 0.01)
    private double boostStrength = 0.15;    // Boost on ball drop (0.05 - 0.4)
    private double boostDecay = 0.92;       // Decay per cycle (0.85 - 0.98)
    private double firstBallScale = 0.4;    // First ball multiplier (0.4 - 1.0)

    // ═══════════════════════════════════════════════════════════════
    //  CONFIGURATION
    // ═══════════════════════════════════════════════════════════════

    private static final double NOMINAL_VOLTAGE = 12.5;
    private static final int MAX_SAMPLES = 300;

    // Speed modes
    private static final double SPEED_FULL = 1.0;
    private static final double SPEED_70 = 0.70;
    private static final double SPEED_45 = 0.45;

    // Drive
    private static final double DRIVE_POWER = 0.8;
    private static final double ROTATE_POWER = 0.6;

    // Servo
    private static final double SERVO_SHOOT = 0.7;
    private static final double SERVO_IDLE = 0.3;

    // Ball detection
    private static final double DROP_THRESHOLD = 0.06;  // 6% velocity drop
    private static final int MIN_BALL_GAP = 30;         // samples between balls

    // ═══════════════════════════════════════════════════════════════
    //  STATE
    // ═══════════════════════════════════════════════════════════════

    private TrowelHardware robot;
    private VoltageSensor voltageSensor;
    private Random random;
    private Gamepad gp1, gp2;

    // Timing
    private long startTime;
    private int loopCount = 0;

    // Voltage
    private double voltage = NOMINAL_VOLTAGE;
    private double voltageComp = 1.0;

    // Speed
    private double speedMode = SPEED_FULL;
    private String speedLabel = "100%";
    private double targetVelocity = 640.0;
    private double effectiveTarget = 640.0;

    // Velocity tracking
    private double velocity = 0;
    private double lastVelocity = 0;
    private double velocityDelta = 0;

    // Control state
    private double currentBoost = 0;
    private boolean depositOn = false;

    // Recording
    private boolean recording = false;
    private double[] samples = new double[MAX_SAMPLES];
    private int sampleCount = 0;

    // Ball tracking
    private int ballCount = 0;
    private int lastBallSample = 0;
    private double[] ballDropDepth = new double[5];   // How deep each ball dropped
    private double[] ballPeakOver = new double[5];    // Overshoot per ball
    private double[] ballRecoveryTime = new double[5]; // Samples to recover

    // Tuning state
    private double[] bestParams = new double[6];
    private double[] currentParams = new double[6];
    private double bestScore = -9999;
    private double lastScore = 0;
    private boolean paramsLocked = false;
    private double searchRadius = 0.3;  // How far to explore

    // Stats
    private int totalShots = 0;
    private int goodShots = 0;
    private int badShots = 0;
    private String lastRating = "-";
    private boolean awaitingRating = false;

    // Last shot analysis
    private double lastConsistency = 0;
    private double lastOvershoot = 0;
    private double lastRecovery = 0;

    // Button states
    private boolean prev_gp1_lt, prev_gp1_back;
    private boolean prev_gp2_x, prev_gp2_up, prev_gp2_down, prev_gp2_left, prev_gp2_right;
    private boolean prev_gp2_rb;

    // ═══════════════════════════════════════════════════════════════
    //  INIT
    // ═══════════════════════════════════════════════════════════════

    @Override
    public void init() {
        robot = new TrowelHardware(hardwareMap);
        robot.resetDepositEncoders();
        random = new Random();

        // Motors
        configureMotors();

        // Voltage
        try {
            voltageSensor = hardwareMap.voltageSensor.iterator().next();
            voltage = voltageSensor.getVoltage();
        } catch (Exception e) {
            voltageSensor = null;
        }

        // Init params
        saveParamsToArray(currentParams);
        copyArray(currentParams, bestParams);

        // Servo
        if (robot.transferServo != null) {
            robot.transferServo.setPosition(SERVO_IDLE);
        }

        startTime = System.currentTimeMillis();

        telemetry.addLine("═══════════════════════════════════");
        telemetry.addLine("  DepoTuner v6.0 - SIMPLE MODE");
        telemetry.addLine("═══════════════════════════════════");
        telemetry.addLine("");
        telemetry.addLine("Only 6 parameters!");
        telemetry.addLine("Priority: ALL SHOTS EQUAL");
        telemetry.addLine("");
        telemetry.addLine("After shooting, RATE IT:");
        telemetry.addLine("  GP2 D-Up = GOOD");
        telemetry.addLine("  GP2 D-Down = BAD");
        telemetry.addLine("  GP2 D-Right = PERFECT");
        telemetry.addLine("");
        telemetry.addLine("GP1: LT=Shoot LB=45% RB=70%");
        telemetry.addLine("GP2: X=Depo A/B=Intake");
        telemetry.update();
    }

    private void configureMotors() {
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
        gp1 = gamepad1;
        gp2 = gamepad2;

        // Always
        updateSpeedMode();
        handleDriver1();
        handleDriver2();
        handleDrive();
        handleIntakes();

        // Every 4th loop
        if (loopCount % 4 == 0) {
            updateVoltage();
            if (depositOn) {
                updateVelocity();
                runControl();
            }
        }

        // Telemetry every 8th loop
        if (loopCount % 8 == 0) {
            showTelemetry();
        }

        updatePrevStates();
    }

    // ═══════════════════════════════════════════════════════════════
    //  SPEED MODE (DIRECTLY CHECKED EACH LOOP)
    // ═══════════════════════════════════════════════════════════════

    private void updateSpeedMode() {
        if (gp1.left_bumper) {
            speedMode = SPEED_45;
            speedLabel = "45%";
        } else if (gp1.right_bumper) {
            speedMode = SPEED_70;
            speedLabel = "70%";
        } else {
            speedMode = SPEED_FULL;
            speedLabel = "100%";
        }
        effectiveTarget = targetVelocity * speedMode;
    }

    // ═══════════════════════════════════════════════════════════════
    //  DRIVER 1: Drive, Shoot, Reset
    // ═══════════════════════════════════════════════════════════════

    private void handleDriver1() {
        boolean lt = gp1.left_trigger > 0.5;

        // Shoot
        if (lt && !prev_gp1_lt && depositOn) {
            startRecording();
            if (robot.transferServo != null) {
                robot.transferServo.setPosition(SERVO_SHOOT);
            }
        }
        if (!lt && prev_gp1_lt) {
            if (recording) endRecording();
            if (robot.transferServo != null) {
                robot.transferServo.setPosition(SERVO_IDLE);
            }
        }

        // Reset
        if (gp1.back && !prev_gp1_back) {
            resetAll();
            rumble(gp1, 400);
        }
    }

    // ═══════════════════════════════════════════════════════════════
    //  DRIVER 2: Deposit, Intake, Rating, Speed Adjust
    // ═══════════════════════════════════════════════════════════════

    private void handleDriver2() {
        // Deposit toggle
        if (gp2.x && !prev_gp2_x) {
            depositOn = !depositOn;
            if (!depositOn) {
                robot.stopDeposit();
                currentBoost = 0;
            }
        }

        // Speed adjust with stick
        double stickY = -gp2.left_stick_y;
        if (Math.abs(stickY) > 0.15) {
            targetVelocity += stickY * 3.0;
            targetVelocity = clamp(targetVelocity, 200, 1200);
        }

        // Lock params
        if (gp2.right_bumper && !prev_gp2_rb) {
            paramsLocked = !paramsLocked;
            rumble(gp2, paramsLocked ? 300 : 100);
        }

        // Rating
        if (awaitingRating) {
            if (gp2.dpad_up && !prev_gp2_up) {
                rateShot(1);  // GOOD
            }
            if (gp2.dpad_down && !prev_gp2_down) {
                rateShot(-1); // BAD
            }
            if (gp2.dpad_right && !prev_gp2_right) {
                rateShot(2);  // PERFECT
            }
            if (gp2.dpad_left && !prev_gp2_left) {
                rateShot(0);  // OK/Skip
            }
        }
    }

    // ═══════════════════════════════════════════════════════════════
    //  DRIVE
    // ═══════════════════════════════════════════════════════════════

    private void handleDrive() {
        double fwd = -gp1.left_stick_y * DRIVE_POWER;
        double str = gp1.left_stick_x * DRIVE_POWER;
        double rot = gp1.right_stick_x * ROTATE_POWER;

        if (Math.abs(fwd) < 0.05 && Math.abs(str) < 0.05 && Math.abs(rot) < 0.05) {
            setPowers(0, 0, 0, 0);
            return;
        }

        double fl = fwd + str + rot;
        double fr = fwd - str - rot;
        double bl = fwd - str + rot;
        double br = fwd + str - rot;

        double max = Math.max(1, Math.max(Math.max(Math.abs(fl), Math.abs(fr)),
                Math.max(Math.abs(bl), Math.abs(br))));
        setPowers(fl/max, fr/max, bl/max, br/max);
    }

    private void setPowers(double fl, double fr, double bl, double br) {
        if (robot.frontLeft != null) robot.frontLeft.setPower(fl);
        if (robot.frontRight != null) robot.frontRight.setPower(fr);
        if (robot.backLeft != null) robot.backLeft.setPower(bl);
        if (robot.backRight != null) robot.backRight.setPower(br);
    }

    // ═══════════════════════════════════════════════════════════════
    //  INTAKES
    // ═══════════════════════════════════════════════════════════════

    private void handleIntakes() {
        double pwr = 0;
        if (gp2.a) pwr = 1.0;
        else if (gp2.b) pwr = -1.0;

        if (robot.intake1 != null) robot.intake1.setPower(pwr);
        if (robot.intake2 != null) robot.intake2.setPower(-pwr);
    }

    // ═══════════════════════════════════════════════════════════════
    //  VOLTAGE
    // ═══════════════════════════════════════════════════════════════

    private void updateVoltage() {
        if (voltageSensor == null) return;
        double v = voltageSensor.getVoltage();
        voltage = voltage * 0.8 + v * 0.2;
        voltageComp = NOMINAL_VOLTAGE / clamp(voltage, 10, 14);
    }

    // ═══════════════════════════════════════════════════════════════
    //  VELOCITY MEASUREMENT
    // ═══════════════════════════════════════════════════════════════

    private void updateVelocity() {
        if (robot.deposit1 == null) return;

        double v1 = robot.getDeposit1Velocity();
        double v2 = robot.getDeposit2Velocity();
        double raw = (v1 + v2) * 0.5;

        lastVelocity = velocity;
        velocity = velocity * 0.7 + raw * 0.3;
        velocityDelta = velocity - lastVelocity;
    }

    // ═══════════════════════════════════════════════════════════════
    //  SIMPLE P-D CONTROL WITH BOOST
    // ═══════════════════════════════════════════════════════════════

    private void runControl() {
        if (robot.deposit1 == null) return;

        // Error (normalized)
        double error = (effectiveTarget - velocity) / effectiveTarget;

        // P term
        double pTerm = kP * error * effectiveTarget;

        // D term (damping)
        double dTerm = -kD * velocityDelta;

        // Boost (decays over time)
        currentBoost *= boostDecay;

        // Detect ball drop and add boost
        if (recording && sampleCount > 20) {
            if (error > DROP_THRESHOLD && sampleCount - lastBallSample > MIN_BALL_GAP) {
                // Ball detected!
                double boostMult = (ballCount == 0) ? firstBallScale : 1.0;
                currentBoost = boostStrength * effectiveTarget * boostMult;

                // Record this ball
                if (ballCount < 5) {
                    ballDropDepth[ballCount] = error * effectiveTarget;
                    ballPeakOver[ballCount] = 0;
                    ballRecoveryTime[ballCount] = 0;
                }
                ballCount++;
                lastBallSample = sampleCount;
            }
        }

        // Track overshoot and recovery for current ball
        if (ballCount > 0 && ballCount <= 5) {
            int idx = ballCount - 1;
            double over = velocity - effectiveTarget;
            if (over > ballPeakOver[idx]) {
                ballPeakOver[idx] = over;
            }
            if (ballRecoveryTime[idx] == 0 && Math.abs(error) < 0.02) {
                ballRecoveryTime[idx] = sampleCount - lastBallSample;
            }
        }

        // Calculate command
        double ff = feedForward * effectiveTarget;
        double cmd = (ff + pTerm + dTerm + currentBoost) * voltageComp;

        // Anti-overshoot: if above target, reduce command
        if (velocity > effectiveTarget * 1.02) {
            cmd = effectiveTarget * 0.9 * voltageComp;
            currentBoost = 0;
        }

        // Send to motors
        if (robot.deposit1 != null) robot.deposit1.setVelocity(cmd);
        if (robot.deposit2 != null) robot.deposit2.setVelocity(cmd);

        // Record sample
        if (recording && sampleCount < MAX_SAMPLES) {
            samples[sampleCount] = velocity;
            sampleCount++;
        }
    }

    // ═══════════════════════════════════════════════════════════════
    //  RECORDING
    // ═══════════════════════════════════════════════════════════════

    private void startRecording() {
        recording = true;
        sampleCount = 0;
        ballCount = 0;
        lastBallSample = 0;
        currentBoost = 0;

        for (int i = 0; i < 5; i++) {
            ballDropDepth[i] = 0;
            ballPeakOver[i] = 0;
            ballRecoveryTime[i] = 0;
        }

        // Generate new params if not locked
        if (!paramsLocked) {
            generateNewParams();
            loadParamsFromArray(currentParams);
        }
    }

    private void endRecording() {
        recording = false;

        if (sampleCount < 50 || ballCount < 2) {
            // Not enough data
            return;
        }

        // Analyze the shot
        analyzeShot();
        awaitingRating = true;
        totalShots++;
    }

    // ═══════════════════════════════════════════════════════════════
    //  SHOT ANALYSIS - CONSISTENCY IS KING
    // ═══════════════════════════════════════════════════════════════

    private void analyzeShot() {
        // === CONSISTENCY SCORE (70% weight) ===
        // All balls should drop the same amount and recover the same

        double avgDrop = 0, avgOver = 0, avgRecovery = 0;
        int validBalls = Math.min(ballCount, 5);

        for (int i = 0; i < validBalls; i++) {
            avgDrop += ballDropDepth[i];
            avgOver += ballPeakOver[i];
            avgRecovery += ballRecoveryTime[i];
        }
        avgDrop /= validBalls;
        avgOver /= validBalls;
        avgRecovery /= validBalls;

        // Calculate variance (how different are the balls?)
        double dropVariance = 0, overVariance = 0, recoveryVariance = 0;
        for (int i = 0; i < validBalls; i++) {
            dropVariance += Math.pow(ballDropDepth[i] - avgDrop, 2);
            overVariance += Math.pow(ballPeakOver[i] - avgOver, 2);
            recoveryVariance += Math.pow(ballRecoveryTime[i] - avgRecovery, 2);
        }
        dropVariance = Math.sqrt(dropVariance / validBalls);
        overVariance = Math.sqrt(overVariance / validBalls);
        recoveryVariance = Math.sqrt(recoveryVariance / validBalls);

        // Consistency score (0-100, higher is better)
        // Lower variance = higher score
        double dropConsistency = Math.max(0, 100 - dropVariance * 2);
        double overConsistency = Math.max(0, 100 - overVariance * 3);
        double recoveryConsistency = Math.max(0, 100 - recoveryVariance * 1);

        lastConsistency = (dropConsistency + overConsistency + recoveryConsistency) / 3;

        // === OVERSHOOT SCORE (20% weight) ===
        // Penalize ANY overshoot heavily
        double maxOver = 0;
        for (int i = 0; i < validBalls; i++) {
            maxOver = Math.max(maxOver, ballPeakOver[i]);
        }
        // Score: 100 if no overshoot, 0 if overshoot > 30
        lastOvershoot = Math.max(0, 100 - maxOver * 3.5);

        // === RECOVERY SCORE (10% weight) ===
        // Faster recovery is better (target: 20 samples or less)
        lastRecovery = Math.max(0, 100 - avgRecovery * 2);

        // === TOTAL SCORE ===
        lastScore = lastConsistency * 0.70 + lastOvershoot * 0.20 + lastRecovery * 0.10;

        // === EXTRA PENALTIES ===

        // HUGE penalty if first ball is very different from others
        if (validBalls >= 2) {
            double firstVsRest = Math.abs(ballDropDepth[0] - avgDrop);
            if (firstVsRest > avgDrop * 0.3) {
                lastScore -= 20;  // Big penalty
            }
        }

        // Penalty if ANY ball overshoots significantly
        for (int i = 0; i < validBalls; i++) {
            if (ballPeakOver[i] > 15) {
                lastScore -= 15;  // Each bad overshoot
            }
        }

        // Penalty if variance between balls is too high
        if (dropVariance > 20) {
            lastScore -= (dropVariance - 20);
        }

        lastScore = clamp(lastScore, 0, 100);
    }

    // ═══════════════════════════════════════════════════════════════
    //  RATING SYSTEM
    // ═══════════════════════════════════════════════════════════════

    private void rateShot(int rating) {
        awaitingRating = false;

        double score = lastScore;

        switch (rating) {
            case 2:  // PERFECT
                score += 30;  // Bonus
                lastRating = "★ PERFECT";
                copyArray(currentParams, bestParams);
                searchRadius = Math.max(0.05, searchRadius * 0.5);
                rumble(gp2, 400);
                goodShots++;
                break;

            case 1:  // GOOD
                score += 10;
                lastRating = "✓ GOOD";
                if (score > bestScore) {
                    bestScore = score;
                    copyArray(currentParams, bestParams);
                }
                searchRadius = Math.max(0.08, searchRadius * 0.7);
                rumble(gp2, 200);
                goodShots++;
                break;

            case 0:  // OK
                lastRating = "~ OK";
                searchRadius = Math.max(0.1, searchRadius * 0.85);
                break;

            case -1: // BAD
            default:
                score -= 20;
                lastRating = "✗ BAD";
                copyArray(bestParams, currentParams);
                loadParamsFromArray(currentParams);
                searchRadius = Math.min(0.5, searchRadius * 1.3);
                rumble(gp2, 100);
                badShots++;
                break;
        }

        lastScore = score;
        if (score > bestScore) {
            bestScore = score;
        }
    }

    // ═══════════════════════════════════════════════════════════════
    //  PARAMETER GENERATION (SIMPLE)
    // ═══════════════════════════════════════════════════════════════

    private void generateNewParams() {
        // Start from best
        copyArray(bestParams, currentParams);

        // Perturb each parameter
        double r = searchRadius;

        // feedForward: 0.8 - 1.1
        currentParams[0] = clamp(currentParams[0] + randPM() * r * 0.1, 0.8, 1.1);

        // kP: 0.001 - 0.02
        currentParams[1] = clamp(currentParams[1] + randPM() * r * 0.005, 0.001, 0.02);

        // kD: 0.0005 - 0.01
        currentParams[2] = clamp(currentParams[2] + randPM() * r * 0.003, 0.0005, 0.01);

        // boostStrength: 0.05 - 0.4
        currentParams[3] = clamp(currentParams[3] + randPM() * r * 0.1, 0.05, 0.4);

        // boostDecay: 0.85 - 0.98
        currentParams[4] = clamp(currentParams[4] + randPM() * r * 0.04, 0.85, 0.98);

        // firstBallScale: 0.4 - 1.0
        currentParams[5] = clamp(currentParams[5] + randPM() * r * 0.15, 0.4, 1.0);
    }

    private double randPM() {
        return random.nextDouble() * 2 - 1;  // -1 to +1
    }

    // ═══════════════════════════════════════════════════════════════
    //  PARAM SYNC
    // ═══════════════════════════════════════════════════════════════

    private void saveParamsToArray(double[] arr) {
        arr[0] = feedForward;
        arr[1] = kP;
        arr[2] = kD;
        arr[3] = boostStrength;
        arr[4] = boostDecay;
        arr[5] = firstBallScale;
    }

    private void loadParamsFromArray(double[] arr) {
        feedForward = arr[0];
        kP = arr[1];
        kD = arr[2];
        boostStrength = arr[3];
        boostDecay = arr[4];
        firstBallScale = arr[5];
    }

    // ═══════════════════════════════════════════════════════════════
    //  RESET
    // ═══════════════════════════════════════════════════════════════

    private void resetAll() {
        feedForward = 0.95;
        kP = 0.008;
        kD = 0.002;
        boostStrength = 0.15;
        boostDecay = 0.92;
        firstBallScale = 0.7;

        saveParamsToArray(currentParams);
        copyArray(currentParams, bestParams);

        bestScore = -9999;
        searchRadius = 0.3;
        paramsLocked = false;

        totalShots = 0;
        goodShots = 0;
        badShots = 0;
        lastRating = "-";
    }

    // ═══════════════════════════════════════════════════════════════
    //  UTILITIES
    // ═══════════════════════════════════════════════════════════════

    private void updatePrevStates() {
        prev_gp1_lt = gp1.left_trigger > 0.5;
        prev_gp1_back = gp1.back;
        prev_gp2_x = gp2.x;
        prev_gp2_up = gp2.dpad_up;
        prev_gp2_down = gp2.dpad_down;
        prev_gp2_left = gp2.dpad_left;
        prev_gp2_right = gp2.dpad_right;
        prev_gp2_rb = gp2.right_bumper;
    }

    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }

    private static void copyArray(double[] src, double[] dst) {
        System.arraycopy(src, 0, dst, 0, src.length);
    }

    private void rumble(Gamepad gp, int ms) {
        try { gp.rumble(ms); } catch (Exception ignored) {}
    }

    // ═══════════════════════════════════════════════════════════════
    //  TELEMETRY
    // ═══════════════════════════════════════════════════════════════

    private void showTelemetry() {
        long elapsed = (System.currentTimeMillis() - startTime) / 1000;

        telemetry.addLine("═══ DepoTuner v6.0 SIMPLE ═══");
        telemetry.addData("Time", "%d:%02d", elapsed / 60, elapsed % 60);

        // Status
        String status = depositOn ? (recording ? "●REC" : "▶ON") : "■OFF";
        telemetry.addData("Status", "%s | %.0f [%s]", status, effectiveTarget, speedLabel);

        if (depositOn) {
            telemetry.addData("Velocity", "%.0f (err:%.0f)", velocity, effectiveTarget - velocity);
            telemetry.addData("Boost", "%.0f", currentBoost);
        }

        telemetry.addData("Battery", "%.1fV", voltage);

        telemetry.addLine("");

        // Rating prompt
        if (awaitingRating) {
            telemetry.addLine("╔═══════════════════════════════╗");
            telemetry.addLine("║   RATE IT! (GP2 D-Pad)        ║");
            telemetry.addLine("║   Up=GOOD  Down=BAD           ║");
            telemetry.addLine("║   Right=PERFECT  Left=Skip    ║");
            telemetry.addLine("╚═══════════════════════════════╝");
            telemetry.addData("Balls", "%d", ballCount);
            telemetry.addData("Consistency", "%.0f%%", lastConsistency);
            telemetry.addData("No Overshoot", "%.0f%%", lastOvershoot);
            telemetry.addData("Recovery", "%.0f%%", lastRecovery);
            telemetry.addData("TOTAL SCORE", "%.0f", lastScore);

            // Per-ball breakdown
            StringBuilder sb = new StringBuilder();
            for (int i = 0; i < Math.min(ballCount, 3); i++) {
                sb.append(String.format("B%d:-%.0f/+%.0f ", i+1, ballDropDepth[i], ballPeakOver[i]));
            }
            telemetry.addData("Balls", sb.toString());
        } else {
            telemetry.addData("Last Rating", lastRating);
            telemetry.addData("Last Score", "%.0f", lastScore);
        }

        telemetry.addLine("");

        // Stats
        telemetry.addData("Shots", "%d (Good:%d Bad:%d)", totalShots, goodShots, badShots);
        telemetry.addData("Best Score", "%.0f", bestScore);
        telemetry.addData("Search", "%.0f%% %s", searchRadius * 100, paramsLocked ? "[LOCKED]" : "");

        telemetry.addLine("");

        // Current params
        telemetry.addLine("─── 6 PARAMS ───");
        telemetry.addData("FF", "%.3f", feedForward);
        telemetry.addData("kP", "%.4f", kP);
        telemetry.addData("kD", "%.4f", kD);
        telemetry.addData("Boost", "%.3f", boostStrength);
        telemetry.addData("Decay", "%.3f", boostDecay);
        telemetry.addData("1stBall", "%.2f", firstBallScale);

        telemetry.addLine("");
        telemetry.addLine("GP1:LT=Shoot LB=45% RB=70% Back=Reset");
        telemetry.addLine("GP2:X=Depo RB=Lock Stick=Speed");

        telemetry.update();
    }

    // ═══════════════════════════════════════════════════════════════
    //  STOP
    // ═══════════════════════════════════════════════════════════════

    @Override
    public void stop() {
        robot.stop();

        telemetry.clearAll();
        telemetry.addLine("═══════════════════════════════════");
        telemetry.addLine("   DepoTuner v6.0 STOPPED");
        telemetry.addLine("═══════════════════════════════════");
        telemetry.addLine("");
        telemetry.addData("Total Shots", "%d", totalShots);
        telemetry.addData("Good", "%d", goodShots);
        telemetry.addData("Bad", "%d", badShots);
        telemetry.addData("Best Score", "%.0f", bestScore);
        telemetry.addLine("");
        telemetry.addLine("BEST PARAMS:");
        telemetry.addData("feedForward", "%.4f", bestParams[0]);
        telemetry.addData("kP", "%.5f", bestParams[1]);
        telemetry.addData("kD", "%.5f", bestParams[2]);
        telemetry.addData("boostStrength", "%.4f", bestParams[3]);
        telemetry.addData("boostDecay", "%.4f", bestParams[4]);
        telemetry.addData("firstBallScale", "%.3f", bestParams[5]);
        telemetry.update();
    }
}