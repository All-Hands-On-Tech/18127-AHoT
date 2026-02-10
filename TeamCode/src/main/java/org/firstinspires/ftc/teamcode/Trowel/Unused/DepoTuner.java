package org.firstinspires.ftc.teamcode.Trowel.Unused;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import org.firstinspires.ftc.teamcode.Trowel.Configs.TrowelHardware;

import java.util.Random;

/**
 * DepoTuner v5.0 - DRIVER FEEDBACK SYSTEM
 *
 * ═══════════════════════════════════════════════════════════════
 *  NEW: DRIVER RATING SYSTEM
 * ═══════════════════════════════════════════════════════════════
 *
 * After each shot sequence, DRIVER 2 rates the result:
 *   D-Pad UP    = GOOD shot (locks in parameters)
 *   D-Pad DOWN  = BAD shot (rejects parameters)
 *   D-Pad LEFT  = OK shot (slight improvement)
 *   D-Pad RIGHT = PERFECT shot (strongly locks parameters)
 *
 * The tuner learns from YOUR feedback, not just sensors!
 *
 * ═══════════════════════════════════════════════════════════════
 *  CONTROLS
 * ═══════════════════════════════════════════════════════════════
 *
 * DRIVER 1 (gamepad1):
 *   Left Stick     = Drive forward/back, strafe
 *   Right Stick    = Rotate
 *   Left Trigger   = Shoot (transfer servo)
 *   Left Bumper    = 45% speed mode (HOLD)
 *   Right Bumper   = 70% speed mode (HOLD)
 *   Y + B          = Export mode
 *   Back           = Reset all
 *
 * DRIVER 2 (gamepad2):
 *   X              = Toggle deposit motors
 *   A              = Intake forward
 *   B              = Intake reverse
 *   Left Stick Y   = Fine adjust target speed
 *   D-Pad UP       = Rate GOOD
 *   D-Pad DOWN     = Rate BAD
 *   D-Pad LEFT     = Rate OK
 *   D-Pad RIGHT    = Rate PERFECT
 *   Left Bumper    = Undo last rating
 *   Right Bumper   = Lock current params (no more changes)
 */
@TeleOp(name = "Do not Use 2", group = "Tuning")
public class DepoTuner extends OpMode {

    // ═══════════════════════════════════════════════════════════════
    //  MOTOR CHARACTERIZATION (measure these for your motors!)
    // ═══════════════════════════════════════════════════════════════

    // GoBILDA 5202/3/4 series typical values
    private static final double MOTOR_MAX_RPM = 435.0;           // 435 RPM for 5202
    private static final double MOTOR_TICKS_PER_REV = 384.5;     // Encoder ticks
    private static final double MOTOR_MAX_TPS = (MOTOR_MAX_RPM / 60.0) * MOTOR_TICKS_PER_REV;  // ~2787 ticks/sec

    // Velocity normalization
    private static final double REFERENCE_VELOCITY = 640.0;      // Your typical target
    private static final double VELOCITY_TOLERANCE = 0.05;       // 5% tolerance band

    // ═══════════════════════════════════════════════════════════════
    //  CONFIGURATION
    // ═══════════════════════════════════════════════════════════════

    private static final double NOMINAL_VOLTAGE = 12.5;
    private static final int MAX_SAMPLES = 200;

    // Speed modes
    private static final double SPEED_FULL = 1.0;
    private static final double SPEED_70 = 0.70;
    private static final double SPEED_45 = 0.45;

    // Drive scaling
    private static final double DRIVE_SCALE = 0.85;
    private static final double STRAFE_SCALE = 0.85;
    private static final double ROTATE_SCALE = 0.65;

    // Servo positions
    private static final double SERVO_SHOOT = 0.7;
    private static final double SERVO_IDLE = 0.3;

    // Session requirements
    private static final int MIN_SAMPLES = 25;
    private static final int WARMUP_SAMPLES = 15;
    private static final double DROP_DETECT_THRESHOLD = 50.0;
    private static final int MIN_BALL_GAP_SAMPLES = 25;

    // ═══════════════════════════════════════════════════════════════
    //  TUNING PARAMETERS - STABLE STARTING POINT
    // ═══════════════════════════════════════════════════════════════

    // Gain curves per ball (ball 1 = conservative)
    private double[] ballGain = {0.35, 0.55, 0.65};      // Multiplier per ball
    private double[] ballCurve = {1.15, 1.25, 1.30};     // Exponent per ball

    // Response parameters
    private double triggerThreshold = 0.08;   // % of target velocity drop to trigger
    private double maxBoostRatio = 0.25;      // Max boost as % of target
    private double rampRate = 0.20;           // How fast to ramp up boost
    private double dampingFactor = 0.70;      // Damping near target
    private double derivativeGain = 0.08;     // Derivative term strength
    private double recoveryRate = 0.03;       // How fast boost decays

    // Predictive
    private double predictiveStrength = 0.12;
    private double predictiveWindow = 70.0;   // ms

    // Integral (anti-windup)
    private double integralStrength = 0.0008;
    private double integralLimit = 0.03;      // As % of target

    // Motor balance
    private double motor1Trim = 0.0;
    private double motor2Trim = 0.0;

    private static final int PARAM_COUNT = 16;

    // ═══════════════════════════════════════════════════════════════
    //  STATE
    // ═══════════════════════════════════════════════════════════════

    private TrowelHardware robot;
    private VoltageSensor voltageSensor;
    private Random random;
    private Gamepad gp1, gp2;

    // Timing
    private long startTime = 0;
    private long lastLoopTime = 0;
    private int loopCount = 0;

    // Voltage compensation
    private double voltage = NOMINAL_VOLTAGE;
    private double voltageCompensation = 1.0;
    private long lastVoltageRead = 0;

    // Speed mode
    private double speedMode = SPEED_FULL;
    private String speedModeLabel = "100%";

    // Target velocity
    private double baseTargetVelocity = 640.0;
    private double effectiveTarget = 640.0;

    // Velocity measurement
    private double rawVelocity = 0;
    private double filteredVelocity = 0;
    private double velocityDerivative = 0;
    private double prevFilteredVelocity = 0;

    // Boost control
    private double currentBoost = 0;
    private double integralAccum = 0;
    private boolean boostActive = false;
    private long boostStartTime = 0;

    // Predictive
    private boolean predictiveActive = false;
    private long servoFireTime = 0;

    // Motor command caching
    private double lastCmd1 = 0, lastCmd2 = 0;

    // Recording
    private boolean recording = false;
    private long[] sampleTimes = new long[MAX_SAMPLES];
    private double[] sampleVels = new double[MAX_SAMPLES];
    private int sampleCount = 0;
    private int detectedBalls = 0;
    private int lastBallSample = 0;

    // Per-ball metrics
    private double[] ballPeakOver = new double[10];
    private double[] ballPeakUnder = new double[10];
    private double[] ballRecoveryTime = new double[10];

    // Deposit state
    private boolean depositOn = false;

    // ═══════════════════════════════════════════════════════════════
    //  OPTIMIZER STATE
    // ═══════════════════════════════════════════════════════════════

    private double[] currentParams = new double[PARAM_COUNT];
    private double[] bestParams = new double[PARAM_COUNT];
    private double[] trialParams = new double[PARAM_COUNT];
    private double[] lastGoodParams = new double[PARAM_COUNT];

    private double bestScore = 0;
    private double explorationRate = 0.5;  // How much to explore vs exploit
    private boolean paramsLocked = false;

    // History for convergence
    private double[] scoreHistory = new double[20];
    private int historyIndex = 0;
    private int historyCount = 0;

    // Stats
    private int totalShots = 0;
    private int goodShots = 0;
    private int perfectShots = 0;
    private int badShots = 0;

    // Last shot info (for rating)
    private boolean awaitingRating = false;
    private double lastShotMaxOver = 0;
    private double lastShotMaxUnder = 0;
    private double lastShotStability = 0;
    private int lastShotBalls = 0;
    private String lastRating = "-";

    // Export mode
    private boolean exportMode = false;
    private boolean javaFormat = false;

    // ═══════════════════════════════════════════════════════════════
    //  BUTTON STATE TRACKING
    // ═══════════════════════════════════════════════════════════════

    private boolean prev_gp1_lt = false;
    private boolean prev_gp1_back = false;
    private boolean prev_gp1_yb = false;
    private boolean prev_gp1_dpadL = false;
    private boolean prev_gp1_dpadR = false;

    private boolean prev_gp2_x = false;
    private boolean prev_gp2_dpadU = false;
    private boolean prev_gp2_dpadD = false;
    private boolean prev_gp2_dpadL = false;
    private boolean prev_gp2_dpadR = false;
    private boolean prev_gp2_lb = false;
    private boolean prev_gp2_rb = false;

    // ═══════════════════════════════════════════════════════════════
    //  INIT
    // ═══════════════════════════════════════════════════════════════

    @Override
    public void init() {
        robot = new TrowelHardware(hardwareMap);
        robot.resetDepositEncoders();
        random = new Random();

        // Configure motors
        configureMotors();

        // Voltage sensor
        try {
            voltageSensor = hardwareMap.voltageSensor.iterator().next();
            voltage = voltageSensor.getVoltage();
        } catch (Exception e) {
            voltageSensor = null;
        }

        // Initialize parameters
        syncParamsFromFields();
        copyArray(currentParams, bestParams);
        copyArray(currentParams, lastGoodParams);

        // Servo
        if (robot.transferServo != null) {
            robot.transferServo.setPosition(SERVO_IDLE);
        }

        startTime = System.currentTimeMillis();
        lastLoopTime = startTime;

        // Init telemetry
        telemetry.addLine("═══════════════════════════════════════");
        telemetry.addLine("   DepoTuner v5.0 - DRIVER FEEDBACK");
        telemetry.addLine("═══════════════════════════════════════");
        telemetry.addLine("");
        telemetry.addLine("After each shot, RATE IT:");
        telemetry.addLine("  GP2 D-Up    = GOOD");
        telemetry.addLine("  GP2 D-Down  = BAD");
        telemetry.addLine("  GP2 D-Left  = OK");
        telemetry.addLine("  GP2 D-Right = PERFECT");
        telemetry.addLine("");
        telemetry.addLine("GP1: Drive | LT=Shoot | LB=45% RB=70%");
        telemetry.addLine("GP2: X=Depo | A/B=Intake | LStick=Speed");
        telemetry.update();
    }

    private void configureMotors() {
        // Drive motors
        if (robot.frontLeft != null) robot.frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        if (robot.frontRight != null) robot.frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        if (robot.backLeft != null) robot.backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        if (robot.backRight != null) robot.backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Deposit motors - run with encoder for velocity control
        try {
            if (robot.deposit1 != null) {
                robot.deposit1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.deposit1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            }
            if (robot.deposit2 != null) {
                robot.deposit2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.deposit2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            }
        } catch (Exception ignored) {}
    }

    // ═══════════════════════════════════════════════════════════════
    //  MAIN LOOP
    // ═══════════════════════════════════════════════════════════════

    @Override
    public void loop() {
        long now = System.currentTimeMillis();
        double dt = (now - lastLoopTime) / 1000.0;
        lastLoopTime = now;
        loopCount++;

        // Get gamepad references
        gp1 = gamepad1;
        gp2 = gamepad2;

        // === EVERY LOOP ===
        handleDriver1();
        handleDriver2();
        handleDrive();
        handleIntakes();
        updateSpeedMode();

        // === PERIODIC (every 4 loops) ===
        if (loopCount % 4 == 0) {
            updateVoltage();

            if (depositOn) {
                updateVelocity(dt);
                runDepositControl(now);
            }
        }

        // === TELEMETRY (every 200ms) ===
        if (loopCount % 8 == 0) {
            if (exportMode) {
                showExportTelemetry();
            } else {
                showMainTelemetry();
            }
        }

        // Update previous states
        updatePrevStates();
    }

    // ═══════════════════════════════════════════════════════════════
    //  SPEED MODE (Driver 1 bumpers)
    // ═══════════════════════════════════════════════════════════════

    private void updateSpeedMode() {
        // Check bumpers DIRECTLY each loop
        if (gp1.left_bumper) {
            speedMode = SPEED_45;
            speedModeLabel = "45%";
        } else if (gp1.right_bumper) {
            speedMode = SPEED_70;
            speedModeLabel = "70%";
        } else {
            speedMode = SPEED_FULL;
            speedModeLabel = "100%";
        }

        // Calculate effective target
        effectiveTarget = baseTargetVelocity * speedMode;
    }

    // ═══════════════════════════════════════════════════════════════
    //  DRIVER 1 CONTROLS
    // ═══════════════════════════════════════════════════════════════

    private void handleDriver1() {
        // Left Trigger - Shoot
        boolean ltPressed = gp1.left_trigger > 0.5;
        boolean ltJustPressed = ltPressed && !prev_gp1_lt;
        boolean ltJustReleased = !ltPressed && prev_gp1_lt;

        if (ltJustPressed && depositOn) {
            startRecording();
            if (robot.transferServo != null) {
                robot.transferServo.setPosition(SERVO_SHOOT);
                servoFireTime = System.currentTimeMillis();
                predictiveActive = true;
            }
        }

        if (ltJustReleased) {
            if (recording) {
                endRecording();
            }
            if (robot.transferServo != null) {
                robot.transferServo.setPosition(SERVO_IDLE);
            }
            predictiveActive = false;
        }

        // Back - Reset
        boolean backJustPressed = gp1.back && !prev_gp1_back;
        if (backJustPressed) {
            resetAll();
            rumble(gp1, 500);
        }

        // Y+B - Export mode
        boolean ybPressed = gp1.y && gp1.b;
        boolean ybJustPressed = ybPressed && !prev_gp1_yb;
        if (ybJustPressed) {
            exportMode = !exportMode;
            javaFormat = false;
        }

        // D-pad in export mode - toggle format
        if (exportMode) {
            if ((gp1.dpad_left && !prev_gp1_dpadL) || (gp1.dpad_right && !prev_gp1_dpadR)) {
                javaFormat = !javaFormat;
            }
        }
    }

    // ═══════════════════════════════════════════════════════════════
    //  DRIVER 2 CONTROLS
    // ═══════════════════════════════════════════════════════════════

    private void handleDriver2() {
        // X - Toggle deposit
        if (gp2.x && !prev_gp2_x) {
            depositOn = !depositOn;
            if (!depositOn) {
                robot.stopDeposit();
                resetBoostState();
            }
        }

        // Left stick Y - Fine adjust target speed
        double stickY = -gp2.left_stick_y;
        if (Math.abs(stickY) > 0.1) {
            baseTargetVelocity += stickY * 2.0;  // Slow adjustment
            baseTargetVelocity = clamp(baseTargetVelocity, 200, 1200);
        }

        // === RATING SYSTEM ===
        if (awaitingRating) {
            // D-Pad UP - GOOD
            if (gp2.dpad_up && !prev_gp2_dpadU) {
                rateShot(Rating.GOOD);
            }
            // D-Pad DOWN - BAD
            if (gp2.dpad_down && !prev_gp2_dpadD) {
                rateShot(Rating.BAD);
            }
            // D-Pad LEFT - OK
            if (gp2.dpad_left && !prev_gp2_dpadL) {
                rateShot(Rating.OK);
            }
            // D-Pad RIGHT - PERFECT
            if (gp2.dpad_right && !prev_gp2_dpadR) {
                rateShot(Rating.PERFECT);
            }
        }

        // Left Bumper - Undo last rating
        if (gp2.left_bumper && !prev_gp2_lb) {
            undoLastRating();
        }

        // Right Bumper - Lock params
        if (gp2.right_bumper && !prev_gp2_rb) {
            paramsLocked = !paramsLocked;
            rumble(gp2, paramsLocked ? 300 : 100);
        }
    }

    // ═══════════════════════════════════════════════════════════════
    //  RATING SYSTEM
    // ═══════════════════════════════════════════════════════════════

    private enum Rating { PERFECT, GOOD, OK, BAD }

    private void rateShot(Rating rating) {
        awaitingRating = false;
        totalShots++;

        double score;
        switch (rating) {
            case PERFECT:
                score = 100;
                perfectShots++;
                lastRating = "★ PERFECT ★";
                // Strongly lock these params
                copyArray(currentParams, bestParams);
                copyArray(currentParams, lastGoodParams);
                explorationRate = Math.max(0.05, explorationRate * 0.5);
                rumble(gp2, 400);
                break;

            case GOOD:
                score = 75;
                goodShots++;
                lastRating = "✓ GOOD";
                // Lock these params
                copyArray(currentParams, lastGoodParams);
                if (score > bestScore) {
                    bestScore = score;
                    copyArray(currentParams, bestParams);
                }
                explorationRate = Math.max(0.1, explorationRate * 0.7);
                rumble(gp2, 200);
                break;

            case OK:
                score = 50;
                lastRating = "~ OK";
                // Slight improvement
                explorationRate = Math.max(0.15, explorationRate * 0.85);
                break;

            case BAD:
            default:
                score = 0;
                badShots++;
                lastRating = "✗ BAD";
                // Reject these params, go back to last good
                copyArray(lastGoodParams, currentParams);
                syncFieldsFromParams();
                explorationRate = Math.min(0.8, explorationRate * 1.3);
                rumble(gp2, 100);
                break;
        }

        // Update history
        scoreHistory[historyIndex] = score;
        historyIndex = (historyIndex + 1) % scoreHistory.length;
        if (historyCount < scoreHistory.length) historyCount++;

        // Check for convergence
        checkConvergence();
    }

    private void undoLastRating() {
        if (historyCount > 0) {
            historyCount--;
            historyIndex = (historyIndex - 1 + scoreHistory.length) % scoreHistory.length;
            lastRating = "(undone)";
            rumble(gp2, 50);
        }
    }

    private void checkConvergence() {
        if (historyCount < 5) return;

        // Calculate average recent score
        double sum = 0;
        int count = Math.min(historyCount, 10);
        for (int i = 0; i < count; i++) {
            int idx = (historyIndex - 1 - i + scoreHistory.length) % scoreHistory.length;
            sum += scoreHistory[idx];
        }
        double avgScore = sum / count;

        // If consistently good, reduce exploration
        if (avgScore >= 70) {
            explorationRate = Math.max(0.05, explorationRate * 0.9);
        }
        // If consistently bad, increase exploration
        else if (avgScore < 40) {
            explorationRate = Math.min(0.7, explorationRate * 1.1);
        }
    }

    // ═══════════════════════════════════════════════════════════════
    //  DRIVE
    // ═══════════════════════════════════════════════════════════════

    private void handleDrive() {
        double forward = -gp1.left_stick_y * DRIVE_SCALE;
        double strafe = gp1.left_stick_x * STRAFE_SCALE;
        double rotate = gp1.right_stick_x * ROTATE_SCALE;

        if (Math.abs(forward) < 0.05 && Math.abs(strafe) < 0.05 && Math.abs(rotate) < 0.05) {
            setDrivePowers(0, 0, 0, 0);
            return;
        }

        double fl = forward + strafe + rotate;
        double fr = forward - strafe - rotate;
        double bl = forward - strafe + rotate;
        double br = forward + strafe - rotate;

        double max = Math.max(1.0, Math.max(Math.max(Math.abs(fl), Math.abs(fr)),
                Math.max(Math.abs(bl), Math.abs(br))));

        setDrivePowers(fl/max, fr/max, bl/max, br/max);
    }

    private void setDrivePowers(double fl, double fr, double bl, double br) {
        if (robot.frontLeft != null) robot.frontLeft.setPower(fl);
        if (robot.frontRight != null) robot.frontRight.setPower(fr);
        if (robot.backLeft != null) robot.backLeft.setPower(bl);
        if (robot.backRight != null) robot.backRight.setPower(br);
    }

    // ═══════════════════════════════════════════════════════════════
    //  INTAKES
    // ═══════════════════════════════════════════════════════════════

    private void handleIntakes() {
        double power = 0;
        if (gp2.a) power = 1.0;
        else if (gp2.b) power = -1.0;

        if (robot.intake1 != null) robot.intake1.setPower(power);
        if (robot.intake2 != null) robot.intake2.setPower(-power);
    }

    // ═══════════════════════════════════════════════════════════════
    //  VOLTAGE COMPENSATION
    // ═══════════════════════════════════════════════════════════════

    private void updateVoltage() {
        long now = System.currentTimeMillis();
        if (now - lastVoltageRead < 500) return;
        lastVoltageRead = now;

        if (voltageSensor != null) {
            double v = voltageSensor.getVoltage();
            voltage = voltage * 0.7 + v * 0.3;  // Smooth
            voltageCompensation = NOMINAL_VOLTAGE / clamp(voltage, 10.0, 14.0);
        }
    }

    // ═══════════════════════════════════════════════════════════════
    //  VELOCITY MEASUREMENT (Standardized)
    // ═══════════════════════════════════════════════════════════════

    private void updateVelocity(double dt) {
        if (robot.deposit1 == null) return;

        // Get raw velocities
        double v1 = robot.getDeposit1Velocity();
        double v2 = robot.getDeposit2Velocity();
        rawVelocity = (v1 + v2) * 0.5;

        // Apply standardized filter (2nd order for stability)
        double alpha = 0.25;
        prevFilteredVelocity = filteredVelocity;
        filteredVelocity = filteredVelocity * (1 - alpha) + rawVelocity * alpha;

        // Calculate derivative (smoothed)
        if (dt > 0.001) {
            double rawDerivative = (filteredVelocity - prevFilteredVelocity) / dt;
            velocityDerivative = velocityDerivative * 0.6 + rawDerivative * 0.4;
        }
    }

    // ═══════════════════════════════════════════════════════════════
    //  DEPOSIT CONTROL (Standardized)
    // ═══════════════════════════════════════════════════════════════

    private void runDepositControl(long now) {
        if (robot.deposit1 == null) return;

        // Normalized error (as fraction of target)
        double error = (effectiveTarget - filteredVelocity) / effectiveTarget;
        double absError = Math.abs(error);

        // Normalized derivative
        double normDerivative = velocityDerivative / effectiveTarget;

        double targetBoost = 0;

        // === PREDICTIVE BOOST ===
        if (predictiveActive) {
            double elapsed = now - servoFireTime;
            if (elapsed < predictiveWindow) {
                double ramp = Math.min(1.0, elapsed / (predictiveWindow * 0.5));
                targetBoost = effectiveTarget * predictiveStrength * ramp;

                // Less for first ball
                if (detectedBalls == 0) {
                    targetBoost *= 0.5;
                }
            }
        }

        // === REACTIVE BOOST ===
        double triggerLevel = boostActive ? (triggerThreshold * 0.7) : triggerThreshold;

        if (error >= triggerLevel) {
            if (!boostActive) {
                boostActive = true;
                boostStartTime = now;
            }

            // Get ball-specific gain
            int ballIdx = Math.min(detectedBalls, 2);
            double gain = ballGain[ballIdx];
            double curve = ballCurve[ballIdx];

            // Calculate boost (normalized then scaled)
            double normBoost = gain * Math.pow(error / triggerThreshold, curve);

            // Cap at max ratio
            normBoost = Math.min(normBoost, maxBoostRatio);

            // Apply damping when approaching target
            if (absError < triggerThreshold * 2) {
                double dampT = absError / (triggerThreshold * 2);
                normBoost *= dampingFactor + (1 - dampingFactor) * dampT;
            }

            // Derivative damping (if recovering, reduce boost)
            if (normDerivative > 0) {
                normBoost -= derivativeGain * normDerivative * 10;
                normBoost = Math.max(0, normBoost);
            }

            // Time decay
            double duration = (now - boostStartTime) / 1000.0;
            normBoost *= Math.exp(-recoveryRate * duration * 15);

            // Scale to actual velocity
            double reactiveBoost = normBoost * effectiveTarget;
            targetBoost = Math.max(targetBoost, reactiveBoost);

        } else if (error < triggerThreshold * 0.25) {
            boostActive = false;
        }

        // === ANTI-OVERSHOOT ===
        if (filteredVelocity > effectiveTarget * 1.01) {
            targetBoost = 0;
            currentBoost *= 0.3;
            integralAccum *= 0.5;
        }

        // === RAMP BOOST ===
        double rampSpeed = (targetBoost > currentBoost) ? rampRate : 0.4;
        currentBoost += (targetBoost - currentBoost) * rampSpeed;
        currentBoost = clamp(currentBoost, 0, effectiveTarget * maxBoostRatio);

        // === INTEGRAL (conservative) ===
        if (absError < triggerThreshold * 0.5 && filteredVelocity <= effectiveTarget) {
            integralAccum += error * integralStrength;
            double maxIntegral = effectiveTarget * integralLimit;
            integralAccum = clamp(integralAccum, -maxIntegral, maxIntegral);
        } else if (filteredVelocity > effectiveTarget) {
            integralAccum = Math.min(0, integralAccum * 0.8);
        }

        // === CALCULATE COMMANDS ===
        double totalBoost = currentBoost + integralAccum;

        double cmd1 = (effectiveTarget + totalBoost) * voltageCompensation;
        double cmd2 = (effectiveTarget + totalBoost) * voltageCompensation;

        // Apply motor trims
        cmd1 *= (1.0 + motor1Trim);
        cmd2 *= (1.0 + motor2Trim);

        // === WRITE TO MOTORS (with caching) ===
        if (Math.abs(cmd1 - lastCmd1) > 2.0) {
            robot.deposit1.setVelocity(cmd1);
            lastCmd1 = cmd1;
        }
        if (Math.abs(cmd2 - lastCmd2) > 2.0) {
            robot.deposit2.setVelocity(cmd2);
            lastCmd2 = cmd2;
        }

        // === RECORD SAMPLES ===
        if (recording && sampleCount < MAX_SAMPLES) {
            sampleTimes[sampleCount] = now;
            sampleVels[sampleCount] = filteredVelocity;
            sampleCount++;

            // Track per-ball metrics
            if (sampleCount > WARMUP_SAMPLES && detectedBalls < 10) {
                double over = filteredVelocity - effectiveTarget;
                double under = effectiveTarget - filteredVelocity;

                if (over > ballPeakOver[detectedBalls]) {
                    ballPeakOver[detectedBalls] = over;
                }
                if (under > ballPeakUnder[detectedBalls]) {
                    ballPeakUnder[detectedBalls] = under;
                }
            }

            detectBallDrop();
        }
    }

    private void resetBoostState() {
        currentBoost = 0;
        integralAccum = 0;
        boostActive = false;
        predictiveActive = false;
    }

    // ═══════════════════════════════════════════════════════════════
    //  BALL DETECTION
    // ═══════════════════════════════════════════════════════════════

    private void detectBallDrop() {
        if (sampleCount < WARMUP_SAMPLES + 15) return;
        if (sampleCount - lastBallSample < MIN_BALL_GAP_SAMPLES) return;

        // Compare recent vs slightly older samples
        double older = 0, newer = 0;
        for (int i = 0; i < 5; i++) {
            older += sampleVels[sampleCount - 12 - i];
            newer += sampleVels[sampleCount - 2 - i];
        }
        older /= 5;
        newer /= 5;

        double drop = older - newer;
        double threshold = effectiveTarget * 0.08;  // 8% drop

        if (drop > threshold) {
            detectedBalls++;
            lastBallSample = sampleCount;

            // Initialize next ball tracking
            if (detectedBalls < 10) {
                ballPeakOver[detectedBalls] = 0;
                ballPeakUnder[detectedBalls] = 0;
            }
        }
    }

    // ═══════════════════════════════════════════════════════════════
    //  RECORDING SESSION
    // ═══════════════════════════════════════════════════════════════

    private void startRecording() {
        recording = true;
        sampleCount = 0;
        detectedBalls = 0;
        lastBallSample = 0;

        for (int i = 0; i < 10; i++) {
            ballPeakOver[i] = 0;
            ballPeakUnder[i] = 0;
            ballRecoveryTime[i] = 0;
        }

        resetBoostState();

        // Generate trial params if not locked
        if (!paramsLocked) {
            generateTrialParams();
            copyArray(trialParams, currentParams);
            syncFieldsFromParams();
        }
    }

    private void endRecording() {
        recording = false;
        predictiveActive = false;

        int validSamples = sampleCount - WARMUP_SAMPLES;
        if (validSamples < MIN_SAMPLES) {
            return;
        }

        // Analyze shot
        analyzeShot();

        // Mark awaiting rating
        awaitingRating = true;
        lastRating = "? RATE IT";
    }

    private void analyzeShot() {
        double maxOver = 0, maxUnder = 0;
        double sumSqErr = 0;
        int count = 0;

        for (int i = WARMUP_SAMPLES; i < sampleCount; i++) {
            double v = sampleVels[i];
            double err = v - effectiveTarget;
            sumSqErr += err * err;
            count++;

            if (v > effectiveTarget) {
                maxOver = Math.max(maxOver, v - effectiveTarget);
            } else {
                maxUnder = Math.max(maxUnder, effectiveTarget - v);
            }
        }

        lastShotMaxOver = maxOver;
        lastShotMaxUnder = maxUnder;
        lastShotStability = count > 0 ? 100 - Math.sqrt(sumSqErr / count) : 0;
        lastShotBalls = detectedBalls;
    }

    // ═══════════════════════════════════════════════════════════════
    //  PARAMETER GENERATION
    // ═══════════════════════════════════════════════════════════════

    private void generateTrialParams() {
        // Start from best known
        copyArray(bestParams, trialParams);

        // Apply perturbations based on exploration rate
        for (int i = 0; i < PARAM_COUNT; i++) {
            double noise = (random.nextDouble() * 2 - 1) * explorationRate;

            switch (i) {
                // Ball gains (0-2)
                case 0: case 1: case 2:
                    trialParams[i] = clamp(trialParams[i] * (1 + noise * 0.15), 0.1, 1.5);
                    break;
                // Ball curves (3-5)
                case 3: case 4: case 5:
                    trialParams[i] = clamp(trialParams[i] + noise * 0.15, 1.0, 1.8);
                    break;
                // Trigger threshold (6)
                case 6:
                    trialParams[i] = clamp(trialParams[i] + noise * 0.02, 0.03, 0.15);
                    break;
                // Max boost ratio (7)
                case 7:
                    trialParams[i] = clamp(trialParams[i] + noise * 0.05, 0.1, 0.5);
                    break;
                // Ramp rate (8)
                case 8:
                    trialParams[i] = clamp(trialParams[i] + noise * 0.08, 0.1, 0.5);
                    break;
                // Damping (9)
                case 9:
                    trialParams[i] = clamp(trialParams[i] + noise * 0.1, 0.4, 0.95);
                    break;
                // Derivative gain (10)
                case 10:
                    trialParams[i] = clamp(trialParams[i] + noise * 0.03, 0.02, 0.2);
                    break;
                // Recovery rate (11)
                case 11:
                    trialParams[i] = clamp(trialParams[i] + noise * 0.01, 0.01, 0.08);
                    break;
                // Predictive strength (12)
                case 12:
                    trialParams[i] = clamp(trialParams[i] + noise * 0.05, 0.0, 0.3);
                    break;
                // Predictive window (13)
                case 13:
                    trialParams[i] = clamp(trialParams[i] + noise * 20, 30, 150);
                    break;
                // Integral strength (14)
                case 14:
                    trialParams[i] = clamp(trialParams[i] + noise * 0.0003, 0.0, 0.002);
                    break;
                // Integral limit (15)
                case 15:
                    trialParams[i] = clamp(trialParams[i] + noise * 0.01, 0.01, 0.08);
                    break;
            }
        }
    }

    // ═══════════════════════════════════════════════════════════════
    //  PARAM SYNC
    // ═══════════════════════════════════════════════════════════════

    private void syncParamsFromFields() {
        currentParams[0] = ballGain[0];
        currentParams[1] = ballGain[1];
        currentParams[2] = ballGain[2];
        currentParams[3] = ballCurve[0];
        currentParams[4] = ballCurve[1];
        currentParams[5] = ballCurve[2];
        currentParams[6] = triggerThreshold;
        currentParams[7] = maxBoostRatio;
        currentParams[8] = rampRate;
        currentParams[9] = dampingFactor;
        currentParams[10] = derivativeGain;
        currentParams[11] = recoveryRate;
        currentParams[12] = predictiveStrength;
        currentParams[13] = predictiveWindow;
        currentParams[14] = integralStrength;
        currentParams[15] = integralLimit;
    }

    private void syncFieldsFromParams() {
        ballGain[0] = currentParams[0];
        ballGain[1] = currentParams[1];
        ballGain[2] = currentParams[2];
        ballCurve[0] = currentParams[3];
        ballCurve[1] = currentParams[4];
        ballCurve[2] = currentParams[5];
        triggerThreshold = currentParams[6];
        maxBoostRatio = currentParams[7];
        rampRate = currentParams[8];
        dampingFactor = currentParams[9];
        derivativeGain = currentParams[10];
        recoveryRate = currentParams[11];
        predictiveStrength = currentParams[12];
        predictiveWindow = currentParams[13];
        integralStrength = currentParams[14];
        integralLimit = currentParams[15];
    }

    // ═══════════════════════════════════════════════════════════════
    //  RESET
    // ═══════════════════════════════════════════════════════════════

    private void resetAll() {
        // Reset to conservative defaults
        ballGain[0] = 0.35; ballGain[1] = 0.55; ballGain[2] = 0.65;
        ballCurve[0] = 1.15; ballCurve[1] = 1.25; ballCurve[2] = 1.30;

        triggerThreshold = 0.08;
        maxBoostRatio = 0.25;
        rampRate = 0.20;
        dampingFactor = 0.70;
        derivativeGain = 0.08;
        recoveryRate = 0.03;

        predictiveStrength = 0.12;
        predictiveWindow = 70.0;

        integralStrength = 0.0008;
        integralLimit = 0.03;

        motor1Trim = 0;
        motor2Trim = 0;

        syncParamsFromFields();
        copyArray(currentParams, bestParams);
        copyArray(currentParams, lastGoodParams);

        bestScore = 0;
        explorationRate = 0.5;
        paramsLocked = false;

        totalShots = 0;
        goodShots = 0;
        perfectShots = 0;
        badShots = 0;
        historyCount = 0;
        historyIndex = 0;

        lastRating = "-";
        awaitingRating = false;
    }

    // ═══════════════════════════════════════════════════════════════
    //  UTILITIES
    // ═══════════════════════════════════════════════════════════════

    private void updatePrevStates() {
        prev_gp1_lt = gp1.left_trigger > 0.5;
        prev_gp1_back = gp1.back;
        prev_gp1_yb = gp1.y && gp1.b;
        prev_gp1_dpadL = gp1.dpad_left;
        prev_gp1_dpadR = gp1.dpad_right;

        prev_gp2_x = gp2.x;
        prev_gp2_dpadU = gp2.dpad_up;
        prev_gp2_dpadD = gp2.dpad_down;
        prev_gp2_dpadL = gp2.dpad_left;
        prev_gp2_dpadR = gp2.dpad_right;
        prev_gp2_lb = gp2.left_bumper;
        prev_gp2_rb = gp2.right_bumper;
    }

    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }

    private static void copyArray(double[] src, double[] dst) {
        System.arraycopy(src, 0, dst, 0, Math.min(src.length, dst.length));
    }

    private void rumble(Gamepad gp, int ms) {
        try { gp.rumble(ms); } catch (Exception ignored) {}
    }

    // ═══════════════════════════════════════════════════════════════
    //  TELEMETRY
    // ═══════════════════════════════════════════════════════════════

    private void showMainTelemetry() {
        long elapsed = (System.currentTimeMillis() - startTime) / 1000;

        telemetry.addLine("═══ DepoTuner v5.0 DRIVER FEEDBACK ═══");
        telemetry.addData("Time", "%d:%02d", elapsed / 60, elapsed % 60);

        // Status
        String status = depositOn ? (recording ? "●REC" : "▶ON") : "■OFF";
        telemetry.addData("Status", "%s | Target:%.0f [%s]", status, effectiveTarget, speedModeLabel);

        // Velocity
        if (depositOn) {
            double error = effectiveTarget - filteredVelocity;
            telemetry.addData("Velocity", "%.0f (err:%.0f) boost:%.0f",
                    filteredVelocity, error, currentBoost);
        }

        // Battery
        telemetry.addData("Battery", "%.1fV (comp:%.2f)", voltage, voltageCompensation);

        telemetry.addLine("");

        // === RATING SECTION ===
        if (awaitingRating) {
            telemetry.addLine("╔════════════════════════════════╗");
            telemetry.addLine("║     RATE THIS SHOT! (GP2)      ║");
            telemetry.addLine("║  D-Up=GOOD   D-Right=PERFECT   ║");
            telemetry.addLine("║  D-Left=OK   D-Down=BAD        ║");
            telemetry.addLine("╚════════════════════════════════╝");
            telemetry.addData("Balls", "%d", lastShotBalls);
            telemetry.addData("Over/Under", "+%.0f / -%.0f", lastShotMaxOver, lastShotMaxUnder);
            telemetry.addData("Stability", "%.0f%%", lastShotStability);
        } else {
            telemetry.addData("Last Rating", lastRating);
        }

        telemetry.addLine("");

        // Stats
        telemetry.addLine("─── STATS ───");
        telemetry.addData("Shots", "%d (Good:%d Perfect:%d Bad:%d)",
                totalShots, goodShots, perfectShots, badShots);
        telemetry.addData("Exploration", "%.0f%% %s",
                explorationRate * 100, paramsLocked ? "[LOCKED]" : "");

        telemetry.addLine("");

        // Controls reminder
        telemetry.addLine("─── CONTROLS ───");
        telemetry.addLine("GP1: LT=Shoot LB=45% RB=70%");
        telemetry.addLine("GP2: X=Depo A/B=Intake LStick=Speed");
        telemetry.addLine("GP2: LB=Undo RB=Lock Y+B=Export");

        telemetry.update();
    }

    private void showExportTelemetry() {
        telemetry.addLine("══════ EXPORT MODE ══════");
        telemetry.addLine("GP1 D-Pad = toggle format");
        telemetry.addLine("GP1 Y+B = exit");
        telemetry.addLine("");
        telemetry.addData("Format", javaFormat ? "JAVA" : "VALUES");
        telemetry.addLine("─────────────────────────");

        if (javaFormat) {
            telemetry.addLine("");
            telemetry.addLine("// Ball-specific gains");
            telemetry.addLine(String.format("double[] ballGain = {%.4f, %.4f, %.4f};",
                    bestParams[0], bestParams[1], bestParams[2]));
            telemetry.addLine(String.format("double[] ballCurve = {%.4f, %.4f, %.4f};",
                    bestParams[3], bestParams[4], bestParams[5]));
            telemetry.addLine("");
            telemetry.addLine("// Response tuning");
            telemetry.addLine(String.format("double triggerThreshold = %.4f;", bestParams[6]));
            telemetry.addLine(String.format("double maxBoostRatio = %.4f;", bestParams[7]));
            telemetry.addLine(String.format("double rampRate = %.4f;", bestParams[8]));
            telemetry.addLine(String.format("double dampingFactor = %.4f;", bestParams[9]));
            telemetry.addLine(String.format("double derivativeGain = %.4f;", bestParams[10]));
            telemetry.addLine(String.format("double recoveryRate = %.4f;", bestParams[11]));
            telemetry.addLine("");
            telemetry.addLine("// Predictive");
            telemetry.addLine(String.format("double predictiveStrength = %.4f;", bestParams[12]));
            telemetry.addLine(String.format("double predictiveWindow = %.1f;", bestParams[13]));
            telemetry.addLine("");
            telemetry.addLine("// Integral");
            telemetry.addLine(String.format("double integralStrength = %.6f;", bestParams[14]));
            telemetry.addLine(String.format("double integralLimit = %.4f;", bestParams[15]));
        } else {
            telemetry.addLine("");
            for (int i = 0; i < PARAM_COUNT; i++) {
                telemetry.addData("p" + i, "%.6f", bestParams[i]);
            }
        }

        telemetry.addLine("");
        telemetry.addData("Total Shots", "%d", totalShots);
        telemetry.addData("Good/Perfect", "%d / %d", goodShots, perfectShots);

        telemetry.update();
    }

    // ═══════════════════════════════════════════════════════════════
    //  STOP
    // ═══════════════════════════════════════════════════════════════

    @Override
    public void stop() {
        robot.stop();

        telemetry.clearAll();
        telemetry.addLine("══════════════════════════════");
        telemetry.addLine("   DepoTuner v5.0 STOPPED");
        telemetry.addLine("══════════════════════════════");
        telemetry.addLine("");
        telemetry.addData("Total Shots", "%d", totalShots);
        telemetry.addData("Good", "%d", goodShots);
        telemetry.addData("Perfect", "%d", perfectShots);
        telemetry.addData("Bad", "%d", badShots);
        telemetry.addLine("");
        telemetry.addLine("Use GP1 Y+B to export params");
        telemetry.addLine("before stopping next time!");
        telemetry.update();
    }
}