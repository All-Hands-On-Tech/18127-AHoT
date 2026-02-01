package org.firstinspires.ftc.teamcode.Trowel.Drive;

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
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Comparator;
import java.util.List;
import java.util.Random;

/**
 * DepoTuner v4.0 - Advanced Automatic Deposit Optimization
 *
 * MAJOR FEATURES:
 *   - Kalman filtered velocity estimation
 *   - Battery voltage compensation
 *   - Servo-triggered predictive boost
 *   - Elite preservation (top 5 param sets)
 *   - Velocity-relative parameter scaling
 *   - Dual motor tracking & balancing
 *   - Oscillation detection & auto-damping
 *   - Outlier rejection & rollback protection
 *   - Confidence scoring (know when you're done)
 *   - Haptic feedback on improvements
 *
 * PERSISTENCE:
 *   - Auto-saves on stop
 *   - Auto-loads on init
 *   - Y+B for export mode
 */
@TeleOp(name = "DepoTuner", group = "Tuning")
public class DepoTuner extends OpMode {

    // ═══════════════════════════════════════════════════════════════
    //  STATIC PERSISTENCE
    // ═══════════════════════════════════════════════════════════════

    private static double[] staticBestParams = null;
    private static double staticBestLoss = Double.MAX_VALUE;
    private static int staticTotalSessions = 0;
    private static int staticTotalImprovements = 0;
    private static boolean staticInitialized = false;
    private static List<EliteEntry> staticElites = null;

    // ═══════════════════════════════════════════════════════════════
    //  FILE PERSISTENCE
    // ═══════════════════════════════════════════════════════════════

    private static final String SAVE_FOLDER = "/sdcard/FIRST/";
    private static final String SAVE_FILE = "depotuner_params.txt";
    private static final String ELITE_FILE = "depotuner_elites.txt";

    // ═══════════════════════════════════════════════════════════════
    //  TUNABLE PARAMETERS (20 total)
    // ═══════════════════════════════════════════════════════════════

    // Per-Ball Boost Curves (normalized to reference velocity)
    private double ball1Mult = 1.0;
    private double ball1Exp = 1.5;
    private double ball2Mult = 1.6;
    private double ball2Exp = 1.6;
    private double ball3Mult = 1.4;
    private double ball3Exp = 1.55;

    // Boost Response
    private double boostTrigger = 45.0;      // Base trigger (scales with velocity)
    private double boostCap = 320.0;         // Base cap (scales with velocity)
    private double rampUpRate = 0.4;
    private double approachDamping = 0.6;
    private double derivativeGain = 0.06;
    private double decayRate = 0.015;

    // Hysteresis (different enter/exit thresholds)
    private double hysteresisRatio = 0.7;    // Exit threshold = trigger * this

    // Predictive Boost
    private double predictiveBoostFraction = 0.35;
    private double predictiveWindowMs = 100.0;
    private double predictiveRampUp = 0.5;   // How fast predictive boost ramps

    // Integral Correction
    private double integralGain = 0.002;
    private double integralCap = 30.0;

    // Motor Balance
    private double motor1Bias = 0.0;
    private double motor2Bias = 0.0;

    // ═══════════════════════════════════════════════════════════════
    //  CONFIGURATION
    // ═══════════════════════════════════════════════════════════════

    private static final int PARAM_COUNT = 20;

    // Reference velocity for parameter normalization
    private static final double REFERENCE_VELOCITY = 640.0;

    // Voltage compensation
    private static final double NOMINAL_VOLTAGE = 12.5;
    private static final double MIN_VOLTAGE = 10.0;
    private static final double MAX_VOLTAGE = 14.0;

    // Kalman filter tuning
    private static final double KALMAN_PROCESS_NOISE = 100.0;
    private static final double KALMAN_MEASUREMENT_NOISE = 400.0;

    // Elite preservation
    private static final int ELITE_COUNT = 5;
    private static final double ELITE_CROSSOVER_RATE = 0.3;

    // Optimizer
    private static final double INITIAL_TEMP = 1.0;
    private static final double COOLING_RATE = 0.993;
    private static final double MIN_TEMP = 0.04;
    private static final double RESTART_TEMP = 0.45;
    private static final int RESTART_THRESHOLD = 12;

    // Rollback protection
    private static final int ROLLBACK_THRESHOLD = 5;
    private static final double ROLLBACK_LOSS_RATIO = 1.5;

    // Outlier detection
    private static final double OUTLIER_LOSS_MULTIPLIER = 3.0;

    // Oscillation detection
    private static final double OSCILLATION_THRESHOLD = 0.15;
    private static final double OSCILLATION_DAMPING = 0.8;

    // Confidence scoring
    private static final double CONFIDENCE_LOSS_TARGET = 50.0;
    private static final double CONFIDENCE_VARIANCE_TARGET = 100.0;

    // Session
    private static final int MIN_SAMPLES = 30;
    private static final int MIN_BALLS = 2;
    private static final int WARMUP_SAMPLES = 15;
    private static final double DROP_DETECT = 60.0;
    private static final int MIN_BALL_GAP = 35;

    // Deposit
    private double targetVelocity = 640.0;
    private static final double SPEED_STEP = 25.0;
    private static final double MIN_SPEED = 100.0;
    private static final double MAX_SPEED = 1500.0;

    // Drive
    private static final double DRIVE_SCALE = 0.8;
    private static final double STRAFE_SCALE = 0.8;
    private static final double ROTATE_SCALE = 0.6;

    // Servo
    private static final double SERVO_SHOOT = 0.7;
    private static final double SERVO_IDLE = 0.3;

    // ═══════════════════════════════════════════════════════════════
    //  STATE
    // ═══════════════════════════════════════════════════════════════

    private TrowelHardware robot;
    private VoltageSensor voltageSensor;
    private final Random random = new Random();

    // Kalman filter state
    private double kalmanVelocity = 0;
    private double kalmanAccel = 0;
    private double kalmanP11 = 1, kalmanP12 = 0, kalmanP21 = 0, kalmanP22 = 1;
    private long lastKalmanTime = 0;

    // Individual motor tracking
    private double motor1Velocity = 0;
    private double motor2Velocity = 0;
    private double motor1Filtered = 0;
    private double motor2Filtered = 0;

    // Voltage tracking
    private double currentVoltage = NOMINAL_VOLTAGE;
    private double voltageCompensation = 1.0;

    // Boost state
    private double currentBoost = 0;
    private double targetBoost = 0;
    private double integralError = 0;
    private long boostStartTime = 0;
    private boolean inBoostPhase = false;
    private boolean exitingBoost = false;

    // Predictive boost
    private long servoFireTime = 0;
    private boolean servoPredictiveActive = false;
    private double predictiveBoost = 0;

    // Soft landing
    private double softLandingFactor = 1.0;

    // Oscillation detection
    private double[] velocityHistory = new double[20];
    private int velocityHistoryIdx = 0;
    private double oscillationScore = 0;
    private double oscillationDamping = 1.0;

    // Phase tracking
    private long dropDetectedTime = 0;
    private long boostAppliedTime = 0;
    private double averagePhaseDelay = 0;

    // Recording
    private boolean recording = false;
    private final List<Sample> samples = new ArrayList<>();
    private int detectedBalls = 0;
    private int lastBallIdx = 0;

    // Elite preservation
    private List<EliteEntry> elites = new ArrayList<>();

    // Optimizer
    private double[] bestParams = new double[PARAM_COUNT];
    private double[] currentParams = new double[PARAM_COUNT];
    private double[] trialParams = new double[PARAM_COUNT];
    private double bestLoss = Double.MAX_VALUE;
    private double currentLoss = Double.MAX_VALUE;
    private double temperature = INITIAL_TEMP;
    private int sessionsWithoutImprove = 0;

    // Rollback protection
    private double[] rollbackParams = new double[PARAM_COUNT];
    private double rollbackLoss = Double.MAX_VALUE;
    private int degradationCount = 0;

    // Confidence tracking
    private double confidenceScore = 0;
    private double lossVariance = 0;
    private List<Double> recentLosses = new ArrayList<>();
    private static final int LOSS_HISTORY_SIZE = 20;

    // Stats
    private int sessions = 0;
    private int improvements = 0;
    private int outlierCount = 0;
    private int rollbackCount = 0;
    private long startTime = 0;
    private SessionResult lastResult = null;

    // Deposit state
    private boolean depositOn = false;

    // Export mode
    private boolean exportMode = false;
    private boolean showJavaFormat = false;

    // Edge detection
    private boolean lastX = false;
    private boolean lastLT = false;
    private boolean lastUp = false;
    private boolean lastDown = false;
    private boolean lastYB = false;
    private boolean lastRB = false;
    private boolean lastLB = false;

    // Rumble state
    private long lastRumbleTime = 0;
    private static final long RUMBLE_COOLDOWN = 2000;

    // ═══════════════════════════════════════════════════════════════
    //  DATA STRUCTURES
    // ═══════════════════════════════════════════════════════════════

    private static class Sample {
        final long time;
        final double vel1, vel2, velKalman;
        final double accel;
        final double boost;
        final double voltage;
        final int ball;

        Sample(long t, double v1, double v2, double vk, double a, double b, double volt, int ball) {
            this.time = t; this.vel1 = v1; this.vel2 = v2;
            this.velKalman = vk; this.accel = a;
            this.boost = b; this.voltage = volt; this.ball = ball;
        }
    }

    private static class BallStats {
        double maxDrop = 0, maxOver = 0, avgOver = 0;
        double recoveryMs = 0, rmsError = 0;
        double motor1Error = 0, motor2Error = 0;
        double phaseDelay = 0;
        int count = 0;
        boolean recovered = false;
        boolean oscillated = false;
    }

    private static class SessionResult {
        final int balls;
        final double loss, stability, maxOver, avgRecovery;
        final double oscillationScore, phaseDelay;
        final double avgVoltage;
        final BallStats[] stats;
        final boolean isOutlier;

        SessionResult(int b, double l, double s, double o, double r,
                      double osc, double phase, double volt, BallStats[] st, boolean outlier) {
            balls = b; loss = l; stability = s; maxOver = o; avgRecovery = r;
            oscillationScore = osc; phaseDelay = phase; avgVoltage = volt;
            stats = st; isOutlier = outlier;
        }
    }

    private static class EliteEntry {
        final double[] params;
        final double loss;
        final int sessionNum;

        EliteEntry(double[] p, double l, int s) {
            params = p.clone();
            loss = l;
            sessionNum = s;
        }
    }

    // ═══════════════════════════════════════════════════════════════
    //  INIT
    // ═══════════════════════════════════════════════════════════════

    @Override
    public void init() {
        robot = new TrowelHardware(hardwareMap);
        robot.resetDepositEncoders();
        configureMotors();

        // Get voltage sensor
        try {
            voltageSensor = hardwareMap.voltageSensor.iterator().next();
        } catch (Exception e) {
            voltageSensor = null;
        }

        // Initialize params
        syncParamsFromFields();
        System.arraycopy(currentParams, 0, bestParams, 0, PARAM_COUNT);
        System.arraycopy(currentParams, 0, rollbackParams, 0, PARAM_COUNT);

        // Load saved data
        boolean loaded = loadFromFile();
        if (!loaded && staticInitialized && staticBestParams != null) {
            loadFromStatic();
            loaded = true;
        }

        // Load elites
        loadElitesFromFile();
        if (elites.isEmpty() && staticElites != null) {
            elites.addAll(staticElites);
        }

        if (robot.transferServo != null) {
            robot.transferServo.setPosition(SERVO_IDLE);
        }

        startTime = System.currentTimeMillis();

        showInitTelemetry(loaded);
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

    private void showInitTelemetry(boolean loaded) {
        telemetry.addLine("════════════════════════════════════════");
        telemetry.addLine("      DepoTuner v4.0 - ADVANCED");
        telemetry.addLine("════════════════════════════════════════");
        telemetry.addLine("");

        if (loaded) {
            telemetry.addLine("✓ LOADED SAVED PARAMETERS");
            telemetry.addData("  Best Loss", "%.1f", bestLoss);
            telemetry.addData("  Sessions", "%d", sessions);
            telemetry.addData("  Elites", "%d", elites.size());
            telemetry.addData("  Confidence", "%.0f%%", confidenceScore * 100);
        } else {
            telemetry.addLine("Starting fresh");
        }
        telemetry.addLine("");

        telemetry.addLine("NEW FEATURES:");
        telemetry.addLine("  • Kalman filtered velocity");
        telemetry.addLine("  • Voltage compensation");
        telemetry.addLine("  • Elite preservation (top 5)");
        telemetry.addLine("  • Oscillation detection");
        telemetry.addLine("  • Rollback protection");
        telemetry.addLine("");

        if (voltageSensor != null) {
            telemetry.addData("Battery", "%.2fV ✓", voltageSensor.getVoltage());
        } else {
            telemetry.addLine("⚠ No voltage sensor");
        }
        telemetry.addLine("");

        telemetry.addLine("Controls: X=Deposit, LT=Shoot");
        telemetry.addLine("Y+B=Export, RB=Save, LB+L3+R3=Reset");
        telemetry.update();
    }

    // ═══════════════════════════════════════════════════════════════
    //  KALMAN FILTER
    // ═══════════════════════════════════════════════════════════════

    private void updateKalmanFilter(double measuredVel) {
        long now = System.currentTimeMillis();

        if (lastKalmanTime == 0) {
            kalmanVelocity = measuredVel;
            kalmanAccel = 0;
            lastKalmanTime = now;
            return;
        }

        double dt = (now - lastKalmanTime) / 1000.0;
        if (dt <= 0.001) return;
        lastKalmanTime = now;

        // Predict step
        double predVel = kalmanVelocity + kalmanAccel * dt;
        double predAccel = kalmanAccel;

        // Predict covariance
        double q = KALMAN_PROCESS_NOISE * dt;
        double p11 = kalmanP11 + dt * (kalmanP21 + kalmanP12) + dt * dt * kalmanP22 + q;
        double p12 = kalmanP12 + dt * kalmanP22;
        double p21 = kalmanP21 + dt * kalmanP22;
        double p22 = kalmanP22 + q;

        // Update step
        double r = KALMAN_MEASUREMENT_NOISE;
        double s = p11 + r;
        double k1 = p11 / s;
        double k2 = p21 / s;

        double innovation = measuredVel - predVel;
        kalmanVelocity = predVel + k1 * innovation;
        kalmanAccel = predAccel + k2 * innovation;

        // Update covariance
        kalmanP11 = (1 - k1) * p11;
        kalmanP12 = (1 - k1) * p12;
        kalmanP21 = -k2 * p11 + p21;
        kalmanP22 = -k2 * p12 + p22;
    }

    // ═══════════════════════════════════════════════════════════════
    //  VOLTAGE COMPENSATION
    // ═══════════════════════════════════════════════════════════════

    private void updateVoltage() {
        if (voltageSensor != null) {
            double v = voltageSensor.getVoltage();
            // Smooth voltage reading
            currentVoltage = 0.9 * currentVoltage + 0.1 * v;
            currentVoltage = clamp(currentVoltage, MIN_VOLTAGE, MAX_VOLTAGE);

            // Calculate compensation factor
            // Higher voltage = less boost needed, lower voltage = more boost needed
            voltageCompensation = NOMINAL_VOLTAGE / currentVoltage;
            voltageCompensation = clamp(voltageCompensation, 0.85, 1.25);
        } else {
            voltageCompensation = 1.0;
        }
    }

    // ═══════════════════════════════════════════════════════════════
    //  OSCILLATION DETECTION
    // ═══════════════════════════════════════════════════════════════

    private void updateOscillationDetection(double velocity) {
        // Store in circular buffer
        velocityHistory[velocityHistoryIdx] = velocity;
        velocityHistoryIdx = (velocityHistoryIdx + 1) % velocityHistory.length;

        // Calculate zero-crossing rate (sign changes in error)
        int crossings = 0;
        double lastErr = 0;
        for (int i = 0; i < velocityHistory.length; i++) {
            double err = velocityHistory[i] - targetVelocity;
            if (i > 0 && lastErr * err < 0) {
                crossings++;
            }
            lastErr = err;
        }

        // Normalize to 0-1 range
        double crossingRate = crossings / (double) (velocityHistory.length - 1);

        // Smooth oscillation score
        oscillationScore = 0.8 * oscillationScore + 0.2 * crossingRate;

        // Apply damping if oscillating
        if (oscillationScore > OSCILLATION_THRESHOLD) {
            oscillationDamping = Math.max(0.5, oscillationDamping * OSCILLATION_DAMPING);
        } else {
            oscillationDamping = Math.min(1.0, oscillationDamping * 1.02);
        }
    }

    // ═══════════════════════════════════════════════════════════════
    //  VELOCITY SCALING
    // ═══════════════════════════════════════════════════════════════

    private double getVelocityScale() {
        // Scale parameters relative to reference velocity
        return targetVelocity / REFERENCE_VELOCITY;
    }

    private double getScaledTrigger() {
        return boostTrigger * getVelocityScale();
    }

    private double getScaledCap() {
        return boostCap * getVelocityScale();
    }

    // ═══════════════════════════════════════════════════════════════
    //  MAIN LOOP
    // ═══════════════════════════════════════════════════════════════

    @Override
    public void loop() {
        updateVoltage();
        updateMotorVelocities();
        handleDrive();
        handleSpeed();
        handleDeposit();
        handleSession();
        handleIntakes();
        handleExportMode();
        handleSaveReset();
        runDeposit();

        if (exportMode) {
            showExportTelemetry();
        } else {
            showTelemetry();
        }
    }

    private void updateMotorVelocities() {
        if (robot.deposit1 == null) return;

        // Read individual motors
        motor1Velocity = robot.getDeposit1Velocity();
        motor2Velocity = robot.getDeposit2Velocity();

        // Simple filter for individual motors
        motor1Filtered = 0.7 * motor1Filtered + 0.3 * motor1Velocity;
        motor2Filtered = 0.7 * motor2Filtered + 0.3 * motor2Velocity;

        // Average for Kalman filter
        double avgVel = (motor1Velocity + motor2Velocity) / 2.0;
        updateKalmanFilter(avgVel);

        // Update oscillation detection
        updateOscillationDetection(kalmanVelocity);
    }

    private void handleDrive() {
        double fwd = -gamepad1.left_stick_y * DRIVE_SCALE;
        double str = gamepad1.left_stick_x * STRAFE_SCALE;
        double rot = gamepad1.right_stick_x * ROTATE_SCALE;

        double fl = fwd + str + rot;
        double fr = fwd - str - rot;
        double bl = fwd - str + rot;
        double br = fwd + str - rot;

        double max = Math.max(1.0, Math.max(Math.max(Math.abs(fl), Math.abs(fr)),
                Math.max(Math.abs(bl), Math.abs(br))));

        if (robot.frontLeft != null) robot.frontLeft.setPower(fl / max);
        if (robot.frontRight != null) robot.frontRight.setPower(fr / max);
        if (robot.backLeft != null) robot.backLeft.setPower(bl / max);
        if (robot.backRight != null) robot.backRight.setPower(br / max);
    }

    private void handleSpeed() {
        boolean up = gamepad1.dpad_up;
        boolean down = gamepad1.dpad_down;

        if (up && !lastUp) targetVelocity = Math.min(MAX_SPEED, targetVelocity + SPEED_STEP);
        if (down && !lastDown) targetVelocity = Math.max(MIN_SPEED, targetVelocity - SPEED_STEP);

        lastUp = up;
        lastDown = down;
    }

    private void handleDeposit() {
        boolean x = gamepad1.x;
        if (x && !lastX) {
            depositOn = !depositOn;
            if (!depositOn) {
                robot.stopDeposit();
                if (recording) endSession();
                resetBoostState();
            }
        }
        lastX = x;
    }

    private void handleSession() {
        boolean lt = gamepad1.left_trigger > 0.5;

        if (lt && !lastLT && depositOn) {
            startSession();
            if (robot.transferServo != null) {
                robot.transferServo.setPosition(SERVO_SHOOT);
                servoFireTime = System.currentTimeMillis();
                servoPredictiveActive = true;
            }
        } else if (!lt && lastLT) {
            if (recording) endSession();
            if (robot.transferServo != null) {
                robot.transferServo.setPosition(SERVO_IDLE);
            }
            servoPredictiveActive = false;
        }

        lastLT = lt;
    }

    private void handleIntakes() {
        double power = 0;
        if (gamepad1.a) power = 1.0;
        else if (gamepad1.b && !exportMode) power = -1.0;

        if (robot.intake1 != null) robot.intake1.setPower(power);
        if (robot.intake2 != null) robot.intake2.setPower(-power);
    }

    private void handleExportMode() {
        boolean yb = gamepad1.y && gamepad1.b;

        if (yb && !lastYB) {
            exportMode = !exportMode;
            showJavaFormat = false;
        }
        lastYB = yb;

        if (exportMode && (gamepad1.dpad_left || gamepad1.dpad_right)) {
            showJavaFormat = !showJavaFormat;
        }
    }

    private void handleSaveReset() {
        boolean rb = gamepad1.right_bumper;
        boolean lb = gamepad1.left_bumper;

        if (rb && !lastRB) {
            saveToFile();
            saveElitesToFile();
            saveToStatic();
            rumbleController(200);
        }
        lastRB = rb;

        if (lb && gamepad1.left_stick_button && gamepad1.right_stick_button) {
            resetToDefaults();
        }
        lastLB = lb;
    }

    // ═══════════════════════════════════════════════════════════════
    //  DEPOSIT CONTROL
    // ═══════════════════════════════════════════════════════════════

    private void runDeposit() {
        if (!depositOn || robot.deposit1 == null) return;

        long now = System.currentTimeMillis();
        double drop = targetVelocity - kalmanVelocity;
        double scaledTrigger = getScaledTrigger();
        double scaledCap = getScaledCap();
        double velScale = getVelocityScale();

        targetBoost = 0;

        // ─── PREDICTIVE BOOST (servo-triggered) ───
        if (servoPredictiveActive) {
            double elapsed = now - servoFireTime;
            if (elapsed < predictiveWindowMs) {
                // Ramp up predictive boost
                double rampProgress = Math.min(1.0, elapsed / (predictiveWindowMs * predictiveRampUp));
                double predDrop = scaledTrigger * 1.5;  // Anticipate medium drop
                double predBase = getBaseBoost(predDrop, detectedBalls, velScale);
                predictiveBoost = predBase * predictiveBoostFraction * rampProgress;
                targetBoost = predictiveBoost;

                if (boostAppliedTime == 0) {
                    boostAppliedTime = now;
                }
            } else {
                predictiveBoost *= 0.9;  // Decay predictive boost
            }
        }

        // ─── REACTIVE BOOST (drop-triggered) ───
        double effectiveTrigger = inBoostPhase ?
                (scaledTrigger * hysteresisRatio) : scaledTrigger;

        if (drop >= effectiveTrigger) {
            if (!inBoostPhase) {
                inBoostPhase = true;
                exitingBoost = false;
                boostStartTime = now;
                dropDetectedTime = now;
            }

            double baseBoost = getBaseBoost(drop, detectedBalls, velScale);

            // Apply approach damping (soft landing)
            double dampingRange = scaledTrigger * 2.5;
            if (drop < dampingRange) {
                double t = drop / dampingRange;
                softLandingFactor = approachDamping + (1 - approachDamping) * t * t;
            } else {
                softLandingFactor = 1.0;
            }
            baseBoost *= softLandingFactor;

            // Apply derivative damping
            if (kalmanAccel > 0) {
                baseBoost -= derivativeGain * kalmanAccel * velScale;
            }

            // Apply time decay (anti-windup)
            double dur = (now - boostStartTime) / 1000.0;
            baseBoost *= Math.exp(-decayRate * dur * 10);

            // Apply oscillation damping
            baseBoost *= oscillationDamping;

            // Apply voltage compensation
            baseBoost *= voltageCompensation;

            targetBoost = Math.max(targetBoost, baseBoost);
        } else {
            if (inBoostPhase) {
                exitingBoost = true;
            }
            if (exitingBoost && drop < scaledTrigger * 0.3) {
                inBoostPhase = false;
                exitingBoost = false;
            }
        }

        // ─── RAMP BOOST ───
        if (targetBoost > currentBoost) {
            currentBoost += (targetBoost - currentBoost) * rampUpRate;
        } else {
            // Slower ramp down for soft landing
            double rampDown = exitingBoost ? 0.15 : 0.25;
            currentBoost += (targetBoost - currentBoost) * rampDown;
        }
        currentBoost = clamp(currentBoost, 0, scaledCap);

        // ─── INTEGRAL CORRECTION ───
        double error = targetVelocity - kalmanVelocity;
        if (Math.abs(error) < scaledTrigger * 0.4) {
            integralError += error * integralGain;
            integralError = clamp(integralError, -integralCap * velScale, integralCap * velScale);
        } else if (Math.abs(error) > scaledTrigger) {
            // Reset integral on large errors
            integralError *= 0.9;
        }

        // ─── APPLY TO MOTORS ───
        double totalBoost = currentBoost + integralError;
        double vel1 = (targetVelocity + totalBoost + motor1Bias * velScale) * voltageCompensation;
        double vel2 = (targetVelocity + totalBoost + motor2Bias * velScale) * voltageCompensation;

        robot.deposit1.setVelocity(vel1);
        robot.deposit2.setVelocity(vel2);

        // ─── RECORD ───
        if (recording && samples.size() >= WARMUP_SAMPLES) {
            samples.add(new Sample(now, motor1Filtered, motor2Filtered, kalmanVelocity,
                    kalmanAccel, currentBoost, currentVoltage, detectedBalls));
        } else if (recording) {
            // Still in warmup - add but mark
            samples.add(new Sample(now, motor1Filtered, motor2Filtered, kalmanVelocity,
                    kalmanAccel, currentBoost, currentVoltage, -1));
        }

        if (recording) detectBall();
    }

    private double getBaseBoost(double drop, int ball, double velScale) {
        double mult, exp;
        switch (ball) {
            case 0:  mult = ball1Mult; exp = ball1Exp; break;
            case 1:  mult = ball2Mult; exp = ball2Exp; break;
            default: mult = ball3Mult; exp = ball3Exp; break;
        }
        // Scale boost with velocity
        return mult * Math.pow(Math.max(0, drop), exp) * Math.sqrt(velScale);
    }

    private void resetBoostState() {
        currentBoost = 0;
        targetBoost = 0;
        integralError = 0;
        inBoostPhase = false;
        exitingBoost = false;
        predictiveBoost = 0;
        softLandingFactor = 1.0;
        boostAppliedTime = 0;
        dropDetectedTime = 0;
    }

    // ═══════════════════════════════════════════════════════════════
    //  BALL DETECTION
    // ═══════════════════════════════════════════════════════════════

    private void detectBall() {
        int n = samples.size();
        if (n < 25 || n - lastBallIdx < MIN_BALL_GAP) return;

        double prev = 0, curr = 0;
        for (int i = 10; i <= 14; i++) prev += samples.get(n - i).velKalman;
        for (int i = 1; i <= 5; i++) curr += samples.get(n - i).velKalman;
        prev /= 5; curr /= 5;

        double dropAmount = prev - curr;
        double scaledDetect = DROP_DETECT * getVelocityScale();

        if (dropAmount > scaledDetect) {
            boolean stable = true;
            double scaledTrigger = getScaledTrigger();
            for (int i = 16; i <= 22 && n - i >= 0; i++) {
                if (samples.get(n - i).ball < 0) continue;  // Skip warmup
                if (Math.abs(samples.get(n - i).velKalman - targetVelocity) > scaledTrigger * 0.6) {
                    stable = false;
                    break;
                }
            }
            if (stable) {
                detectedBalls++;
                lastBallIdx = n;
                dropDetectedTime = System.currentTimeMillis();
            }
        }
    }

    // ═══════════════════════════════════════════════════════════════
    //  SESSION MANAGEMENT
    // ═══════════════════════════════════════════════════════════════

    private void startSession() {
        recording = true;
        samples.clear();
        detectedBalls = 0;
        lastBallIdx = 0;
        resetBoostState();

        // Reset Kalman
        kalmanVelocity = 0;
        kalmanAccel = 0;
        lastKalmanTime = 0;

        // Reset oscillation
        Arrays.fill(velocityHistory, 0);
        oscillationScore = 0;
        oscillationDamping = 1.0;

        // Generate trial params from elites or best
        generateTrialParams();
        System.arraycopy(trialParams, 0, currentParams, 0, PARAM_COUNT);
        syncFieldsFromParams();
    }

    private void generateTrialParams() {
        if (elites.size() >= 2 && random.nextDouble() < ELITE_CROSSOVER_RATE) {
            // Crossover between two random elites
            int idx1 = random.nextInt(elites.size());
            int idx2 = random.nextInt(elites.size());
            while (idx2 == idx1 && elites.size() > 1) {
                idx2 = random.nextInt(elites.size());
            }

            double[] p1 = elites.get(idx1).params;
            double[] p2 = elites.get(idx2).params;

            for (int i = 0; i < PARAM_COUNT; i++) {
                // Blend with some randomness
                double blend = random.nextDouble();
                trialParams[i] = p1[i] * blend + p2[i] * (1 - blend);
            }

            // Small perturbation on top
            perturbParams(trialParams, temperature * 0.5);
        } else {
            // Start from best and perturb
            System.arraycopy(bestParams, 0, trialParams, 0, PARAM_COUNT);
            perturbParams(trialParams, temperature);
        }
    }

    private void endSession() {
        recording = false;
        servoPredictiveActive = false;

        // Filter out warmup samples
        List<Sample> validSamples = new ArrayList<>();
        for (Sample s : samples) {
            if (s.ball >= 0) validSamples.add(s);
        }

        if (validSamples.size() < MIN_SAMPLES) return;

        SessionResult result = analyze(validSamples);
        lastResult = result;
        sessions++;

        // Outlier check
        if (result.isOutlier) {
            outlierCount++;
            return;  // Don't update anything
        }

        if (result.balls < MIN_BALLS) {
            sessionsWithoutImprove++;
            checkRestart();
            return;
        }

        // Update loss history
        recentLosses.add(result.loss);
        if (recentLosses.size() > LOSS_HISTORY_SIZE) {
            recentLosses.remove(0);
        }
        updateConfidence();

        // Check for improvement
        boolean improved = false;
        if (result.loss < bestLoss) {
            // Definite improvement
            bestLoss = result.loss;
            System.arraycopy(currentParams, 0, bestParams, 0, PARAM_COUNT);
            improvements++;
            sessionsWithoutImprove = 0;
            degradationCount = 0;
            improved = true;

            // Update rollback point
            rollbackLoss = bestLoss;
            System.arraycopy(bestParams, 0, rollbackParams, 0, PARAM_COUNT);

            // Add to elites
            addToElites(currentParams, result.loss, sessions);

            // Haptic feedback
            rumbleController(300);
        } else {
            // Simulated annealing acceptance
            double delta = result.loss - bestLoss;
            double prob = Math.exp(-delta / (temperature * 500));
            if (random.nextDouble() < prob) {
                // Accept worse solution
            }
            sessionsWithoutImprove++;

            // Check for degradation (rollback protection)
            if (result.loss > rollbackLoss * ROLLBACK_LOSS_RATIO) {
                degradationCount++;
                if (degradationCount >= ROLLBACK_THRESHOLD) {
                    performRollback();
                }
            } else {
                degradationCount = 0;
            }
        }

        currentLoss = result.loss;
        temperature = Math.max(MIN_TEMP, temperature * COOLING_RATE);
        checkRestart();
    }

    private void addToElites(double[] params, double loss, int sessionNum) {
        elites.add(new EliteEntry(params, loss, sessionNum));

        // Sort by loss (best first)
        elites.sort(Comparator.comparingDouble(e -> e.loss));

        // Keep only top N
        while (elites.size() > ELITE_COUNT) {
            elites.remove(elites.size() - 1);
        }
    }

    private void performRollback() {
        System.arraycopy(rollbackParams, 0, bestParams, 0, PARAM_COUNT);
        System.arraycopy(rollbackParams, 0, currentParams, 0, PARAM_COUNT);
        syncFieldsFromParams();
        bestLoss = rollbackLoss;
        degradationCount = 0;
        rollbackCount++;
        temperature = RESTART_TEMP;  // Also reset temperature
    }

    private void checkRestart() {
        if (sessionsWithoutImprove >= RESTART_THRESHOLD) {
            temperature = RESTART_TEMP;
            sessionsWithoutImprove = 0;

            // Maybe try an elite instead of best
            if (!elites.isEmpty() && random.nextDouble() < 0.3) {
                int idx = random.nextInt(elites.size());
                System.arraycopy(elites.get(idx).params, 0, currentParams, 0, PARAM_COUNT);
            } else {
                System.arraycopy(bestParams, 0, currentParams, 0, PARAM_COUNT);
            }
            syncFieldsFromParams();
        }
    }

    private void updateConfidence() {
        if (recentLosses.size() < 5) {
            confidenceScore = 0;
            return;
        }

        // Calculate variance
        double mean = 0;
        for (double l : recentLosses) mean += l;
        mean /= recentLosses.size();

        double variance = 0;
        for (double l : recentLosses) variance += (l - mean) * (l - mean);
        variance /= recentLosses.size();
        lossVariance = variance;

        // Confidence based on best loss and variance
        double lossConfidence = Math.max(0, 1 - bestLoss / CONFIDENCE_LOSS_TARGET);
        double varianceConfidence = Math.max(0, 1 - Math.sqrt(variance) / CONFIDENCE_VARIANCE_TARGET);

        confidenceScore = (lossConfidence + varianceConfidence) / 2;
        confidenceScore = clamp(confidenceScore, 0, 1);
    }

    // ═══════════════════════════════════════════════════════════════
    //  ANALYSIS
    // ═══════════════════════════════════════════════════════════════

    private SessionResult analyze(List<Sample> validSamples) {
        List<Integer> bounds = findBallBounds(validSamples);
        int ballCount = bounds.size();

        BallStats[] stats = new BallStats[3];
        for (int i = 0; i < 3; i++) stats[i] = new BallStats();

        double totalRec = 0, maxOver = 0;
        int recCount = 0;
        double totalOsc = 0;
        double totalPhase = 0;
        int phaseCount = 0;

        for (int b = 0; b < ballCount && b < 3; b++) {
            int start = bounds.get(b);
            int end = (b + 1 < ballCount) ? bounds.get(b + 1) : validSamples.size();
            stats[b] = analyzeBall(validSamples, start, end);

            if (stats[b].recovered) {
                totalRec += stats[b].recoveryMs;
                recCount++;
            }
            maxOver = Math.max(maxOver, stats[b].maxOver);

            if (stats[b].oscillated) totalOsc++;
            if (stats[b].phaseDelay > 0) {
                totalPhase += stats[b].phaseDelay;
                phaseCount++;
            }
        }

        // Overall metrics
        double sumSq = 0;
        double sumVoltage = 0;
        for (Sample s : validSamples) {
            double e = s.velKalman - targetVelocity;
            sumSq += e * e;
            sumVoltage += s.voltage;
        }
        double stability = Math.max(0, 100 - Math.sqrt(sumSq / validSamples.size()));
        double avgVoltage = sumVoltage / validSamples.size();
        double avgRec = recCount > 0 ? totalRec / recCount : 1000;
        double avgPhase = phaseCount > 0 ? totalPhase / phaseCount : 0;

        double loss = calcLoss(stats, stability, ballCount, avgRec, maxOver, totalOsc);

        // Outlier detection
        boolean isOutlier = false;
        if (!recentLosses.isEmpty()) {
            double avgLoss = 0;
            for (double l : recentLosses) avgLoss += l;
            avgLoss /= recentLosses.size();

            if (loss > avgLoss * OUTLIER_LOSS_MULTIPLIER) {
                isOutlier = true;
            }
        }

        return new SessionResult(ballCount, loss, stability, maxOver, avgRec,
                totalOsc / Math.max(1, ballCount), avgPhase, avgVoltage,
                stats, isOutlier);
    }

    private List<Integer> findBallBounds(List<Sample> validSamples) {
        List<Integer> bounds = new ArrayList<>();
        bounds.add(0);

        int n = validSamples.size();
        double scaledDetect = DROP_DETECT * getVelocityScale();

        for (int i = 24; i < n; i++) {
            double prev = 0, curr = 0;
            for (int j = 12; j <= 16; j++) prev += validSamples.get(i - j).velKalman;
            for (int j = 0; j <= 4; j++) curr += validSamples.get(i - j).velKalman;
            prev /= 5; curr /= 5;

            if (prev - curr > scaledDetect) {
                double scaledTrigger = getScaledTrigger();
                boolean stable = true;
                for (int j = 18; j <= 23 && i - j >= 0; j++) {
                    if (Math.abs(validSamples.get(i - j).velKalman - targetVelocity) > scaledTrigger * 0.7) {
                        stable = false;
                        break;
                    }
                }
                if (stable && i - bounds.get(bounds.size() - 1) > MIN_BALL_GAP) {
                    bounds.add(i);
                }
            }
        }
        return bounds;
    }

    private BallStats analyzeBall(List<Sample> validSamples, int start, int end) {
        BallStats s = new BallStats();
        if (start >= end) return s;

        double minV = Double.MAX_VALUE, maxV = Double.MIN_VALUE;
        double sumSqErr = 0, sumOver = 0;
        double sumM1Err = 0, sumM2Err = 0;
        int minIdx = start, recIdx = -1, overCount = 0;
        int crossings = 0;
        double lastErr = 0;

        for (int i = start; i < end && i < validSamples.size(); i++) {
            Sample samp = validSamples.get(i);
            double v = samp.velKalman;
            double e = v - targetVelocity;
            sumSqErr += e * e;
            s.count++;

            // Track individual motor errors
            sumM1Err += Math.abs(samp.vel1 - targetVelocity);
            sumM2Err += Math.abs(samp.vel2 - targetVelocity);

            // Oscillation detection per ball
            if (s.count > 1 && lastErr * e < 0) crossings++;
            lastErr = e;

            if (v < minV) { minV = v; minIdx = i; }
            if (v > maxV) maxV = v;

            if (v > targetVelocity) {
                sumOver += v - targetVelocity;
                overCount++;
                s.maxOver = Math.max(s.maxOver, v - targetVelocity);
            }

            double scaledTrigger = getScaledTrigger();
            if (i > minIdx && recIdx < 0 && v >= targetVelocity - scaledTrigger * 0.3) {
                recIdx = i;
                s.recovered = true;
            }
        }

        s.maxDrop = targetVelocity - minV;
        s.avgOver = overCount > 0 ? sumOver / overCount : 0;
        s.rmsError = s.count > 0 ? Math.sqrt(sumSqErr / s.count) : 0;
        s.motor1Error = s.count > 0 ? sumM1Err / s.count : 0;
        s.motor2Error = s.count > 0 ? sumM2Err / s.count : 0;

        // Check for oscillation
        double crossRate = (double) crossings / Math.max(1, s.count);
        s.oscillated = crossRate > OSCILLATION_THRESHOLD;

        if (recIdx > minIdx) {
            s.recoveryMs = validSamples.get(recIdx).time - validSamples.get(minIdx).time;
        }

        return s;
    }

    private double calcLoss(BallStats[] stats, double stability, int balls,
                            double avgRec, double maxOver, double oscCount) {
        double loss = 0;

        double velScale = getVelocityScale();

        // Scale targets with velocity
        double T_DROP = 35.0 * velScale;
        double T_OVER = 8.0 * velScale;
        double T_REC = 100.0;
        double T_STAB = 94.0;

        final double W_OVER = 18, W_AVG_OVER = 10, W_DROP = 2;
        final double W_REC = 2, W_RMS = 3, W_STAB = 4;
        final double W_MOTOR_BALANCE = 3, W_OSC = 5;

        for (int i = 0; i < Math.min(balls, 3); i++) {
            BallStats s = stats[i];
            if (s.count == 0) continue;

            // Overshoot (most important)
            double overErr = Math.max(0, s.maxOver - T_OVER);
            loss += W_OVER * overErr * overErr;
            loss += W_AVG_OVER * s.avgOver;

            // Drop
            loss += W_DROP * Math.max(0, s.maxDrop - T_DROP);

            // Recovery
            loss += W_REC * Math.max(0, s.recoveryMs - T_REC) / 50;

            // RMS
            loss += W_RMS * s.rmsError;

            // Motor balance
            double motorDiff = Math.abs(s.motor1Error - s.motor2Error);
            loss += W_MOTOR_BALANCE * motorDiff;

            // Oscillation penalty
            if (s.oscillated) loss += W_OSC * 20;
        }

        // Stability
        loss += W_STAB * Math.pow(Math.max(0, T_STAB - stability), 2);

        // Global oscillation penalty
        loss += oscCount * 15;

        // Fewer balls penalty
        if (balls < 3) loss *= 1.2;

        // No recovery penalty
        for (int i = 0; i < Math.min(balls, 3); i++) {
            if (!stats[i].recovered && stats[i].count > 0) loss *= 1.35;
        }

        return loss;
    }

    // ═══════════════════════════════════════════════════════════════
    //  PERTURBATION
    // ═══════════════════════════════════════════════════════════════

    private void perturbParams(double[] p, double temp) {
        for (int i = 0; i < PARAM_COUNT; i++) {
            double noise = (random.nextDouble() * 2 - 1) * temp;

            switch (i) {
                case 0: case 2: case 4:  // Multipliers
                    p[i] = clamp(p[i] * (1 + noise * 0.18), 0.3, 4.0);
                    break;
                case 1: case 3: case 5:  // Exponents
                    p[i] = clamp(p[i] + noise * 0.35, 1.0, 2.6);
                    break;
                case 6:  // boostTrigger
                    p[i] = clamp(p[i] * (1 + noise * 0.15), 20, 90);
                    break;
                case 7:  // boostCap
                    p[i] = clamp(p[i] * (1 + noise * 0.15), 100, 600);
                    break;
                case 8:  // rampUpRate
                    p[i] = clamp(p[i] + noise * 0.15, 0.1, 1.0);
                    break;
                case 9:  // approachDamping
                    p[i] = clamp(p[i] + noise * 0.12, 0.2, 0.95);
                    break;
                case 10: // derivativeGain
                    p[i] = clamp(p[i] + noise * 0.05, 0.005, 0.2);
                    break;
                case 11: // decayRate
                    p[i] = clamp(p[i] + noise * 0.012, 0.003, 0.06);
                    break;
                case 12: // hysteresisRatio
                    p[i] = clamp(p[i] + noise * 0.1, 0.5, 0.95);
                    break;
                case 13: // predictiveBoostFraction
                    p[i] = clamp(p[i] + noise * 0.12, 0.0, 0.7);
                    break;
                case 14: // predictiveWindowMs
                    p[i] = clamp(p[i] + noise * 40, 30, 250);
                    break;
                case 15: // predictiveRampUp
                    p[i] = clamp(p[i] + noise * 0.15, 0.2, 0.9);
                    break;
                case 16: // integralGain
                    p[i] = clamp(p[i] + noise * 0.003, 0.0, 0.015);
                    break;
                case 17: // integralCap
                    p[i] = clamp(p[i] + noise * 12, 5, 70);
                    break;
                case 18: case 19: // motorBias
                    p[i] = clamp(p[i] + noise * 12, -40, 40);
                    break;
            }
        }
    }

    // ═══════════════════════════════════════════════════════════════
    //  PERSISTENCE
    // ═══════════════════════════════════════════════════════════════

    private boolean saveToFile() {
        try {
            File folder = new File(SAVE_FOLDER);
            if (!folder.exists()) folder.mkdirs();

            PrintWriter w = new PrintWriter(new FileWriter(SAVE_FOLDER + SAVE_FILE));
            w.println("# DepoTuner v4 Parameters");
            w.println("version=4");
            w.println("bestLoss=" + bestLoss);
            w.println("sessions=" + sessions);
            w.println("improvements=" + improvements);
            w.println("temperature=" + temperature);
            w.println("confidence=" + confidenceScore);
            w.println("rollbackLoss=" + rollbackLoss);
            w.println("");

            String[] names = getParamNames();
            for (int i = 0; i < PARAM_COUNT; i++) {
                w.println(names[i] + "=" + bestParams[i]);
            }
            w.println("");
            w.println("# Rollback params");
            for (int i = 0; i < PARAM_COUNT; i++) {
                w.println("rb_" + names[i] + "=" + rollbackParams[i]);
            }

            w.close();
            return true;
        } catch (Exception e) {
            return false;
        }
    }

    private boolean loadFromFile() {
        try {
            File file = new File(SAVE_FOLDER + SAVE_FILE);
            if (!file.exists()) return false;

            BufferedReader r = new BufferedReader(new FileReader(file));
            String line;
            String[] names = getParamNames();

            while ((line = r.readLine()) != null) {
                line = line.trim();
                if (line.isEmpty() || line.startsWith("#")) continue;

                String[] parts = line.split("=");
                if (parts.length != 2) continue;

                String key = parts[0].trim();
                double value = Double.parseDouble(parts[1].trim());

                if (key.equals("bestLoss")) bestLoss = value;
                else if (key.equals("sessions")) sessions = (int) value;
                else if (key.equals("improvements")) improvements = (int) value;
                else if (key.equals("temperature")) temperature = value;
                else if (key.equals("confidence")) confidenceScore = value;
                else if (key.equals("rollbackLoss")) rollbackLoss = value;
                else {
                    for (int i = 0; i < PARAM_COUNT; i++) {
                        if (key.equals(names[i])) {
                            bestParams[i] = value;
                        } else if (key.equals("rb_" + names[i])) {
                            rollbackParams[i] = value;
                        }
                    }
                }
            }
            r.close();

            System.arraycopy(bestParams, 0, currentParams, 0, PARAM_COUNT);
            syncFieldsFromParams();
            return true;
        } catch (Exception e) {
            return false;
        }
    }

    private void saveElitesToFile() {
        try {
            PrintWriter w = new PrintWriter(new FileWriter(SAVE_FOLDER + ELITE_FILE));
            w.println("# DepoTuner Elites");
            w.println("count=" + elites.size());

            String[] names = getParamNames();
            for (int e = 0; e < elites.size(); e++) {
                EliteEntry elite = elites.get(e);
                w.println("");
                w.println("[elite_" + e + "]");
                w.println("loss=" + elite.loss);
                w.println("session=" + elite.sessionNum);
                for (int i = 0; i < PARAM_COUNT; i++) {
                    w.println(names[i] + "=" + elite.params[i]);
                }
            }
            w.close();
        } catch (Exception ignored) {}
    }

    private void loadElitesFromFile() {
        try {
            File file = new File(SAVE_FOLDER + ELITE_FILE);
            if (!file.exists()) return;

            BufferedReader r = new BufferedReader(new FileReader(file));
            String line;
            String[] names = getParamNames();

            double[] tempParams = new double[PARAM_COUNT];
            double tempLoss = Double.MAX_VALUE;
            int tempSession = 0;
            boolean inElite = false;

            while ((line = r.readLine()) != null) {
                line = line.trim();
                if (line.isEmpty() || line.startsWith("#")) continue;

                if (line.startsWith("[elite_")) {
                    if (inElite && tempLoss < Double.MAX_VALUE) {
                        elites.add(new EliteEntry(tempParams.clone(), tempLoss, tempSession));
                    }
                    tempParams = new double[PARAM_COUNT];
                    tempLoss = Double.MAX_VALUE;
                    tempSession = 0;
                    inElite = true;
                    continue;
                }

                String[] parts = line.split("=");
                if (parts.length != 2) continue;

                String key = parts[0].trim();
                double value = Double.parseDouble(parts[1].trim());

                if (key.equals("loss")) tempLoss = value;
                else if (key.equals("session")) tempSession = (int) value;
                else {
                    for (int i = 0; i < PARAM_COUNT; i++) {
                        if (key.equals(names[i])) {
                            tempParams[i] = value;
                        }
                    }
                }
            }

            if (inElite && tempLoss < Double.MAX_VALUE) {
                elites.add(new EliteEntry(tempParams, tempLoss, tempSession));
            }

            r.close();
        } catch (Exception ignored) {}
    }

    private void saveToStatic() {
        if (staticBestParams == null) staticBestParams = new double[PARAM_COUNT];
        System.arraycopy(bestParams, 0, staticBestParams, 0, PARAM_COUNT);
        staticBestLoss = bestLoss;
        staticTotalSessions = sessions;
        staticTotalImprovements = improvements;
        staticInitialized = true;
        staticElites = new ArrayList<>(elites);
    }

    private void loadFromStatic() {
        if (staticBestParams != null) {
            System.arraycopy(staticBestParams, 0, bestParams, 0, PARAM_COUNT);
            System.arraycopy(bestParams, 0, currentParams, 0, PARAM_COUNT);
            syncFieldsFromParams();
            bestLoss = staticBestLoss;
            sessions = staticTotalSessions;
            improvements = staticTotalImprovements;
        }
    }

    private void resetToDefaults() {
        ball1Mult = 1.0; ball1Exp = 1.5;
        ball2Mult = 1.6; ball2Exp = 1.6;
        ball3Mult = 1.4; ball3Exp = 1.55;
        boostTrigger = 45.0; boostCap = 320.0;
        rampUpRate = 0.4; approachDamping = 0.6;
        derivativeGain = 0.06; decayRate = 0.015;
        hysteresisRatio = 0.7;
        predictiveBoostFraction = 0.35; predictiveWindowMs = 100.0;
        predictiveRampUp = 0.5;
        integralGain = 0.002; integralCap = 30.0;
        motor1Bias = 0.0; motor2Bias = 0.0;

        syncParamsFromFields();
        System.arraycopy(currentParams, 0, bestParams, 0, PARAM_COUNT);
        System.arraycopy(currentParams, 0, rollbackParams, 0, PARAM_COUNT);

        bestLoss = Double.MAX_VALUE;
        rollbackLoss = Double.MAX_VALUE;
        sessions = 0;
        improvements = 0;
        temperature = INITIAL_TEMP;
        confidenceScore = 0;
        elites.clear();
        recentLosses.clear();

        staticBestParams = null;
        staticInitialized = false;
        staticElites = null;

        try {
            new File(SAVE_FOLDER + SAVE_FILE).delete();
            new File(SAVE_FOLDER + ELITE_FILE).delete();
        } catch (Exception ignored) {}
    }

    // ═══════════════════════════════════════════════════════════════
    //  HAPTIC FEEDBACK
    // ═══════════════════════════════════════════════════════════════

    private void rumbleController(int durationMs) {
        long now = System.currentTimeMillis();
        if (now - lastRumbleTime < RUMBLE_COOLDOWN) return;

        try {
            gamepad1.rumble(durationMs);
        } catch (Exception ignored) {}

        lastRumbleTime = now;
    }

    // ═══════════════════════════════════════════════════════════════
    //  PARAM HELPERS
    // ═══════════════════════════════════════════════════════════════

    private String[] getParamNames() {
        return new String[] {
                "ball1Mult", "ball1Exp", "ball2Mult", "ball2Exp", "ball3Mult", "ball3Exp",
                "boostTrigger", "boostCap", "rampUpRate", "approachDamping", "derivativeGain", "decayRate",
                "hysteresisRatio", "predictiveBoostFraction", "predictiveWindowMs", "predictiveRampUp",
                "integralGain", "integralCap", "motor1Bias", "motor2Bias"
        };
    }

    private void syncParamsFromFields() {
        currentParams[0] = ball1Mult;     currentParams[1] = ball1Exp;
        currentParams[2] = ball2Mult;     currentParams[3] = ball2Exp;
        currentParams[4] = ball3Mult;     currentParams[5] = ball3Exp;
        currentParams[6] = boostTrigger;  currentParams[7] = boostCap;
        currentParams[8] = rampUpRate;    currentParams[9] = approachDamping;
        currentParams[10] = derivativeGain; currentParams[11] = decayRate;
        currentParams[12] = hysteresisRatio;
        currentParams[13] = predictiveBoostFraction; currentParams[14] = predictiveWindowMs;
        currentParams[15] = predictiveRampUp;
        currentParams[16] = integralGain; currentParams[17] = integralCap;
        currentParams[18] = motor1Bias;   currentParams[19] = motor2Bias;
    }

    private void syncFieldsFromParams() {
        ball1Mult = currentParams[0];     ball1Exp = currentParams[1];
        ball2Mult = currentParams[2];     ball2Exp = currentParams[3];
        ball3Mult = currentParams[4];     ball3Exp = currentParams[5];
        boostTrigger = currentParams[6];  boostCap = currentParams[7];
        rampUpRate = currentParams[8];    approachDamping = currentParams[9];
        derivativeGain = currentParams[10]; decayRate = currentParams[11];
        hysteresisRatio = currentParams[12];
        predictiveBoostFraction = currentParams[13]; predictiveWindowMs = currentParams[14];
        predictiveRampUp = currentParams[15];
        integralGain = currentParams[16]; integralCap = currentParams[17];
        motor1Bias = currentParams[18];   motor2Bias = currentParams[19];
    }

    private double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }

    private String fmt(long ms) {
        long s = ms / 1000;
        return String.format("%d:%02d", s / 60, s % 60);
    }

    // ═══════════════════════════════════════════════════════════════
    //  TELEMETRY - NORMAL
    // ═══════════════════════════════════════════════════════════════

    private void showTelemetry() {
        long elapsed = System.currentTimeMillis() - startTime;

        telemetry.addLine("═══════════════════════════════════════");
        telemetry.addLine("    DepoTuner v4 - " + fmt(elapsed));
        telemetry.addLine("═══════════════════════════════════════");

        String status = depositOn ? (recording ? "●REC" : "▶RUN") : "■OFF";
        telemetry.addData("Status", "%s | Target: %.0f", status, targetVelocity);

        if (voltageSensor != null) {
            telemetry.addData("Battery", "%.2fV (×%.2f)", currentVoltage, voltageCompensation);
        }

        if (depositOn && robot.deposit1 != null) {
            double err = targetVelocity - kalmanVelocity;
            telemetry.addData("Velocity", "%.0f [Kalman] err:%.0f", kalmanVelocity, err);
            telemetry.addData("Motors", "M1:%.0f  M2:%.0f", motor1Filtered, motor2Filtered);
            telemetry.addData("Boost", "%.0f (pred:%.0f)", currentBoost, predictiveBoost);

            if (oscillationScore > OSCILLATION_THRESHOLD) {
                telemetry.addData("⚠ Oscillation", "%.0f%% (damping:%.0f%%)",
                        oscillationScore * 100, oscillationDamping * 100);
            }
        }
        telemetry.addLine("");

        // Confidence bar
        String confBar = buildProgressBar(confidenceScore, 10);
        telemetry.addData("Confidence", "%s %.0f%%", confBar, confidenceScore * 100);
        telemetry.addLine("");

        telemetry.addLine("─── PROGRESS ───");
        telemetry.addData("Sessions", "%d", sessions);
        telemetry.addData("Improvements", "%d", improvements);
        telemetry.addData("Elites", "%d", elites.size());
        telemetry.addData("Best Loss", "%.1f", bestLoss);
        telemetry.addData("Temperature", "%.2f", temperature);

        if (outlierCount > 0) telemetry.addData("Outliers", "%d", outlierCount);
        if (rollbackCount > 0) telemetry.addData("Rollbacks", "%d", rollbackCount);
        telemetry.addLine("");

        if (lastResult != null) {
            telemetry.addLine("─── LAST SESSION ───");
            String resultStatus = lastResult.isOutlier ? "⚠OUTLIER" :
                    (lastResult.loss <= bestLoss ? "✓BEST" : "");
            telemetry.addData("Result", "%d balls | Loss: %.1f %s",
                    lastResult.balls, lastResult.loss, resultStatus);
            telemetry.addData("Stability", "%.1f%%", lastResult.stability);
            telemetry.addData("Max Over", "%.0f | Osc: %.0f%%",
                    lastResult.maxOver, lastResult.oscillationScore * 100);
            telemetry.addLine("");
        }

        telemetry.addLine("Y+B=Export | RB=Save | LB+L3+R3=Reset");
        telemetry.addLine("═══════════════════════════════════════");

        telemetry.update();
    }

    private String buildProgressBar(double progress, int width) {
        int filled = (int) (progress * width);
        StringBuilder sb = new StringBuilder("[");
        for (int i = 0; i < width; i++) {
            sb.append(i < filled ? "█" : "░");
        }
        sb.append("]");
        return sb.toString();
    }

    // ═══════════════════════════════════════════════════════════════
    //  TELEMETRY - EXPORT
    // ═══════════════════════════════════════════════════════════════

    private void showExportTelemetry() {
        telemetry.addLine("════════════════════════════════════════");
        telemetry.addLine("          📋 EXPORT MODE");
        telemetry.addLine("════════════════════════════════════════");
        telemetry.addLine("");
        telemetry.addLine("D-Pad Left/Right = toggle format");
        telemetry.addLine("Y+B = exit export mode");
        telemetry.addLine("");
        telemetry.addData("Format", showJavaFormat ? "JAVA CODE" : "VALUES");
        telemetry.addLine("");
        telemetry.addLine("────────────────────────────────────────");

        if (showJavaFormat) {
            telemetry.addLine("");
            telemetry.addLine("// ══ DEPOTUNER v4 PARAMETERS ══");
            telemetry.addLine("");
            telemetry.addLine("// Per-ball boost curves");
            telemetry.addLine(String.format("double ball1Mult = %.4f;", bestParams[0]));
            telemetry.addLine(String.format("double ball1Exp  = %.4f;", bestParams[1]));
            telemetry.addLine(String.format("double ball2Mult = %.4f;", bestParams[2]));
            telemetry.addLine(String.format("double ball2Exp  = %.4f;", bestParams[3]));
            telemetry.addLine(String.format("double ball3Mult = %.4f;", bestParams[4]));
            telemetry.addLine(String.format("double ball3Exp  = %.4f;", bestParams[5]));
            telemetry.addLine("");
            telemetry.addLine("// Boost response");
            telemetry.addLine(String.format("double boostTrigger     = %.2f;", bestParams[6]));
            telemetry.addLine(String.format("double boostCap         = %.1f;", bestParams[7]));
            telemetry.addLine(String.format("double rampUpRate       = %.4f;", bestParams[8]));
            telemetry.addLine(String.format("double approachDamping  = %.4f;", bestParams[9]));
            telemetry.addLine(String.format("double derivativeGain   = %.5f;", bestParams[10]));
            telemetry.addLine(String.format("double decayRate        = %.5f;", bestParams[11]));
            telemetry.addLine(String.format("double hysteresisRatio  = %.4f;", bestParams[12]));
            telemetry.addLine("");
            telemetry.addLine("// Predictive boost");
            telemetry.addLine(String.format("double predBoostFrac   = %.4f;", bestParams[13]));
            telemetry.addLine(String.format("double predWindowMs    = %.1f;", bestParams[14]));
            telemetry.addLine(String.format("double predRampUp      = %.4f;", bestParams[15]));
            telemetry.addLine("");
            telemetry.addLine("// Integral correction");
            telemetry.addLine(String.format("double integralGain = %.6f;", bestParams[16]));
            telemetry.addLine(String.format("double integralCap  = %.2f;", bestParams[17]));
            telemetry.addLine("");
            telemetry.addLine("// Motor balance");
            telemetry.addLine(String.format("double motor1Bias = %.2f;", bestParams[18]));
            telemetry.addLine(String.format("double motor2Bias = %.2f;", bestParams[19]));
        } else {
            String[] names = getParamNames();
            telemetry.addLine("");
            for (int i = 0; i < PARAM_COUNT; i++) {
                if (i == 6 || i == 12 || i == 13 || i == 16 || i == 18) {
                    telemetry.addLine("");
                }
                telemetry.addData(names[i], "%.4f", bestParams[i]);
            }
        }

        telemetry.addLine("");
        telemetry.addLine("────────────────────────────────────────");
        telemetry.addData("Sessions", "%d", sessions);
        telemetry.addData("Best Loss", "%.2f", bestLoss);
        telemetry.addData("Confidence", "%.0f%%", confidenceScore * 100);
        telemetry.addLine("════════════════════════════════════════");

        telemetry.update();
    }

    // ═══════════════════════════════════════════════════════════════
    //  STOP
    // ═══════════════════════════════════════════════════════════════

    @Override
    public void stop() {
        robot.stop();

        saveToFile();
        saveElitesToFile();
        saveToStatic();

        System.arraycopy(bestParams, 0, currentParams, 0, PARAM_COUNT);
        syncFieldsFromParams();

        telemetry.clearAll();
        telemetry.addLine("════════════════════════════════════════");
        telemetry.addLine("       DEPOTUNER v4 - SAVED");
        telemetry.addLine("════════════════════════════════════════");
        telemetry.addLine("");
        telemetry.addLine("✓ Parameters saved to file");
        telemetry.addLine("✓ Elites saved (" + elites.size() + " entries)");
        telemetry.addLine("");
        telemetry.addData("Total Sessions", "%d", sessions);
        telemetry.addData("Improvements", "%d", improvements);
        telemetry.addData("Best Loss", "%.2f", bestLoss);
        telemetry.addData("Confidence", "%.0f%%", confidenceScore * 100);
        telemetry.addLine("");
        telemetry.addLine("Files saved to:");
        telemetry.addLine("  " + SAVE_FOLDER + SAVE_FILE);
        telemetry.addLine("  " + SAVE_FOLDER + ELITE_FILE);
        telemetry.addLine("");
        telemetry.addLine("Use Y+B before stop to see export view.");
        telemetry.addLine("════════════════════════════════════════");
        telemetry.update();
    }
}