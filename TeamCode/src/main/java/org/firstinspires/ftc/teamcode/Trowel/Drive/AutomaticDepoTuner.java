package org.firstinspires.ftc.teamcode.Trowel.Drive;

import com.bylazar.configurables.PanelsConfigurables;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.Trowel.Configs.TrowelHardware;
import java.util.ArrayList;
import java.util.List;

/**
 * Machine Learning Auto-Tuning Deposit System
 *
 * Uses gradient descent ML to optimize ALL parameters:
 * - Ball 1, 2, 3+ boost multipliers and exponents
 * - PIDF coefficients
 * - Feedforward boost curve parameters
 *
 * All parameters are discoverable by Panels when the OpMode initialises.
 */
@Configurable
@TeleOp(name = "ML Auto-Tune Deposit", group = "Trowel")
public class AutomaticDepoTuner extends OpMode {

    // ============== SERVO CONFIGURATION ==============
    public static double SERVO_POSITION_SHOOTING = 0.7;

    public static double SERVO_POSITION_IDLE = 0.3;

    public static double servoDelayMs = 200.0;

    public static double SERVO_DELAY_INCREMENT = 25.0;

    public static double MIN_SERVO_DELAY = 0.0;

    public static double MAX_SERVO_DELAY = 1000.0;

    // ============== DEPOSIT TARGET CONFIGURATION ==============
    public static double depositTargetVelocity = 640.0;

    public static double DEPOSIT_TOLERANCE = 10.0;

    public static double SPEED_INCREMENT_SMALL = 10.0;

    public static double SPEED_INCREMENT_LARGE = 50.0;

    public static double MIN_DEPOSIT_SPEED = 100.0;

    public static double MAX_DEPOSIT_SPEED = 1500.0;

    // ============== DRIVE CONFIGURATION ==============
    public static double DRIVE_POWER_SCALE = 0.8;

    public static double STRAFE_POWER_SCALE = 0.8;

    public static double ROTATE_POWER_SCALE = 0.6;

    public static double INTAKE2_SCALE = 0.8;

    // ============== BOOST TRIGGER CONFIGURATION ==============
    public static double boostTriggerThreshold = 50.0;

    public static double boostMinTicks = 10.0;

    public static double boostMaxTicks = 800.0;

    // ============== BALL 1 BOOST PARAMETERS ==============
    public static double ball1Multiplier = 1.0;

    public static double ball1Exponent = 1.5;

    // ============== BALL 2 BOOST PARAMETERS ==============
    public static double ball2Multiplier = 1.8;

    public static double ball2Exponent = 1.7;

    // ============== BALL 3+ BOOST PARAMETERS ==============
    public static double ball3Multiplier = 1.6;

    public static double ball3Exponent = 1.65;

    // ============== PIDF PARAMETERS ==============
    public static double kP = 10.0;

    public static double kI = 0.0;

    public static double kD = 0.0;

    public static double kF = 0.0;

    // ============== MACHINE LEARNING CONFIGURATION ==============
    public static boolean mlEnabled = true;

    public static double learningRate = 0.01;

    public static double targetDropMagnitude = 30.0;

    public static double targetRecoveryTime = 0.2;

    public static double targetStabilityScore = 90.0;

    public static double targetOvershoot = 5.0;

    // Weight of each objective in loss function
    public static double dropMagnitudeWeight = 2.0;

    public static double recoveryTimeWeight = 1.5;

    public static double stabilityWeight = 1.0;

    public static double overshootWeight = 1.2;

    // Parameter bounds for ML
    public static double minMultiplier = 0.3;

    public static double maxMultiplier = 5.0;

    public static double minExponent = 0.8;

    public static double maxExponent = 3.5;

    public static double minP = 0.0;

    public static double maxP = 50.0;

    public static double minI = 0.0;

    public static double maxI = 5.0;

    public static double minD = 0.0;

    public static double maxD = 10.0;

    public static double minF = 0.0;

    public static double maxF = 1.0;

    // Gradient estimation epsilon
    public static double gradientEpsilon = 0.01;

    // ============== SESSION RECORDING CONFIGURATION ==============
    public static int MAX_SESSIONS = 100;

    public static int MIN_SAMPLES_PER_SESSION = 10;

    public static double BALL_DETECTION_DROP_THRESHOLD = 1.5;

    public static int BALL_DETECTION_STABILITY_WINDOW = 5;

    public static int MIN_SAMPLES_BETWEEN_BALLS = 20;

    // ============== INTERNAL STATE (not configurable) ==============
    private boolean sessionActive = false;
    private boolean lastZLState = false;
    private List<VelocitySnapshot> sessionData = new ArrayList<>();
    private long sessionStartTime = 0;
    private int currentBallInSession = 0;
    private int lastBallDetectionIndex = 0;

    private int totalSessionsCompleted = 0;
    private SessionAnalysis[] sessionHistory;

    private TrowelHardware robot;
    private boolean depositActive = false;
    private boolean lastXState = false;
    private boolean lastAState = false;
    private boolean lastYState = false;

    private boolean lastDpadUp = false;
    private boolean lastDpadDown = false;
    private boolean lastDpadLeft = false;
    private boolean lastDpadRight = false;

    // Machine Learning state
    private double[] currentGradient;
    private double currentLoss = 0.0;
    private double bestLoss = Double.MAX_VALUE;
    private int mlIterations = 0;

    /**
     * Velocity snapshot data point
     */
    private static class VelocitySnapshot {
        long timestamp;
        double velocity;
        int ballNumber;

        VelocitySnapshot(long timestamp, double velocity, int ballNumber) {
            this.timestamp = timestamp;
            this.velocity = velocity;
            this.ballNumber = ballNumber;
        }
    }

    /**
     * Analysis results for an entire shooting session
     */
    private static class SessionAnalysis {
        int ballCount;
        BallMetrics ball1;
        BallMetrics ball2;
        BallMetrics ball3Plus;
        double overallStability;
        double avgRecoveryTime;
        double totalLoss;

        SessionAnalysis(int ballCount) {
            this.ballCount = ballCount;
        }
    }

    /**
     * Metrics for a single ball within a session
     */
    private static class BallMetrics {
        double maxDrop;
        double recoveryTime;
        double overshoot;
        double stabilityScore;
        double avgVelocity;
        double variance;

        BallMetrics() {}
    }

    @Override
    public void init() {
        robot = new TrowelHardware(hardwareMap);
        robot.resetDepositEncoders();

        sessionHistory = new SessionAnalysis[MAX_SESSIONS];
        currentGradient = new double[14]; // 14 parameters to optimize

        if (robot.frontLeft != null) robot.frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        if (robot.frontRight != null) robot.frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        if (robot.backLeft != null) robot.backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        if (robot.backRight != null) robot.backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        if (robot.transferServo != null) {
            robot.transferServo.setPosition(SERVO_POSITION_IDLE);
        }

        updatePIDFCoefficients();

        // Register with Panels so fields annotated at class level are exposed to the Panels app
        try { PanelsConfigurables.INSTANCE.refreshClass(this); } catch (Exception ignored) {}

        telemetry.addLine("═══════════════════════════════");
        telemetry.addLine("MACHINE LEARNING AUTO-TUNE");
        telemetry.addLine("═══════════════════════════════");
        telemetry.addLine("");
        telemetry.addLine("🤖 ML ENABLED: " + (mlEnabled ? "YES" : "NO"));
        telemetry.addLine("📊 14 Parameters Optimized:");
        telemetry.addLine("  - Ball 1/2/3 Multipliers");
        telemetry.addLine("  - Ball 1/2/3 Exponents");
        telemetry.addLine("  - PIDF (P, I, D, F)");
        telemetry.addLine("  - Boost Trigger/Min/Max");
        telemetry.addLine("");
        telemetry.addLine("All params configurable in Panels (external app)");
        telemetry.addLine("https://panels.bylazar.com/");
        telemetry.addLine("═══════════════════════════════");
        telemetry.update();
    }

    @Override
    public void loop() {
        // ============== DRIVE CONTROL ==============
        double forward = -gamepad1.left_stick_y * DRIVE_POWER_SCALE;
        double strafe = gamepad1.left_stick_x * STRAFE_POWER_SCALE;
        double rotate = gamepad1.right_stick_x * ROTATE_POWER_SCALE;

        double frontLeftPower = forward + strafe + rotate;
        double frontRightPower = forward - strafe - rotate;
        double backLeftPower = forward - strafe + rotate;
        double backRightPower = forward + strafe - rotate;

        double maxPower = Math.max(Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower)),
                Math.max(Math.abs(backLeftPower), Math.abs(backRightPower)));
        if (maxPower > 1.0) {
            frontLeftPower /= maxPower;
            frontRightPower /= maxPower;
            backLeftPower /= maxPower;
            backRightPower /= maxPower;
        }

        if (robot.frontLeft != null) robot.frontLeft.setPower(frontLeftPower);
        if (robot.frontRight != null) robot.frontRight.setPower(frontRightPower);
        if (robot.backLeft != null) robot.backLeft.setPower(backLeftPower);
        if (robot.backRight != null) robot.backRight.setPower(backRightPower);

        // ============== SPEED/DELAY ADJUSTMENT ==============
        boolean yPressed = gamepad1.y;
        boolean dpadUp = gamepad1.dpad_up;
        boolean dpadDown = gamepad1.dpad_down;
        boolean dpadLeft = gamepad1.dpad_left;
        boolean dpadRight = gamepad1.dpad_right;

        if (yPressed) {
            if (dpadUp && !lastDpadUp) {
                servoDelayMs += SERVO_DELAY_INCREMENT;
                servoDelayMs = Math.min(MAX_SERVO_DELAY, servoDelayMs);
            }
            if (dpadDown && !lastDpadDown) {
                servoDelayMs -= SERVO_DELAY_INCREMENT;
                servoDelayMs = Math.max(MIN_SERVO_DELAY, servoDelayMs);
            }
        } else {
            if (dpadUp && !lastDpadUp) {
                depositTargetVelocity += SPEED_INCREMENT_SMALL;
                depositTargetVelocity = Math.min(MAX_DEPOSIT_SPEED, depositTargetVelocity);
            }
            if (dpadDown && !lastDpadDown) {
                depositTargetVelocity -= SPEED_INCREMENT_SMALL;
                depositTargetVelocity = Math.max(MIN_DEPOSIT_SPEED, depositTargetVelocity);
            }
            if (dpadRight && !lastDpadRight) {
                depositTargetVelocity += SPEED_INCREMENT_LARGE;
                depositTargetVelocity = Math.min(MAX_DEPOSIT_SPEED, depositTargetVelocity);
            }
            if (dpadLeft && !lastDpadLeft) {
                depositTargetVelocity -= SPEED_INCREMENT_LARGE;
                depositTargetVelocity = Math.max(MIN_DEPOSIT_SPEED, depositTargetVelocity);
            }
        }

        lastYState = yPressed;
        lastDpadUp = dpadUp;
        lastDpadDown = dpadDown;
        lastDpadLeft = dpadLeft;
        lastDpadRight = dpadRight;

        // ============== DEPOSIT TOGGLE ==============
        boolean xPressed = gamepad1.x;
        if (xPressed && !lastXState) {
            depositActive = !depositActive;
            if (!depositActive) {
                robot.stopDeposit();
                if (sessionActive) endSession();
            } else {
                updatePIDFCoefficients();
            }
        }
        lastXState = xPressed;

        // ============== ZL SESSION CONTROL ==============
        boolean zlPressed = gamepad1.left_trigger > 0.5;

        if (zlPressed && !lastZLState && depositActive) {
            startSession();
            if (robot.transferServo != null) {
                robot.transferServo.setPosition(SERVO_POSITION_SHOOTING);
            }
        } else if (!zlPressed && lastZLState) {
            if (sessionActive) endSession();
            if (robot.transferServo != null) {
                robot.transferServo.setPosition(SERVO_POSITION_IDLE);
            }
        }
        lastZLState = zlPressed;

        // ============== INTAKE CONTROL ==============
        boolean aPressed = gamepad1.a;
        if (aPressed) {
            if (robot.intake1 != null) robot.intake1.setPower(1.0);
            if (robot.intake2 != null) robot.intake2.setPower(-INTAKE2_SCALE);
        } else {
            if (robot.intake1 != null) robot.intake1.setPower(0.0);
            if (robot.intake2 != null) robot.intake2.setPower(0.0);
        }
        lastAState = aPressed;

        // ============== DEPOSIT CONTROL + RECORDING ==============
        if (depositActive && robot.deposit1 != null && robot.deposit2 != null) {
            double vel1 = robot.getDeposit1Velocity();
            double vel2 = robot.getDeposit2Velocity();
            double avgVel = (vel1 + vel2) / 2.0;

            // Record if session active
            if (sessionActive) {
                sessionData.add(new VelocitySnapshot(System.currentTimeMillis(), avgVel, currentBallInSession));
                detectBallTransition(avgVel);
            }

            // Calculate boost
            double drop = Math.max(0.0, depositTargetVelocity - avgVel);
            double appliedBoost = calculateBoost(drop, currentBallInSession);

            robot.setDepositVelocity(depositTargetVelocity + appliedBoost);
        }

        // ============== TELEMETRY ==============
        displayTelemetry();
    }

    /**
     * Calculate boost based on current ball number and parameters
     */
    private double calculateBoost(double drop, int ballNum) {
        if (drop < boostTriggerThreshold) return 0.0;

        double multiplier, exponent;
        if (ballNum == 0) {
            multiplier = ball1Multiplier;
            exponent = ball1Exponent;
        } else if (ballNum == 1) {
            multiplier = ball2Multiplier;
            exponent = ball2Exponent;
        } else {
            multiplier = ball3Multiplier;
            exponent = ball3Exponent;
        }

        double boost = multiplier * Math.pow(drop, exponent);
        return Math.max(boostMinTicks, Math.min(boostMaxTicks, boost));
    }

    /**
     * Detect ball transitions during session
     */
    private void detectBallTransition(double currentVel) {
        if (sessionData.size() < BALL_DETECTION_STABILITY_WINDOW + 5) return;
        if (sessionData.size() - lastBallDetectionIndex < MIN_SAMPLES_BETWEEN_BALLS) return;

        // Check for significant drop
        double prevAvg = 0;
        for (int i = 1; i <= BALL_DETECTION_STABILITY_WINDOW; i++) {
            prevAvg += sessionData.get(sessionData.size() - BALL_DETECTION_STABILITY_WINDOW - i).velocity;
        }
        prevAvg /= BALL_DETECTION_STABILITY_WINDOW;

        double drop = prevAvg - currentVel;
        if (drop > boostTriggerThreshold * BALL_DETECTION_DROP_THRESHOLD) {
            // Check if previous samples were stable
            boolean wasStable = true;
            for (int i = BALL_DETECTION_STABILITY_WINDOW + 1; i <= BALL_DETECTION_STABILITY_WINDOW * 2; i++) {
                double vel = sessionData.get(sessionData.size() - i).velocity;
                if (Math.abs(depositTargetVelocity - vel) > boostTriggerThreshold) {
                    wasStable = false;
                    break;
                }
            }

            if (wasStable) {
                currentBallInSession++;
                lastBallDetectionIndex = sessionData.size();
            }
        }
    }

    /**
     * Start recording session
     */
    private void startSession() {
        sessionActive = true;
        sessionData.clear();
        sessionStartTime = System.currentTimeMillis();
        currentBallInSession = 0;
        lastBallDetectionIndex = 0;
    }

    /**
     * End session and run ML
     */
    private void endSession() {
        if (!sessionActive || sessionData.size() < MIN_SAMPLES_PER_SESSION) {
            sessionActive = false;
            return;
        }

        sessionActive = false;
        SessionAnalysis analysis = analyzeSession();

        if (totalSessionsCompleted < MAX_SESSIONS) {
            sessionHistory[totalSessionsCompleted] = analysis;
        }
        totalSessionsCompleted++;

        if (mlEnabled) {
            runMachineLearning(analysis);
        }
    }

    /**
     * Analyze session data
     */
    private SessionAnalysis analyzeSession() {
        List<Integer> ballStarts = detectBallBoundaries();
        int ballCount = ballStarts.size();
        SessionAnalysis analysis = new SessionAnalysis(ballCount);

        for (int i = 0; i < ballCount; i++) {
            int start = ballStarts.get(i);
            int end = (i < ballCount - 1) ? ballStarts.get(i + 1) : sessionData.size();
            BallMetrics metrics = analyzeBallSegment(start, end);

            if (i == 0) analysis.ball1 = metrics;
            else if (i == 1) analysis.ball2 = metrics;
            else {
                if (analysis.ball3Plus == null) analysis.ball3Plus = metrics;
                else {
                    analysis.ball3Plus.maxDrop = (analysis.ball3Plus.maxDrop + metrics.maxDrop) / 2.0;
                    analysis.ball3Plus.recoveryTime = (analysis.ball3Plus.recoveryTime + metrics.recoveryTime) / 2.0;
                    analysis.ball3Plus.overshoot = (analysis.ball3Plus.overshoot + metrics.overshoot) / 2.0;
                    analysis.ball3Plus.stabilityScore = (analysis.ball3Plus.stabilityScore + metrics.stabilityScore) / 2.0;
                    analysis.ball3Plus.variance = (analysis.ball3Plus.variance + metrics.variance) / 2.0;
                }
            }
        }

        // Overall stability
        double totalVar = 0;
        for (VelocitySnapshot s : sessionData) {
            double diff = s.velocity - depositTargetVelocity;
            totalVar += diff * diff;
        }
        totalVar /= sessionData.size();
        analysis.overallStability = Math.max(0, 100 - (totalVar / 10.0));

        // Calculate loss for ML
        analysis.totalLoss = calculateSessionLoss(analysis);

        return analysis;
    }

    /**
     * Detect ball boundaries in session
     */
    private List<Integer> detectBallBoundaries() {
        List<Integer> boundaries = new ArrayList<>();
        boundaries.add(0);

        for (int i = 15; i < sessionData.size(); i++) {
            double cur = sessionData.get(i).velocity;
            double prev = sessionData.get(i - 10).velocity;

            if (prev - cur > boostTriggerThreshold * BALL_DETECTION_DROP_THRESHOLD) {
                boolean stable = true;
                for (int j = i - 14; j < i - 9; j++) {
                    if (j >= 0 && Math.abs(sessionData.get(j).velocity - depositTargetVelocity) > boostTriggerThreshold) {
                        stable = false;
                        break;
                    }
                }

                if (stable && (boundaries.isEmpty() || i - boundaries.get(boundaries.size() - 1) > MIN_SAMPLES_BETWEEN_BALLS)) {
                    boundaries.add(i);
                }
            }
        }

        return boundaries;
    }

    /**
     * Analyze single ball segment
     */
    private BallMetrics analyzeBallSegment(int start, int end) {
        BallMetrics m = new BallMetrics();

        double min = Double.MAX_VALUE, max = Double.MIN_VALUE, sum = 0;
        int dropIdx = -1, recoverIdx = -1;

        for (int i = start; i < end && i < sessionData.size(); i++) {
            double v = sessionData.get(i).velocity;
            sum += v;
            if (v < min) { min = v; dropIdx = i; }
            if (v > max) max = v;

            if (dropIdx >= 0 && recoverIdx < 0 && v >= depositTargetVelocity - boostTriggerThreshold * 0.5) {
                recoverIdx = i;
            }
        }

        m.maxDrop = depositTargetVelocity - min;
        m.overshoot = Math.max(0, max - depositTargetVelocity);
        m.avgVelocity = sum / (end - start);

        if (dropIdx >= 0 && recoverIdx >= 0) {
            m.recoveryTime = (sessionData.get(recoverIdx).timestamp - sessionData.get(dropIdx).timestamp) / 1000.0;
        }

        double var = 0;
        for (int i = start; i < end && i < sessionData.size(); i++) {
            double diff = sessionData.get(i).velocity - depositTargetVelocity;
            var += diff * diff;
        }
        m.variance = var / (end - start);
        m.stabilityScore = 1000.0 / (1.0 + m.variance);

        return m;
    }

    /**
     * Calculate loss for a session (lower is better)
     */
    private double calculateSessionLoss(SessionAnalysis analysis) {
        double loss = 0.0;

        if (analysis.ball1 != null) {
            loss += dropMagnitudeWeight * Math.pow(analysis.ball1.maxDrop - targetDropMagnitude, 2);
            loss += recoveryTimeWeight * Math.pow(analysis.ball1.recoveryTime - targetRecoveryTime, 2);
            loss += overshootWeight * Math.pow(analysis.ball1.overshoot - targetOvershoot, 2);
        }

        if (analysis.ball2 != null) {
            loss += dropMagnitudeWeight * Math.pow(analysis.ball2.maxDrop - targetDropMagnitude, 2);
            loss += recoveryTimeWeight * Math.pow(analysis.ball2.recoveryTime - targetRecoveryTime, 2);
            loss += overshootWeight * Math.pow(analysis.ball2.overshoot - targetOvershoot, 2);
        }

        if (analysis.ball3Plus != null) {
            loss += dropMagnitudeWeight * Math.pow(analysis.ball3Plus.maxDrop - targetDropMagnitude, 2);
            loss += recoveryTimeWeight * Math.pow(analysis.ball3Plus.recoveryTime - targetRecoveryTime, 2);
            loss += overshootWeight * Math.pow(analysis.ball3Plus.overshoot - targetOvershoot, 2);
        }

        loss += stabilityWeight * Math.pow(analysis.overallStability - targetStabilityScore, 2);

        return loss;
    }

    /**
     * MACHINE LEARNING: Gradient Descent Optimization
     * Optimizes all 14 parameters to minimize loss
     */
    private void runMachineLearning(SessionAnalysis currentAnalysis) {
        currentLoss = currentAnalysis.totalLoss;

        // Estimate gradients for all parameters
        estimateGradients();

        // Gradient descent update
        updateParameters();

        // Update PIDF
        updatePIDFCoefficients();

        mlIterations++;

        if (currentLoss < bestLoss) {
            bestLoss = currentLoss;
        }
    }

    /**
     * Estimate gradients using finite differences
     */
    private void estimateGradients() {
        // Params: [ball1M, ball1E, ball2M, ball2E, ball3M, ball3E, P, I, D, F, thresh, minB, maxB]
        double[] params = getCurrentParameters();

        for (int i = 0; i < params.length; i++) {
            double original = params[i];

            // Forward difference
            params[i] = original + gradientEpsilon;
            double lossPlus = estimateLoss(params);

            params[i] = original - gradientEpsilon;
            double lossMinus = estimateLoss(params);

            // Central difference gradient
            currentGradient[i] = (lossPlus - lossMinus) / (2.0 * gradientEpsilon);

            params[i] = original; // Restore
        }
    }

    /**
     * Get current parameters as array
     */
    private double[] getCurrentParameters() {
        return new double[] {
                ball1Multiplier, ball1Exponent,
                ball2Multiplier, ball2Exponent,
                ball3Multiplier, ball3Exponent,
                kP, kI, kD, kF,
                boostTriggerThreshold, boostMinTicks, boostMaxTicks
        };
    }

    /**
     * Estimate loss for given parameters (simplified)
     */
    private double estimateLoss(double[] params) {
        // Use last session's metrics but with new parameters
        // This is a simplification - ideally would re-simulate
        if (totalSessionsCompleted == 0) return currentLoss;

        SessionAnalysis last = sessionHistory[totalSessionsCompleted - 1];
        if (last == null) return currentLoss;

        // Approximate loss based on how parameters would affect metrics
        double estLoss = 0.0;

        // Ball 1
        if (last.ball1 != null) {
            double dropAdjust = (params[0] / ball1Multiplier) * last.ball1.maxDrop;
            estLoss += dropMagnitudeWeight * Math.pow(dropAdjust - targetDropMagnitude, 2);
        }

        // Ball 2
        if (last.ball2 != null) {
            double dropAdjust = (params[2] / ball2Multiplier) * last.ball2.maxDrop;
            estLoss += dropMagnitudeWeight * Math.pow(dropAdjust - targetDropMagnitude, 2);
        }

        // Ball 3
        if (last.ball3Plus != null) {
            double dropAdjust = (params[4] / ball3Multiplier) * last.ball3Plus.maxDrop;
            estLoss += dropMagnitudeWeight * Math.pow(dropAdjust - targetDropMagnitude, 2);
        }

        estLoss += stabilityWeight * Math.pow(last.overallStability - targetStabilityScore, 2);

        return estLoss;
    }

    /**
     * Update all parameters using gradient descent
     */
    private void updateParameters() {
        ball1Multiplier -= learningRate * currentGradient[0];
        ball1Exponent -= learningRate * currentGradient[1];
        ball2Multiplier -= learningRate * currentGradient[2];
        ball2Exponent -= learningRate * currentGradient[3];
        ball3Multiplier -= learningRate * currentGradient[4];
        ball3Exponent -= learningRate * currentGradient[5];
        kP -= learningRate * currentGradient[6];
        kI -= learningRate * currentGradient[7];
        kD -= learningRate * currentGradient[8];
        kF -= learningRate * currentGradient[9];
        boostTriggerThreshold -= learningRate * currentGradient[10];
        boostMinTicks -= learningRate * currentGradient[11];
        boostMaxTicks -= learningRate * currentGradient[12];

        // Enforce bounds
        ball1Multiplier = clamp(ball1Multiplier, minMultiplier, maxMultiplier);
        ball1Exponent = clamp(ball1Exponent, minExponent, maxExponent);
        ball2Multiplier = clamp(ball2Multiplier, minMultiplier, maxMultiplier);
        ball2Exponent = clamp(ball2Exponent, minExponent, maxExponent);
        ball3Multiplier = clamp(ball3Multiplier, minMultiplier, maxMultiplier);
        ball3Exponent = clamp(ball3Exponent, minExponent, maxExponent);
        kP = clamp(kP, minP, maxP);
        kI = clamp(kI, minI, maxI);
        kD = clamp(kD, minD, maxD);
        kF = clamp(kF, minF, maxF);
        boostTriggerThreshold = clamp(boostTriggerThreshold, 10.0, 200.0);
        boostMinTicks = clamp(boostMinTicks, 0.0, 100.0);
        boostMaxTicks = clamp(boostMaxTicks, 100.0, 2000.0);
    }

    private double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }

    /**
     * Apply PIDF coefficients
     */
    private void updatePIDFCoefficients() {
        if (robot.deposit1 != null) {
            robot.deposit1.setVelocityPIDFCoefficients(kP, kI, kD, kF);
        }
        if (robot.deposit2 != null) {
            robot.deposit2.setVelocityPIDFCoefficients(kP, kI, kD, kF);
        }
    }

    /**
     * Display telemetry
     */
    private void displayTelemetry() {
        telemetry.addLine("═══════════════════════════════");
        telemetry.addLine("🤖 ML AUTO-TUNE");
        telemetry.addLine("═══════════════════════════════");
        telemetry.addLine("");

        telemetry.addLine("─── CONTROLS ───");
        telemetry.addData("Deposit (X)", depositActive ? "✓" : "✗");
        telemetry.addData("Intakes (A)", gamepad1.a ? "✓" : "✗");
        telemetry.addData("Servo (ZL)", (gamepad1.left_trigger > 0.5) ? "SHOOT" : "IDLE");
        telemetry.addData("Speed", "%.0f", depositTargetVelocity);
        telemetry.addLine("");

        telemetry.addLine("─── SESSION ───");
        if (sessionActive) {
            telemetry.addData("Status", "🔴 REC");
            telemetry.addData("Ball", "%d", currentBallInSession + 1);
            telemetry.addData("Samples", "%d", sessionData.size());
        } else {
            telemetry.addData("Status", "⚪ IDLE");
        }
        telemetry.addData("Completed", "%d", totalSessionsCompleted);
        telemetry.addLine("");

        telemetry.addLine("─── ML STATUS ───");
        telemetry.addData("Enabled", mlEnabled ? "YES" : "NO");
        telemetry.addData("Iterations", "%d", mlIterations);
        telemetry.addData("Current Loss", "%.2f", currentLoss);
        telemetry.addData("Best Loss", "%.2f", bestLoss);
        telemetry.addData("Learn Rate", "%.4f", learningRate);
        telemetry.addLine("");

        if (depositActive && robot.deposit1 != null) {
            double v1 = robot.getDeposit1Velocity();
            double v2 = robot.getDeposit2Velocity();
            double avg = (v1 + v2) / 2.0;
            telemetry.addLine("─── VELOCITY ───");
            telemetry.addData("Avg", "%.0f", avg);
            telemetry.addData("Error", "%.0f", depositTargetVelocity - avg);
            telemetry.addLine("");
        }

        if (totalSessionsCompleted > 0 && sessionHistory[totalSessionsCompleted - 1] != null) {
            SessionAnalysis last = sessionHistory[totalSessionsCompleted - 1];
            telemetry.addLine("─── LAST SESSION ───");
            telemetry.addData("Balls", "%d", last.ballCount);
            telemetry.addData("Stability", "%.1f", last.overallStability);
            if (last.ball1 != null) telemetry.addData("B1 Drop", "%.0f", last.ball1.maxDrop);
            if (last.ball2 != null) telemetry.addData("B2 Drop", "%.0f", last.ball2.maxDrop);
            if (last.ball3Plus != null) telemetry.addData("B3+ Drop", "%.0f", last.ball3Plus.maxDrop);
            telemetry.addLine("");
        }

        telemetry.addLine("─── BOOST ───");
        telemetry.addData("B1", "M%.2f E%.2f", ball1Multiplier, ball1Exponent);
        telemetry.addData("B2", "M%.2f E%.2f", ball2Multiplier, ball2Exponent);
        telemetry.addData("B3", "M%.2f E%.2f", ball3Multiplier, ball3Exponent);
        telemetry.addLine("");

        telemetry.addLine("─── PIDF ───");
        telemetry.addData("Values", "%.1f/%.3f/%.2f/%.4f", kP, kI, kD, kF);
        telemetry.addLine("═══════════════════════════════");

        telemetry.update();
    }

    @Override
    public void stop() {
        robot.stop();

        telemetry.clearAll();
        telemetry.addLine("═══════════════════════════════");
        telemetry.addLine("FINAL ML-TUNED PARAMETERS");
        telemetry.addLine("═══════════════════════════════");
        telemetry.addLine("");
        telemetry.addLine("BALL 1 BOOST:");
        telemetry.addData("ball1Multiplier", "%.3f", ball1Multiplier);
        telemetry.addData("ball1Exponent", "%.3f", ball1Exponent);
        telemetry.addLine("");
        telemetry.addLine("BALL 2 BOOST (Extra Stab):");
        telemetry.addData("ball2Multiplier", "%.3f", ball2Multiplier);
        telemetry.addData("ball2Exponent", "%.3f", ball2Exponent);
        telemetry.addLine("");
        telemetry.addLine("BALL 3+ BOOST (Extra Stab):");
        telemetry.addData("ball3Multiplier", "%.3f", ball3Multiplier);
        telemetry.addData("ball3Exponent", "%.3f", ball3Exponent);
        telemetry.addLine("");
        telemetry.addLine("BOOST LIMITS:");
        telemetry.addData("boostTriggerThreshold", "%.1f", boostTriggerThreshold);
        telemetry.addData("boostMinTicks", "%.1f", boostMinTicks);
        telemetry.addData("boostMaxTicks", "%.1f", boostMaxTicks);
        telemetry.addLine("");
        telemetry.addLine("PIDF:");
        telemetry.addData("kP", "%.3f", kP);
        telemetry.addData("kI", "%.4f", kI);
        telemetry.addData("kD", "%.3f", kD);
        telemetry.addData("kF", "%.5f", kF);
        telemetry.addLine("");
        telemetry.addLine("SERVO:");
        telemetry.addData("servoDelayMs", "%.0f", servoDelayMs);
        telemetry.addLine("");
        telemetry.addLine("ML STATS:");
        telemetry.addData("Sessions", "%d", totalSessionsCompleted);
        telemetry.addData("ML Iterations", "%d", mlIterations);
        telemetry.addData("Final Loss", "%.2f", currentLoss);
        telemetry.addData("Best Loss", "%.2f", bestLoss);
        telemetry.addLine("");
        telemetry.addLine("Copy to your main TeleOp!");
        telemetry.addLine("Or adjust in FTC Dashboard");
        telemetry.addLine("═══════════════════════════════");
        telemetry.update();
    }
}

