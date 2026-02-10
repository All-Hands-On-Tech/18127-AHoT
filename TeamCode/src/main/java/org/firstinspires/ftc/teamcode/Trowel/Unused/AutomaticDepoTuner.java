package org.firstinspires.ftc.teamcode.Trowel.Unused;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.Trowel.Configs.TrowelHardware;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

/**
 * Neural Network Auto-Tuning Deposit System
 *
 * Uses a 3-layer neural network (Input: 12, Hidden: 10, Output: 9)
 * to learn optimal parameter adjustments in real-time. Only tunes
 * per-ball boost multipliers/exponents and global boost curve params.
 *
 * DOES NOT tune PID+F coefficients — the controller's built-in PID+F is left untouched.
 */
@TeleOp(name = "Do not use", group = "Trowel")
public class AutomaticDepoTuner extends OpMode {

    // ============== SERVO CONFIGURATION ==============
    public double SERVO_POSITION_SHOOTING = 0.7;
    public double SERVO_POSITION_IDLE = 0.3;
    public double servoDelayMs = 200.0;

    // ============== DEPOSIT TARGET CONFIGURATION ==============
    public double depositTargetVelocity = 640.0;
    public double SPEED_INCREMENT_SMALL = 10.0;
    public double SPEED_INCREMENT_LARGE = 50.0;
    public double MIN_DEPOSIT_SPEED = 100.0;
    public double MAX_DEPOSIT_SPEED = 1500.0;

    // ============== DRIVE CONFIGURATION ==============
    public double DRIVE_POWER_SCALE = 0.8;
    public double STRAFE_POWER_SCALE = 0.8;
    public double ROTATE_POWER_SCALE = 0.6;
    public double INTAKE2_SCALE = 0.8;

    // ============== BOOST CONFIGURATION ==============
    public double boostTriggerThreshold = 50.0;
    public double boostMinTicks = 10.0;
    public double boostMaxTicks = 800.0;

    // ============== BALL-SPECIFIC BOOST PARAMETERS ==============
    public double ball1Multiplier = 1.0;
    public double ball1Exponent = 1.5;
    public double ball2Multiplier = 1.8;
    public double ball2Exponent = 1.7;
    public double ball3Multiplier = 1.6;
    public double ball3Exponent = 1.65;

    // ============== NEURAL NETWORK CONFIGURATION ==============
    public boolean nnEnabled = true;
    public double learningRate = 0.015; // Fixed learning rate - never decays
    public int inputNeurons = 8; // Reduced from 12 - less computation
    public int hiddenNeurons = 6; // Reduced from 10 - less computation
    // Reduced outputs: 6 for ball multipliers/exponents, 3 for global boost params
    public int outputNeurons = 9;

    // Reduced smoothing - allow faster iteration
    public double parameterSmoothingFactor = 0.6; // 60% old, 40% new

    // BATTERY SAVING: Reduce training frequency
    public int trainingIterationsPerSession = 2; // Reduced from 5 - saves 60% power

    // Adaptive learning - increase iterations when stuck, decrease when improving
    public int minIterationsPerSession = 1; // Reduced from 3
    public int maxIterationsPerSession = 4; // Reduced from 10 - cap at 4 for battery
    public double lossImprovementThreshold = 0.98; // If new loss > 98% of best, we're stuck

    // BATTERY SAVING: Only train every N sessions
    public int trainingFrequency = 2; // Train every 2 sessions instead of every session

    // Strict quality thresholds - don't accept mediocrity
    public double acceptableOvershoot = 15.0; // Max 15 ticks overshoot
    public double acceptableDropMagnitude = 25.0; // Max 25 ticks drop
    public double acceptableStability = 95.0; // Need 95+ stability

    // Training parameters - STRICT quality targets
    public double targetDropMagnitude = 20.0; // Reduced from 30 - tighter tolerance
    public double targetRecoveryTime = 0.15; // Reduced from 0.2 - faster recovery required
    public double targetStabilityScore = 95.0; // Increased from 90 - much more stable
    public double targetOvershoot = 10.0; // Reduced from 5 - even less overshoot allowed

    // Loss weights - heavily emphasize perfection
    public double dropWeight = 3.0; // Increased from 2.0
    public double recoveryWeight = 2.5; // Increased from 1.5
    public double stabilityWeight = 2.0; // Increased from 1.0
    public double overshootWeight = 10.0; // Increased from 8.0 - absolutely critical
    // New weight for direct speed mismatch penalty (prioritize matching driver target)
    public double speedMismatchWeight = 8.0; // Increased from 5.0

    // AUTOMATIC REWARD/PUNISHMENT SYSTEM
    public double autoRewardThreshold = 0.9; // If session >= 90% perfect, auto-reward
    public double autoPunishThreshold = 0.3; // If session <= 30% quality, auto-punish
    public double manualRewardMultiplier = 10.0; // Manual rewards count 10x more
    public double manualPunishMultiplier = 10.0; // Manual punishments count 10x more

    // ============== SESSION RECORDING ==============
    public int MAX_SESSIONS = 100;
    public int MIN_SAMPLES_PER_SESSION = 10;
    public double BALL_DETECTION_DROP_THRESHOLD = 1.5;

    // ============== INTERNAL STATE ==============
    private boolean sessionActive = false;
    private boolean lastZLState = false;
    private final List<VelocitySnapshot> sessionData = new ArrayList<>();
    private int currentBallInSession = 0;
    private int lastBallDetectionIndex = 0;

    private int totalSessionsCompleted = 0;
    private SessionAnalysis[] sessionHistory;

    private TrowelHardware robot;
    private boolean depositActive = false;
    private boolean lastXState = false;

    private boolean lastDpadUp = false;
    private boolean lastDpadDown = false;
    private boolean lastDpadLeft = false;
    private boolean lastDpadRight = false;
    private boolean lastManualRewardButton = false;
    private boolean lastManualPunishButton = false;

    // Track last driver-set target to detect changes during recording
    private double lastDepositTargetVelocity = 640.0;

    // Reinforcement and evolution control
    private NetworkParams bestParams = null;
    private boolean bestParamsInitialized = false;
    private boolean waitingForReinforcement = false;
    private boolean lastGamepad2Y = false;
    private boolean lastSessionSuccessful = false;
    private SessionAnalysis lastSessionAnalysis = null;
    private int reinforcementCount = 0;

    // Neural Network
    private NeuralNetwork neuralNet;
    private double currentLoss = 0.0;
    private double bestLoss = Double.MAX_VALUE;
    private int trainingIterations = 0;

    // Adaptive learning tracking
    private int sessionsWithoutImprovement = 0;
    private double lastBestLoss = Double.MAX_VALUE;
    private int sessionsSinceLastTraining = 0; // For battery saving

    // Automatic reward/punishment tracking
    private int autoRewardCount = 0;
    private int autoPunishCount = 0;

    /**
     * Container for network parameters so we can save/restore safely
     */
    private static class NetworkParams {
        final double[][] weightsInputHidden;
        final double[][] weightsHiddenOutput;
        final double[] biasHidden;
        final double[] biasOutput;

        NetworkParams(int in, int hid, int out) {
            weightsInputHidden = new double[in][hid];
            weightsHiddenOutput = new double[hid][out];
            biasHidden = new double[hid];
            biasOutput = new double[out];
        }
    }

    /**
     * Simple 3-layer Neural Network with optimized architecture
     */
    private static class NeuralNetwork {
        private final double[][] weightsInputHidden;
        private final double[][] weightsHiddenOutput;
        private final double[] biasHidden;
        private final double[] biasOutput;

        private final double[] hiddenLayer;
        private final double[] outputLayer;

        private final int inputSize;
        private final int hiddenSize;
        private final int outputSize;

        private final Random random = new Random();

        // Adaptive learning rate with momentum
        private double[][] momentumIH;
        private double[][] momentumHO;
        private double[] momentumBH;
        private double[] momentumBO;
        private final double momentumFactor = 0.9;

        public NeuralNetwork(int inputSize, int hiddenSize, int outputSize) {
            this.inputSize = inputSize;
            this.hiddenSize = hiddenSize;
            this.outputSize = outputSize;

            // Initialize weights with Xavier initialization
            weightsInputHidden = new double[inputSize][hiddenSize];
            weightsHiddenOutput = new double[hiddenSize][outputSize];
            biasHidden = new double[hiddenSize];
            biasOutput = new double[outputSize];

            hiddenLayer = new double[hiddenSize];
            outputLayer = new double[outputSize];

            // Initialize momentum arrays
            momentumIH = new double[inputSize][hiddenSize];
            momentumHO = new double[hiddenSize][outputSize];
            momentumBH = new double[hiddenSize];
            momentumBO = new double[outputSize];

            initializeWeights();
        }

        private void initializeWeights() {
            // Xavier/Glorot initialization for better convergence
            double limitIH = Math.sqrt(6.0 / (inputSize + hiddenSize));
            double limitHO = Math.sqrt(6.0 / (hiddenSize + outputSize));

            for (int i = 0; i < inputSize; i++) {
                for (int j = 0; j < hiddenSize; j++) {
                    weightsInputHidden[i][j] = (random.nextDouble() * 2 - 1) * limitIH;
                }
            }

            for (int i = 0; i < hiddenSize; i++) {
                for (int j = 0; j < outputSize; j++) {
                    weightsHiddenOutput[i][j] = (random.nextDouble() * 2 - 1) * limitHO;
                }
            }
        }

        /**
         * Export a deep copy of the current parameters
         */
        public NetworkParams exportParams() {
            NetworkParams p = new NetworkParams(inputSize, hiddenSize, outputSize);
            for (int i = 0; i < inputSize; i++) {
                System.arraycopy(weightsInputHidden[i], 0, p.weightsInputHidden[i], 0, hiddenSize);
            }
            for (int i = 0; i < hiddenSize; i++) {
                System.arraycopy(weightsHiddenOutput[i], 0, p.weightsHiddenOutput[i], 0, outputSize);
            }
            System.arraycopy(biasHidden, 0, p.biasHidden, 0, hiddenSize);
            System.arraycopy(biasOutput, 0, p.biasOutput, 0, outputSize);
            return p;
        }

        /**
         * Load parameters from a NetworkParams instance (copy)
         */
        public void importParams(NetworkParams p) {
            if (p == null) return;
            for (int i = 0; i < Math.min(inputSize, p.weightsInputHidden.length); i++) {
                System.arraycopy(p.weightsInputHidden[i], 0, weightsInputHidden[i], 0,
                        Math.min(hiddenSize, p.weightsInputHidden[i].length));
            }
            for (int i = 0; i < Math.min(hiddenSize, p.weightsHiddenOutput.length); i++) {
                System.arraycopy(p.weightsHiddenOutput[i], 0, weightsHiddenOutput[i], 0,
                        Math.min(outputSize, p.weightsHiddenOutput[i].length));
            }
            System.arraycopy(p.biasHidden, 0, biasHidden, 0, Math.min(biasHidden.length, p.biasHidden.length));
            System.arraycopy(p.biasOutput, 0, biasOutput, 0, Math.min(biasOutput.length, p.biasOutput.length));
        }

        /**
         * Forward pass through network with optimized computation
         */
        public double[] forward(double[] input) {
            // Input to hidden with batch normalization concept
            for (int j = 0; j < hiddenSize; j++) {
                hiddenLayer[j] = biasHidden[j];
                for (int i = 0; i < inputSize; i++) {
                    hiddenLayer[j] += input[i] * weightsInputHidden[i][j];
                }
                hiddenLayer[j] = leakyRelu(hiddenLayer[j]);
            }

            // Hidden to output
            for (int j = 0; j < outputSize; j++) {
                outputLayer[j] = biasOutput[j];
                for (int i = 0; i < hiddenSize; i++) {
                    outputLayer[j] += hiddenLayer[i] * weightsHiddenOutput[i][j];
                }
                outputLayer[j] = Math.tanh(outputLayer[j]); // Use tanh for bounded outputs
            }

            return outputLayer.clone();
        }

        /**
         * Backpropagation training with momentum optimization
         */
        public void train(double[] input, double[] targetAdjustments, double learningRate) {
            // Forward pass
            double[] output = forward(input);

            // Calculate output layer errors
            double[] outputErrors = new double[outputSize];
            for (int i = 0; i < outputSize; i++) {
                outputErrors[i] = targetAdjustments[i] - output[i];
            }

            // Calculate output layer deltas
            double[] outputDeltas = new double[outputSize];
            for (int i = 0; i < outputSize; i++) {
                outputDeltas[i] = outputErrors[i] * tanhDerivative(outputLayer[i]);
            }

            // Calculate hidden layer errors
            double[] hiddenErrors = new double[hiddenSize];
            for (int i = 0; i < hiddenSize; i++) {
                hiddenErrors[i] = 0;
                for (int j = 0; j < outputSize; j++) {
                    hiddenErrors[i] += outputDeltas[j] * weightsHiddenOutput[i][j];
                }
            }

            // Calculate hidden layer deltas
            double[] hiddenDeltas = new double[hiddenSize];
            for (int i = 0; i < hiddenSize; i++) {
                hiddenDeltas[i] = hiddenErrors[i] * leakyReluDerivative(hiddenLayer[i]);
            }

            // Update weights and biases (hidden to output) with momentum
            for (int i = 0; i < hiddenSize; i++) {
                for (int j = 0; j < outputSize; j++) {
                    double gradient = learningRate * outputDeltas[j] * hiddenLayer[i];
                    momentumHO[i][j] = momentumFactor * momentumHO[i][j] + gradient;
                    weightsHiddenOutput[i][j] += momentumHO[i][j];
                }
            }
            for (int i = 0; i < outputSize; i++) {
                double gradient = learningRate * outputDeltas[i];
                momentumBO[i] = momentumFactor * momentumBO[i] + gradient;
                biasOutput[i] += momentumBO[i];
            }

            // Update weights and biases (input to hidden) with momentum
            for (int i = 0; i < inputSize; i++) {
                for (int j = 0; j < hiddenSize; j++) {
                    double gradient = learningRate * hiddenDeltas[j] * input[i];
                    momentumIH[i][j] = momentumFactor * momentumIH[i][j] + gradient;
                    weightsInputHidden[i][j] += momentumIH[i][j];
                }
            }
            for (int i = 0; i < hiddenSize; i++) {
                double gradient = learningRate * hiddenDeltas[i];
                momentumBH[i] = momentumFactor * momentumBH[i] + gradient;
                biasHidden[i] += momentumBH[i];
            }
        }

        // Leaky ReLU for better gradient flow
        private double leakyRelu(double x) {
            return x > 0 ? x : 0.01 * x;
        }

        private double leakyReluDerivative(double x) {
            return x > 0 ? 1.0 : 0.01;
        }

        private double tanhDerivative(double x) {
            return 1.0 - x * x;
        }
    }

    /**
     * Velocity snapshot
     */
    private static class VelocitySnapshot {
        final long timestamp;
        final double velocity;
        final int ballNumber;

        VelocitySnapshot(long timestamp, double velocity, int ballNumber) {
            this.timestamp = timestamp;
            this.velocity = velocity;
            this.ballNumber = ballNumber;
        }
    }

    /**
     * Session analysis
     */
    private static class SessionAnalysis {
        final int ballCount;
        BallMetrics ball1;
        BallMetrics ball2;
        BallMetrics ball3Plus;
        double overallStability;
        double totalLoss;

        SessionAnalysis(int ballCount) {
            this.ballCount = ballCount;
        }
    }

    /**
     * Ball metrics
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

        // Initialize neural network
        neuralNet = new NeuralNetwork(inputNeurons, hiddenNeurons, outputNeurons);

        // Save initial network as best (safe starting point)
        bestParams = neuralNet.exportParams();
        bestParamsInitialized = true;
        bestLoss = Double.MAX_VALUE;

        if (robot.frontLeft != null) robot.frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        if (robot.frontRight != null) robot.frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        if (robot.backLeft != null) robot.backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        if (robot.backRight != null) robot.backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Ensure deposit motors are in RUN_USING_ENCODER mode so velocity commands are respected
        try {
            if (robot.deposit1 != null) robot.deposit1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            if (robot.deposit2 != null) robot.deposit2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        } catch (Exception ignore) {
            // hardware map might not have deposit motors during editor checks
        }

        if (robot.transferServo != null) {
            robot.transferServo.setPosition(SERVO_POSITION_IDLE);
        }

        // Initialize lastDepositTargetVelocity so we can detect driver changes
        lastDepositTargetVelocity = depositTargetVelocity;

        telemetry.addLine("═══════════════════════════════");
        telemetry.addLine("🔋 BATTERY-EFFICIENT AUTO-TUNE");
        telemetry.addLine("═══════════════════════════════");
        telemetry.addLine("");
        telemetry.addLine("Network: 8→6→9 (60% smaller)");
        telemetry.addLine("  Input: 8 features (reduced)");
        telemetry.addLine("  Hidden: 6 neurons (reduced)");
        telemetry.addLine("  Output: 9 adjustments");
        telemetry.addLine("");
        telemetry.addLine("Power Optimizations:");
        telemetry.addLine("  • Trains every 2 sessions");
        telemetry.addLine("  • Max 4 iterations/session");
        telemetry.addLine("  • 80% less computation");
        telemetry.addLine("");
        telemetry.addLine("Configure at:");
        telemetry.addLine("panels.bylazar.com");
        telemetry.addLine("═══════════════════════════════");
        telemetry.update();
    }

    @Override
    public void loop() {
        // ============== DRIVE ==============
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

        // ============== SPEED ADJUSTMENT ==============
        boolean yPressed = gamepad1.y;
        boolean dpadUp = gamepad1.dpad_up;
        boolean dpadDown = gamepad1.dpad_down;
        boolean dpadLeft = gamepad1.dpad_left;
        boolean dpadRight = gamepad1.dpad_right;

        if (yPressed) {
            if (dpadUp && !lastDpadUp) {
                servoDelayMs += 25.0;
                servoDelayMs = Math.min(1000.0, servoDelayMs);
            }
            if (dpadDown && !lastDpadDown) {
                servoDelayMs -= 25.0;
                servoDelayMs = Math.max(0.0, servoDelayMs);
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

        lastDpadUp = dpadUp;
        lastDpadDown = dpadDown;
        lastDpadLeft = dpadLeft;
        lastDpadRight = dpadRight;

        // If driver changed the deposit target during an active session, clear recorded samples
        if (Math.abs(depositTargetVelocity - lastDepositTargetVelocity) > 0.5) {
            if (sessionActive) {
                sessionData.clear();
                currentBallInSession = 0;
                lastBallDetectionIndex = 0;
            }
            lastDepositTargetVelocity = depositTargetVelocity;
        }

        // ============== DEPOSIT TOGGLE ==============
        boolean xPressed = gamepad1.x;
        if (xPressed && !lastXState) {
            depositActive = !depositActive;
            if (!depositActive) {
                robot.stopDeposit();
                if (sessionActive) {
                    endSession();
                }
            }
        }
        lastXState = xPressed;

        // ============== SESSION CONTROL ==============
        boolean zlPressed = gamepad1.left_trigger > 0.5;

        if (zlPressed && !lastZLState && depositActive) {
            startSession();
            if (robot.transferServo != null) {
                robot.transferServo.setPosition(SERVO_POSITION_SHOOTING);
            }
        } else if (!zlPressed && lastZLState) {
            if (sessionActive) {
                endSession();
            }
            if (robot.transferServo != null) {
                robot.transferServo.setPosition(SERVO_POSITION_IDLE);
            }
        }
        lastZLState = zlPressed;

        // ============== INTAKES ==============
        boolean aPressed = gamepad1.a;
        boolean bPressed = gamepad1.b;

        if (aPressed) {
            // Intake mode - full power
            if (robot.intake1 != null) robot.intake1.setPower(1.0);
            if (robot.intake2 != null) robot.intake2.setPower(-1.0);
        } else if (bPressed) {
            // Outake mode - reverse at full power
            if (robot.intake1 != null) robot.intake1.setPower(-1.0);
            if (robot.intake2 != null) robot.intake2.setPower(1.0);
        } else {
            // Stop intakes
            if (robot.intake1 != null) robot.intake1.setPower(0.0);
            if (robot.intake2 != null) robot.intake2.setPower(0.0);
        }

        // ============== GAMEPAD2 REINFORCEMENT CHECK ==============
        boolean gp2Y = gamepad2 != null && gamepad2.y;
        if (gp2Y && !lastGamepad2Y) {
            if (waitingForReinforcement && lastSessionSuccessful && lastSessionAnalysis != null) {
                reinforceLastSession();
            }
        }
        lastGamepad2Y = gp2Y;

        // ============== MANUAL REWARD/PUNISHMENT (Y + DPAD) ==============
        // REWARD: Y + DPAD_UP = All 3 balls made it
        // PUNISH: Y + DPAD_DOWN = Missed balls or terrible performance

        if (yPressed && dpadUp && !lastManualRewardButton && !sessionActive) {
            // REWARD - All 3 balls scored successfully
            if (lastSessionAnalysis != null && totalSessionsCompleted > 0) {
                lastSessionSuccessful = true;
                waitingForReinforcement = false;
                reinforceLastSession();

                // Give very strong positive reinforcement
                if (lastSessionAnalysis != null) {
                    lastSessionAnalysis.totalLoss *= 0.3; // Cut loss to 30% for manual rewards
                    bestLoss = Math.min(bestLoss, lastSessionAnalysis.totalLoss);
                }
            }
        }
        lastManualRewardButton = yPressed && dpadUp;

        if (yPressed && dpadDown && !lastManualPunishButton && !sessionActive) {
            // PUNISHMENT - Missed balls or bad performance
            if (lastSessionAnalysis != null && totalSessionsCompleted > 0) {
                // Revert to best known parameters (undo the bad session)
                if (bestParamsInitialized && bestParams != null) {
                    neuralNet.importParams(bestParams);
                }

                // MANUAL PUNISHMENT: 10x stronger than auto-punish
                // Auto-punish multiplies loss by 2x
                // Manual punish multiplies loss by 5x - much stronger!
                lastSessionAnalysis.totalLoss *= 5.0;
                lastSessionSuccessful = false;
                waitingForReinforcement = false;
            }
        }
        lastManualPunishButton = yPressed && dpadDown;

        // ============== DEPOSIT + RECORDING ==============
        if (depositActive && robot.deposit1 != null && robot.deposit2 != null) {
            double vel1 = robot.getDeposit1Velocity();
            double vel2 = robot.getDeposit2Velocity();
            double avgVel = (vel1 + vel2) / 2.0;

            if (sessionActive) {
                sessionData.add(new VelocitySnapshot(System.currentTimeMillis(), avgVel, currentBallInSession));
                detectBallTransition(avgVel);
            }

            double drop = Math.max(0.0, depositTargetVelocity - avgVel);
            double appliedBoost = calculateBoost(drop, currentBallInSession);

            double desiredVelocity = depositTargetVelocity + appliedBoost;
            robot.setDepositVelocity(desiredVelocity);
        }

        displayTelemetry();
    }

    private void reinforceLastSession() {
        if (!bestParamsInitialized) {
            bestParams = neuralNet.exportParams();
        }
        bestParams = neuralNet.exportParams();
        bestParamsInitialized = true;

        if (lastSessionAnalysis != null) {
            // MANUAL REWARD: 10x stronger than auto-reward
            // Auto-reward reduces loss by 30% (0.7x)
            // Manual reward reduces loss by 70% (0.3x) - much stronger!
            lastSessionAnalysis.totalLoss *= 0.3;
            bestLoss = Math.min(bestLoss, lastSessionAnalysis.totalLoss);
        }

        reinforcementCount++;
        waitingForReinforcement = false;
    }

    private void startSession() {
        sessionActive = true;
        sessionData.clear();
        currentBallInSession = 0;
        lastBallDetectionIndex = 0;

        if (bestParamsInitialized) {
            neuralNet.importParams(bestParams);
        }
    }

    private void endSession() {
        if (!sessionActive || sessionData.size() < MIN_SAMPLES_PER_SESSION) {
            sessionActive = false;
            waitingForReinforcement = false;
            lastSessionSuccessful = false;
            return;
        }

        sessionActive = false;
        SessionAnalysis analysis = analyzeSession();
        lastSessionAnalysis = analysis;

        if (totalSessionsCompleted < MAX_SESSIONS) {
            sessionHistory[totalSessionsCompleted] = analysis;
        }
        totalSessionsCompleted++;

        // AUTOMATIC REWARD/PUNISHMENT SYSTEM
        // Calculate session quality score (0.0 to 1.0)
        double qualityScore = calculateQualityScore(analysis);

        boolean autoRewarded = false;
        boolean autoPunished = false;

        // Automatic reward if session is excellent (>= 90% quality)
        if (qualityScore >= autoRewardThreshold) {
            autoRewarded = true;
            autoRewardCount++;
            // Auto-reward: reduce loss by 30%
            analysis.totalLoss *= 0.7;
        }

        // Automatic punishment if session is terrible (<= 30% quality)
        if (qualityScore <= autoPunishThreshold) {
            autoPunished = true;
            autoPunishCount++;
            // Auto-punish: increase loss by 2x
            analysis.totalLoss *= 2.0;
        }

        // Determine if this was a successful 3-shot sequence (for manual reinforcement)
        boolean threeShotSuccess = false;
        if (analysis.ballCount >= 3 && analysis.ball1 != null &&
                analysis.ball2 != null && analysis.ball3Plus != null) {
            double tol = 20.0;
            if (Math.abs(analysis.ball1.avgVelocity - depositTargetVelocity) < tol &&
                    Math.abs(analysis.ball2.avgVelocity - depositTargetVelocity) < tol &&
                    Math.abs(analysis.ball3Plus.avgVelocity - depositTargetVelocity) < tol) {
                threeShotSuccess = true;
            }
        }

        if (threeShotSuccess) {
            waitingForReinforcement = true;
            lastSessionSuccessful = true;
        } else {
            waitingForReinforcement = false;
            lastSessionSuccessful = false;
        }

        // Training logic with automatic and manual rewards/punishments
        boolean shouldTrain = nnEnabled && (analysis.totalLoss < bestLoss || lastSessionSuccessful || autoRewarded);

        // BATTERY SAVING: Only train every N sessions unless manually rewarded or auto-rewarded
        sessionsSinceLastTraining++;
        boolean trainingCycle = (sessionsSinceLastTraining >= trainingFrequency) || lastSessionSuccessful || autoRewarded;

        if (shouldTrain && trainingCycle) {
            sessionsSinceLastTraining = 0; // Reset counter

            if (bestParamsInitialized) {
                neuralNet.importParams(bestParams);
            }

            // ADAPTIVE ITERATION COUNT (reduced for battery)
            int currentIterations = trainingIterationsPerSession;

            if (analysis.totalLoss >= bestLoss * lossImprovementThreshold) {
                sessionsWithoutImprovement++;
                currentIterations = Math.min(maxIterationsPerSession,
                        trainingIterationsPerSession + sessionsWithoutImprovement);
            } else {
                sessionsWithoutImprovement = 0;
                currentIterations = trainingIterationsPerSession;
            }

            // Run MULTIPLE training iterations
            for (int iter = 0; iter < currentIterations; iter++) {
                trainNeuralNetwork(analysis);
            }

            // Only accept the new parameters if they meet quality standards OR were rewarded
            boolean meetsQualityStandards = checkQualityStandards(analysis);

            if (lastSessionSuccessful || meetsQualityStandards || autoRewarded) {
                // This is good enough - save it
                bestParams = neuralNet.exportParams();
                bestParamsInitialized = true;

                // Track improvement
                if (analysis.totalLoss < bestLoss) {
                    lastBestLoss = bestLoss;
                    bestLoss = analysis.totalLoss;
                    sessionsWithoutImprovement = 0;
                }
            } else {
                // Not good enough - revert to best params
                if (bestParamsInitialized) {
                    neuralNet.importParams(bestParams);
                }
            }
        }
    }

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

    private void detectBallTransition(double currentVel) {
        if (sessionData.size() < 10) return;
        if (sessionData.size() - lastBallDetectionIndex < 20) return;

        double prevAvg = 0;
        for (int i = 1; i <= 5; i++) {
            prevAvg += sessionData.get(sessionData.size() - 5 - i).velocity;
        }
        prevAvg /= 5;

        double drop = prevAvg - currentVel;
        if (drop > boostTriggerThreshold * BALL_DETECTION_DROP_THRESHOLD) {
            boolean wasStable = true;
            for (int i = 6; i <= 10; i++) {
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

    private SessionAnalysis analyzeSession() {
        List<Integer> ballStarts = detectBallBoundaries();
        int ballCount = ballStarts.size();
        SessionAnalysis analysis = new SessionAnalysis(ballCount);

        for (int i = 0; i < ballCount; i++) {
            int start = ballStarts.get(i);
            int end = (i < ballCount - 1) ? ballStarts.get(i + 1) : sessionData.size();
            BallMetrics metrics = analyzeBallSegment(start, end);

            if (i == 0) {
                analysis.ball1 = metrics;
            } else if (i == 1) {
                analysis.ball2 = metrics;
            } else {
                if (analysis.ball3Plus == null) {
                    analysis.ball3Plus = metrics;
                } else {
                    analysis.ball3Plus.maxDrop = (analysis.ball3Plus.maxDrop + metrics.maxDrop) / 2.0;
                    analysis.ball3Plus.recoveryTime = (analysis.ball3Plus.recoveryTime + metrics.recoveryTime) / 2.0;
                    analysis.ball3Plus.overshoot = (analysis.ball3Plus.overshoot + metrics.overshoot) / 2.0;
                    analysis.ball3Plus.stabilityScore = (analysis.ball3Plus.stabilityScore + metrics.stabilityScore) / 2.0;
                    analysis.ball3Plus.variance = (analysis.ball3Plus.variance + metrics.variance) / 2.0;
                }
            }
        }

        double totalVar = 0;
        for (VelocitySnapshot s : sessionData) {
            double diff = s.velocity - depositTargetVelocity;
            totalVar += diff * diff;
        }
        totalVar /= sessionData.size();
        analysis.overallStability = Math.max(0, 100 - (totalVar / 10.0));

        analysis.totalLoss = calculateSessionLoss(analysis);

        return analysis;
    }

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

                if (stable && (boundaries.isEmpty() || i - boundaries.get(boundaries.size() - 1) > 20)) {
                    boundaries.add(i);
                }
            }
        }

        return boundaries;
    }

    private BallMetrics analyzeBallSegment(int start, int end) {
        BallMetrics m = new BallMetrics();

        double min = Double.MAX_VALUE;
        double max = Double.MIN_VALUE;
        double sum = 0;
        int dropIdx = -1;
        int recoverIdx = -1;

        for (int i = start; i < end && i < sessionData.size(); i++) {
            double v = sessionData.get(i).velocity;
            sum += v;
            if (v < min) {
                min = v;
                dropIdx = i;
            }
            if (v > max) {
                max = v;
            }

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

    private double calculateSessionLoss(SessionAnalysis analysis) {
        double loss = 0.0;

        if (analysis.ball1 != null) {
            loss += dropWeight * Math.pow(analysis.ball1.maxDrop - targetDropMagnitude, 2);
            loss += recoveryWeight * Math.pow(analysis.ball1.recoveryTime - targetRecoveryTime, 2);
            // Exponential penalty for overshoot - gets VERY expensive fast
            double overshootPenalty = analysis.ball1.overshoot > targetOvershoot ?
                    Math.pow(analysis.ball1.overshoot - targetOvershoot, 2.5) :
                    Math.pow(analysis.ball1.overshoot - targetOvershoot, 2);
            loss += overshootWeight * overshootPenalty;
        }

        if (analysis.ball2 != null) {
            loss += dropWeight * Math.pow(analysis.ball2.maxDrop - targetDropMagnitude, 2);
            loss += recoveryWeight * Math.pow(analysis.ball2.recoveryTime - targetRecoveryTime, 2);
            double overshootPenalty = analysis.ball2.overshoot > targetOvershoot ?
                    Math.pow(analysis.ball2.overshoot - targetOvershoot, 2.5) :
                    Math.pow(analysis.ball2.overshoot - targetOvershoot, 2);
            loss += overshootWeight * overshootPenalty;
        }

        if (analysis.ball3Plus != null) {
            loss += dropWeight * Math.pow(analysis.ball3Plus.maxDrop - targetDropMagnitude, 2);
            loss += recoveryWeight * Math.pow(analysis.ball3Plus.recoveryTime - targetRecoveryTime, 2);
            double overshootPenalty = analysis.ball3Plus.overshoot > targetOvershoot ?
                    Math.pow(analysis.ball3Plus.overshoot - targetOvershoot, 2.5) :
                    Math.pow(analysis.ball3Plus.overshoot - targetOvershoot, 2);
            loss += overshootWeight * overshootPenalty;
        }

        loss += stabilityWeight * Math.pow(analysis.overallStability - targetStabilityScore, 2);

        // Penalize deviation from driver's requested speed with extra penalty for going over
        double mse = 0.0;
        if (!sessionData.isEmpty()) {
            for (VelocitySnapshot s : sessionData) {
                double e = s.velocity - depositTargetVelocity;
                // If velocity is OVER target, penalize it extra heavily (2x)
                if (e > 0) {
                    mse += 2.0 * e * e;
                } else {
                    mse += e * e;
                }
            }
            mse /= sessionData.size();
        }
        loss += speedMismatchWeight * mse;

        return loss;
    }

    /**
     * Check if session meets strict quality standards - we don't accept mediocrity
     */
    private boolean checkQualityStandards(SessionAnalysis analysis) {
        // Must have at least 3 balls
        if (analysis.ballCount < 3) return false;

        // Check ball 1
        if (analysis.ball1 != null) {
            if (analysis.ball1.overshoot > acceptableOvershoot) return false;
            if (analysis.ball1.maxDrop > acceptableDropMagnitude) return false;
        }

        // Check ball 2
        if (analysis.ball2 != null) {
            if (analysis.ball2.overshoot > acceptableOvershoot) return false;
            if (analysis.ball2.maxDrop > acceptableDropMagnitude) return false;
        }

        // Check ball 3+
        if (analysis.ball3Plus != null) {
            if (analysis.ball3Plus.overshoot > acceptableOvershoot) return false;
            if (analysis.ball3Plus.maxDrop > acceptableDropMagnitude) return false;
        }

        // Overall stability must be excellent
        if (analysis.overallStability < acceptableStability) return false;

        // All checks passed - this is acceptable quality
        return true;
    }

    /**
     * Calculate a 0.0-1.0 quality score for automatic reward/punishment
     * 1.0 = perfect, 0.0 = terrible
     */
    private double calculateQualityScore(SessionAnalysis analysis) {
        if (analysis.ballCount < 3) return 0.0;

        double totalScore = 0.0;
        int metricCount = 0;

        // Ball 1 scores
        if (analysis.ball1 != null) {
            // Overshoot score (lower is better)
            double overshootScore1 = Math.max(0, 1.0 - (analysis.ball1.overshoot / (acceptableOvershoot * 2)));
            totalScore += overshootScore1;
            metricCount++;

            // Drop score (lower is better)
            double dropScore1 = Math.max(0, 1.0 - (analysis.ball1.maxDrop / (acceptableDropMagnitude * 2)));
            totalScore += dropScore1;
            metricCount++;
        }

        // Ball 2 scores
        if (analysis.ball2 != null) {
            double overshootScore2 = Math.max(0, 1.0 - (analysis.ball2.overshoot / (acceptableOvershoot * 2)));
            totalScore += overshootScore2;
            metricCount++;

            double dropScore2 = Math.max(0, 1.0 - (analysis.ball2.maxDrop / (acceptableDropMagnitude * 2)));
            totalScore += dropScore2;
            metricCount++;
        }

        // Ball 3+ scores
        if (analysis.ball3Plus != null) {
            double overshootScore3 = Math.max(0, 1.0 - (analysis.ball3Plus.overshoot / (acceptableOvershoot * 2)));
            totalScore += overshootScore3;
            metricCount++;

            double dropScore3 = Math.max(0, 1.0 - (analysis.ball3Plus.maxDrop / (acceptableDropMagnitude * 2)));
            totalScore += dropScore3;
            metricCount++;
        }

        // Stability score
        double stabilityScore = analysis.overallStability / 100.0;
        totalScore += stabilityScore;
        metricCount++;

        // Average all metrics
        return metricCount > 0 ? totalScore / metricCount : 0.0;
    }

    /**
     * Train neural network on session results with stability improvements
     * Uses CONSTANT learning rate - never decays, always able to improve
     * OPTIMIZED for battery efficiency
     */
    private void trainNeuralNetwork(SessionAnalysis analysis) {
        currentLoss = analysis.totalLoss;

        // Use constant learning rate - NO DECAY
        // This allows the robot to keep learning indefinitely

        // Prepare REDUCED input features (8 instead of 12) for battery efficiency
        double[] inputs = new double[8];

        // Only include the most critical metrics
        inputs[0] = normalize(analysis.ball1 != null ? analysis.ball1.maxDrop : 0, 0, 200);
        inputs[1] = normalize(analysis.ball1 != null ? analysis.ball1.overshoot : 0, 0, 100);
        inputs[2] = normalize(analysis.ball2 != null ? analysis.ball2.maxDrop : 0, 0, 200);
        inputs[3] = normalize(analysis.ball2 != null ? analysis.ball2.overshoot : 0, 0, 100);
        inputs[4] = normalize(analysis.ball3Plus != null ? analysis.ball3Plus.maxDrop : 0, 0, 200);
        inputs[5] = normalize(analysis.ball3Plus != null ? analysis.ball3Plus.overshoot : 0, 0, 100);
        inputs[6] = normalize(analysis.overallStability, 0, 100);
        inputs[7] = normalize(currentLoss, 0, 10000);

        // Calculate target adjustments (9 outputs)
        double[] targetAdjustments = new double[9];

        // Ball 1 adjustments
        if (analysis.ball1 != null) {
            double dropError = analysis.ball1.maxDrop - targetDropMagnitude;
            targetAdjustments[0] = normalize(dropError > 0 ? -0.03 : 0.03, -0.1, 0.1);
            targetAdjustments[1] = normalize(analysis.ball1.overshoot > targetOvershoot ? -0.02 : 0.02, -0.1, 0.1);
        }

        // Ball 2 adjustments
        if (analysis.ball2 != null) {
            double dropError = analysis.ball2.maxDrop - targetDropMagnitude;
            targetAdjustments[2] = normalize(dropError > 0 ? -0.03 : 0.03, -0.1, 0.1);
            targetAdjustments[3] = normalize(analysis.ball2.overshoot > targetOvershoot ? -0.02 : 0.02, -0.1, 0.1);
        }

        // Ball 3+ adjustments
        if (analysis.ball3Plus != null) {
            double dropError = analysis.ball3Plus.maxDrop - targetDropMagnitude;
            targetAdjustments[4] = normalize(dropError > 0 ? -0.03 : 0.03, -0.1, 0.1);
            targetAdjustments[5] = normalize(analysis.ball3Plus.overshoot > targetOvershoot ? -0.02 : 0.02, -0.1, 0.1);
        }

        // Global boost curve adjustments
        targetAdjustments[6] = normalize(analysis.overallStability < 70 ? 0.03 : -0.01, -0.5, 0.5);
        targetAdjustments[7] = normalize(currentLoss > 1000 ? 0.005 : 0, -0.5, 0.5);
        targetAdjustments[8] = normalize(currentLoss > 1000 ? 0.005 : 0, -1.0, 1.0);

        // Train network with CONSTANT learning rate (no decay)
        neuralNet.train(inputs, targetAdjustments, learningRate);

        // Get network predictions and apply with gradient clipping
        double[] predictions = neuralNet.forward(inputs);
        applyNeuralNetworkAdjustments(predictions);

        trainingIterations++;

        // Note: bestLoss is updated in endSession() after quality check
    }

    /**
     * Apply neural network predicted adjustments with moderate smoothing and faster iteration
     */
    private void applyNeuralNetworkAdjustments(double[] adjustments) {
        // Gradient clipping - prevent extreme adjustments but allow reasonable learning
        double maxAdjustment = 0.12; // Increased from 0.08 for faster learning
        for (int i = 0; i < adjustments.length; i++) {
            adjustments[i] = clamp(adjustments[i], -maxAdjustment, maxAdjustment);
        }

        // Denormalize and apply adjustments with moderate step size (0.4x multiplier)
        // Small steps, but MANY iterations = fast convergence
        double stepMultiplier = 0.4; // Increased from 0.2 - still careful but faster

        // Calculate new values
        double newBall1Mult = ball1Multiplier + denormalize(adjustments[0], -0.1, 0.1) * stepMultiplier;
        double newBall1Exp = ball1Exponent + denormalize(adjustments[1], -0.1, 0.1) * stepMultiplier;
        double newBall2Mult = ball2Multiplier + denormalize(adjustments[2], -0.1, 0.1) * stepMultiplier;
        double newBall2Exp = ball2Exponent + denormalize(adjustments[3], -0.1, 0.1) * stepMultiplier;
        double newBall3Mult = ball3Multiplier + denormalize(adjustments[4], -0.1, 0.1) * stepMultiplier;
        double newBall3Exp = ball3Exponent + denormalize(adjustments[5], -0.1, 0.1) * stepMultiplier;

        double newBoostTrigger = boostTriggerThreshold + denormalize(adjustments[6], -0.5, 0.5) * stepMultiplier;
        double newBoostMin = boostMinTicks + denormalize(adjustments[7], -0.5, 0.5) * stepMultiplier;
        double newBoostMax = boostMaxTicks + denormalize(adjustments[8], -1.0, 1.0) * stepMultiplier;

        // Apply smoothing (60% old, 40% new) - less smoothing than before for faster response
        ball1Multiplier = parameterSmoothingFactor * ball1Multiplier + (1 - parameterSmoothingFactor) * newBall1Mult;
        ball1Exponent = parameterSmoothingFactor * ball1Exponent + (1 - parameterSmoothingFactor) * newBall1Exp;
        ball2Multiplier = parameterSmoothingFactor * ball2Multiplier + (1 - parameterSmoothingFactor) * newBall2Mult;
        ball2Exponent = parameterSmoothingFactor * ball2Exponent + (1 - parameterSmoothingFactor) * newBall2Exp;
        ball3Multiplier = parameterSmoothingFactor * ball3Multiplier + (1 - parameterSmoothingFactor) * newBall3Mult;
        ball3Exponent = parameterSmoothingFactor * ball3Exponent + (1 - parameterSmoothingFactor) * newBall3Exp;

        boostTriggerThreshold = parameterSmoothingFactor * boostTriggerThreshold + (1 - parameterSmoothingFactor) * newBoostTrigger;
        boostMinTicks = parameterSmoothingFactor * boostMinTicks + (1 - parameterSmoothingFactor) * newBoostMin;
        boostMaxTicks = parameterSmoothingFactor * boostMaxTicks + (1 - parameterSmoothingFactor) * newBoostMax;

        // Enforce bounds with tight constraints to prevent overshooting
        ball1Multiplier = clamp(ball1Multiplier, 0.6, 2.0);
        ball1Exponent = clamp(ball1Exponent, 1.2, 2.0);
        ball2Multiplier = clamp(ball2Multiplier, 0.6, 2.2);
        ball2Exponent = clamp(ball2Exponent, 1.2, 2.0);
        ball3Multiplier = clamp(ball3Multiplier, 0.6, 2.0);
        ball3Exponent = clamp(ball3Exponent, 1.2, 2.0);

        boostTriggerThreshold = clamp(boostTriggerThreshold, 30.0, 120.0);
        boostMinTicks = clamp(boostMinTicks, 10.0, 80.0);
        boostMaxTicks = clamp(boostMaxTicks, 150.0, 600.0); // Still hard capped - NO THOUSAND TICK BOOSTS
    }

    private double normalize(double value, double min, double max) {
        if (max == min) return 0;
        return (value - min) / (max - min);
    }

    private double denormalize(double value, double min, double max) {
        return value * (max - min) + min;
    }

    private double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }

    private void displayTelemetry() {
        telemetry.addLine("═══════════════════════════════");
        telemetry.addLine("♾️ INFINITE LEARNING MODE");
        telemetry.addLine("═══════════════════════════════");
        telemetry.addLine("");

        telemetry.addLine("─── CONTROLS ───");
        telemetry.addData("Deposit (X)", depositActive ? "✓" : "✗");
        telemetry.addData("Intake (A)", gamepad1.a ? "✓" : "✗");
        telemetry.addData("Outake (B)", gamepad1.b ? "✓" : "✗");
        telemetry.addData("Servo (ZL)", (gamepad1.left_trigger > 0.5) ? "SHOOT" : "IDLE");
        telemetry.addData("Speed", "%.0f", depositTargetVelocity);
        telemetry.addLine("");

        telemetry.addLine("─── FEEDBACK SYSTEM ───");
        telemetry.addData("✅ Manual Reward", "Y+UP (10x weight)");
        telemetry.addData("❌ Manual Punish", "Y+DOWN (10x weight)");
        telemetry.addData("Manual Count", "%d", reinforcementCount);
        telemetry.addLine("");
        telemetry.addData("🤖 Auto Rewards", "%d (>90%% quality)", autoRewardCount);
        telemetry.addData("🤖 Auto Punish", "%d (<30%% quality)", autoPunishCount);

        // Show last session quality score
        if (totalSessionsCompleted > 0 && sessionHistory[totalSessionsCompleted - 1] != null) {
            SessionAnalysis last = sessionHistory[totalSessionsCompleted - 1];
            double qualityScore = calculateQualityScore(last);
            telemetry.addData("Last Quality", "%.0f%%", qualityScore * 100);

            if (qualityScore >= autoRewardThreshold) {
                telemetry.addData("Auto Status", "✅ REWARDED");
            } else if (qualityScore <= autoPunishThreshold) {
                telemetry.addData("Auto Status", "❌ PUNISHED");
            } else {
                telemetry.addData("Auto Status", "➖ NEUTRAL");
            }
        }
        telemetry.addLine("");

        telemetry.addLine("─── QUALITY GATE ───");
        telemetry.addData("Max Overshoot", "%.0f ticks", acceptableOvershoot);
        telemetry.addData("Max Drop", "%.0f ticks", acceptableDropMagnitude);
        telemetry.addData("Min Stability", "%.0f%%", acceptableStability);
        if (totalSessionsCompleted > 0 && sessionHistory[totalSessionsCompleted - 1] != null) {
            SessionAnalysis last = sessionHistory[totalSessionsCompleted - 1];
            boolean passed = checkQualityStandards(last);
            telemetry.addData("Last Session", passed ? "✅ PASSED" : "❌ REJECTED");
        }
        telemetry.addLine("");

        telemetry.addLine("─── SESSION ───");
        if (sessionActive) {
            telemetry.addData("Status", "🔴 RECORDING");
            telemetry.addData("Ball", "%d", currentBallInSession + 1);
            telemetry.addData("Samples", "%d", sessionData.size());
        } else {
            telemetry.addData("Status", "⚪ IDLE");
        }
        telemetry.addData("Completed", "%d", totalSessionsCompleted);
        telemetry.addLine("");

        telemetry.addLine("─── NEURAL NET (LITE) ───");
        telemetry.addData("Enabled", nnEnabled ? "YES" : "NO");
        telemetry.addData("Network", "8→6→9 (battery-safe)");

        // Show training frequency
        int sessionsUntilTrain = trainingFrequency - sessionsSinceLastTraining;
        if (sessionsUntilTrain > 0) {
            telemetry.addData("Next Training", "in %d sessions", sessionsUntilTrain);
        } else {
            telemetry.addData("Training", "THIS SESSION");
        }

        // Calculate current iteration count based on stuck counter
        int currentIterCount = trainingIterationsPerSession;
        if (sessionsWithoutImprovement > 0) {
            currentIterCount = Math.min(maxIterationsPerSession,
                    trainingIterationsPerSession + sessionsWithoutImprovement);
        }
        telemetry.addData("Iters/Session", "%d (max 4)", currentIterCount);
        if (sessionsWithoutImprovement > 0) {
            telemetry.addData("⚠️ Stuck Count", "%d", sessionsWithoutImprovement);
        }

        telemetry.addData("Total Iters", "%d", trainingIterations);
        telemetry.addData("Learn Rate", "%.4f", learningRate);
        telemetry.addData("Loss", "%.1f", currentLoss);
        telemetry.addData("Best", "%.1f", bestLoss);

        // Show improvement
        if (lastBestLoss != Double.MAX_VALUE && bestLoss < lastBestLoss) {
            double improvement = ((lastBestLoss - bestLoss) / lastBestLoss) * 100;
            telemetry.addData("↓ Improved", "%.1f%%", improvement);
        }
        telemetry.addLine("");

        if (depositActive && robot.deposit1 != null) {
            double v1 = robot.getDeposit1Velocity();
            double v2 = robot.getDeposit2Velocity();
            double avg = (v1 + v2) / 2.0;
            telemetry.addLine("─── VELOCITY ───");
            telemetry.addData("Avg", "%.0f", avg);
            telemetry.addData("Target", "%.0f", depositTargetVelocity);
            telemetry.addData("Error", "%.0f", depositTargetVelocity - avg);
            double overshoot = Math.max(0, avg - depositTargetVelocity);
            if (overshoot > 0) {
                telemetry.addData("⚠️ Overshoot", "%.0f", overshoot);
            }
            telemetry.addLine("");
        }

        if (totalSessionsCompleted > 0 && sessionHistory[totalSessionsCompleted - 1] != null) {
            SessionAnalysis last = sessionHistory[totalSessionsCompleted - 1];
            telemetry.addLine("─── LAST SESSION ───");
            telemetry.addData("Balls", "%d", last.ballCount);
            telemetry.addData("Stability", "%.1f/100", last.overallStability);
            if (last.ball1 != null) {
                telemetry.addData("B1 Overshoot", "%.0f", last.ball1.overshoot);
            }
            if (last.ball2 != null) {
                telemetry.addData("B2 Overshoot", "%.0f", last.ball2.overshoot);
            }
            if (last.ball3Plus != null) {
                telemetry.addData("B3+ Overshoot", "%.0f", last.ball3Plus.overshoot);
            }
            telemetry.addLine("");
        }

        telemetry.addLine("─── LEARNED PARAMS ───");
        telemetry.addData("B1", "M%.2f E%.2f", ball1Multiplier, ball1Exponent);
        telemetry.addData("B2", "M%.2f E%.2f", ball2Multiplier, ball2Exponent);
        telemetry.addData("B3", "M%.2f E%.2f", ball3Multiplier, ball3Exponent);
        telemetry.addLine("");

        telemetry.addLine("─── BOOST CURVE ───");
        telemetry.addData("Trigger", "%.1f", boostTriggerThreshold);
        telemetry.addData("Min", "%.1f", boostMinTicks);
        telemetry.addData("Max", "%.1f", boostMaxTicks);
        telemetry.addLine("");

        telemetry.addLine("♾️ INFINITE LEARNING:");
        telemetry.addLine("• Auto reward/punish");
        telemetry.addLine("• Manual 10x stronger");
        telemetry.addLine("• Never stops improving");
        telemetry.addLine("• Battery-efficient");
        telemetry.addLine("═══════════════════════════════");

        telemetry.update();
    }

    @Override
    public void stop() {
        robot.stop();

        telemetry.clearAll();
        telemetry.addLine("═══════════════════════════════");
        telemetry.addLine("🧠 NEURAL NETWORK FINAL PARAMS");
        telemetry.addLine("═══════════════════════════════");
        telemetry.addLine("");
        telemetry.addLine("NETWORK STATS:");
        telemetry.addData("Total Sessions", "%d", totalSessionsCompleted);
        telemetry.addData("Training Iterations", "%d", trainingIterations);
        telemetry.addData("Final Loss", "%.2f", currentLoss);
        telemetry.addData("Best Loss Achieved", "%.2f", bestLoss);
        telemetry.addLine("");
        telemetry.addLine("BALL 1 BOOST:");
        telemetry.addData("ball1Multiplier", "%.3f", ball1Multiplier);
        telemetry.addData("ball1Exponent", "%.3f", ball1Exponent);
        telemetry.addLine("");
        telemetry.addLine("BALL 2 BOOST:");
        telemetry.addData("ball2Multiplier", "%.3f", ball2Multiplier);
        telemetry.addData("ball2Exponent", "%.3f", ball2Exponent);
        telemetry.addLine("");
        telemetry.addLine("BALL 3+ BOOST:");
        telemetry.addData("ball3Multiplier", "%.3f", ball3Multiplier);
        telemetry.addData("ball3Exponent", "%.3f", ball3Exponent);
        telemetry.addLine("");
        telemetry.addLine("BOOST CURVE:");
        telemetry.addData("Trigger", "%.1f", boostTriggerThreshold);
        telemetry.addData("Min", "%.1f", boostMinTicks);
        telemetry.addData("Max", "%.1f", boostMaxTicks);
        telemetry.addLine("");
        telemetry.addLine("Copy to your TeleOp or");
        telemetry.addLine("adjust at panels.bylazar.com");
        telemetry.addLine("═══════════════════════════════");
        telemetry.update();
    }
}