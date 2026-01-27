package org.firstinspires.ftc.teamcode.Trowel.Drive;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.Trowel.Configs.TrowelHardware;
import org.firstinspires.ftc.teamcode.Trowel.Configs.RandyButterNubs;
import org.firstinspires.ftc.teamcode.Trowel.common.Odometry;
import org.firstinspires.ftc.teamcode.Trowel.pedroPathing.Constants;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import java.util.ArrayList;
import java.util.List;
import java.util.Random;

@TeleOp(name = "ML Auto-Tune Deposit", group = "Trowel")
public class AdvancedTeleop extends OpMode {

    // ============== SERVO CONFIGURATION ==============
    public double SERVO_POSITION_SHOOTING = 0.7;
    public double SERVO_POSITION_IDLE = 0.3;

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

    // ============== PIDF PARAMETERS ==============
    public double kP = 10.0;
    public double kI = 0.0;
    public double kD = 0.0;
    public double kF = 0.0;

    // ============== NEURAL NETWORK CONFIGURATION ==============
    public boolean nnEnabled = true;
    public double learningRate = 0.05;
    public int inputNeurons = 12;
    public int hiddenNeurons = 10;
    public int outputNeurons = 13;

    // Training parameters
    public double targetDropMagnitude = 30.0;
    public double targetRecoveryTime = 0.2;
    public double targetStabilityScore = 90.0;
    public double targetOvershoot = 5.0;

    // Loss weights
    public double dropWeight = 2.0;
    public double recoveryWeight = 1.5;
    public double stabilityWeight = 1.0;
    public double overshootWeight = 1.2;

    // ============== SESSION RECORDING ==============
    public int MAX_SESSIONS = 100;
    public int MIN_SAMPLES_PER_SESSION = 10;
    public double BALL_DETECTION_DROP_THRESHOLD = 1.5;

    // ============== INTERNAL STATE ==============
    private boolean sessionActive = false;
    private boolean lastZLState = false;
    private List<VelocitySnapshot> sessionData = new ArrayList<>();
    private int currentBallInSession = 0;
    private int lastBallDetectionIndex = 0;

    private int totalSessionsCompleted = 0;
    private SessionAnalysis[] sessionHistory;

    private TrowelHardware robot;
    private RandyButterNubs drive;
    private boolean depositActive = false;
    private boolean lastXState = false;

    private boolean lastDpadUp = false;
    private boolean lastDpadDown = false;
    private boolean lastDpadLeft = false;
    private boolean lastDpadRight = false;

    // Neural Network
    private NeuralNetwork neuralNet;
    private double currentLoss = 0.0;
    private double bestLoss = Double.MAX_VALUE;
    private int trainingIterations = 0;

    // Odometry and Auto-Aiming
    private Odometry odometry;
    private boolean odometryEnabled = false;
    private Follower follower = null;

    // Auto-aim state
    private boolean hasSetScoringPosition = false;
    private Pose scoringPose = null;
    private Pose lastKnownPose = new Pose(0, 0, 0);
    private boolean prevDriver1LB = false;
    private boolean prevDriver1ZL = false;
    private boolean prevDriver1ZR = false;

    /**
     * Simple 3-layer Neural Network
     */
    private static class NeuralNetwork {
        private double[][] weightsInputHidden;
        private double[][] weightsHiddenOutput;
        private double[] biasHidden;
        private double[] biasOutput;

        private double[] hiddenLayer;
        private double[] outputLayer;

        private int inputSize;
        private int hiddenSize;
        private int outputSize;

        private Random random = new Random();

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

            initializeWeights();
        }

        private void initializeWeights() {
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
         * Forward pass through network
         */
        public double[] forward(double[] input) {
            // Input to hidden
            for (int j = 0; j < hiddenSize; j++) {
                hiddenLayer[j] = biasHidden[j];
                for (int i = 0; i < inputSize; i++) {
                    hiddenLayer[j] += input[i] * weightsInputHidden[i][j];
                }
                hiddenLayer[j] = relu(hiddenLayer[j]);
            }

            // Hidden to output
            for (int j = 0; j < outputSize; j++) {
                outputLayer[j] = biasOutput[j];
                for (int i = 0; i < hiddenSize; i++) {
                    outputLayer[j] += hiddenLayer[i] * weightsHiddenOutput[i][j];
                }
                outputLayer[j] = tanh(outputLayer[j]); // Use tanh for bounded outputs
            }

            return outputLayer.clone();
        }

        /**
         * Backpropagation training
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
                hiddenDeltas[i] = hiddenErrors[i] * reluDerivative(hiddenLayer[i]);
            }

            // Update weights and biases (hidden to output)
            for (int i = 0; i < hiddenSize; i++) {
                for (int j = 0; j < outputSize; j++) {
                    weightsHiddenOutput[i][j] += learningRate * outputDeltas[j] * hiddenLayer[i];
                }
            }
            for (int i = 0; i < outputSize; i++) {
                biasOutput[i] += learningRate * outputDeltas[i];
            }

            // Update weights and biases (input to hidden)
            for (int i = 0; i < inputSize; i++) {
                for (int j = 0; j < hiddenSize; j++) {
                    weightsInputHidden[i][j] += learningRate * hiddenDeltas[j] * input[i];
                }
            }
            for (int i = 0; i < hiddenSize; i++) {
                biasHidden[i] += learningRate * hiddenDeltas[i];
            }
        }

        private double relu(double x) {
            return Math.max(0, x);
        }

        private double reluDerivative(double x) {
            return x > 0 ? 1.0 : 0.0;
        }

        private double tanh(double x) {
            return Math.tanh(x);
        }

        private double tanhDerivative(double x) {
            return 1.0 - x * x;
        }
    }

    /**
     * Velocity snapshot
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
     * Session analysis
     */
    private static class SessionAnalysis {
        int ballCount;
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
        drive = new RandyButterNubs(robot.frontLeft, robot.frontRight, robot.backLeft, robot.backRight);
        robot.resetDepositEncoders();

        sessionHistory = new SessionAnalysis[MAX_SESSIONS];

        // Initialize neural network
        neuralNet = new NeuralNetwork(inputNeurons, hiddenNeurons, outputNeurons);

        // Initialize follower
        try {
            follower = Constants.createFollower(hardwareMap);
        } catch (Exception ignored) {
            follower = null;
        }

        // Initialize Pinpoint odometry
        try {
            robot.initPinpoint();
            if (robot.pinpoint != null) {
                odometryEnabled = true;
                telemetry.addLine("✓ Pinpoint Odometry Enabled");
            }
        } catch (Exception e) {
            telemetry.addLine("✗ Pinpoint Not Found - Odometry Disabled");
            odometryEnabled = false;
        }

        if (robot.frontLeft != null) robot.frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        if (robot.frontRight != null) robot.frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        if (robot.backLeft != null) robot.backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        if (robot.backRight != null) robot.backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        if (robot.transferServo != null) {
            robot.transferServo.setPosition(SERVO_POSITION_IDLE);
        }

        updatePIDFCoefficients();

        telemetry.addLine("═══════════════════════════════");
        telemetry.addLine("🧠 NEURAL NETWORK AUTO-TUNE");
        telemetry.addLine("═══════════════════════════════");
        telemetry.addLine("");
        telemetry.addLine("Network: 12→10→13 neurons");
        telemetry.addLine("");
        telemetry.addLine("CONTROLS:");
        telemetry.addLine("X = Toggle Deposit");
        telemetry.addLine("ZL = Set Position + Shoot");
        telemetry.addLine("ZR = Servo Shooting Position");
        telemetry.addLine("A = Run Intakes");
        telemetry.addLine("LB = Auto-drive to position");
        telemetry.addLine("");
        telemetry.addLine("Configure: panels.bylazar.com");
        telemetry.addLine("═══════════════════════════════");
        telemetry.update();
    }

    @Override
    public void init_loop() {
        // No team selection needed
    }

    @Override
    public void start() {
        // No team-specific setup needed
    }

    @Override
    public void loop() {
        // ============== UPDATE ODOMETRY ==============
        if (odometryEnabled && robot.pinpoint != null) {
            robot.updatePinpoint();
            if (odometry == null) {
                odometry = new Odometry(robot, robot.pinpoint);
                try {
                    java.lang.reflect.Field hRadField = Odometry.class.getDeclaredField("hRad");
                    hRadField.setAccessible(true);
                    hRadField.set(odometry, Math.toRadians(90));
                } catch (Exception ignored) {}
            }
            odometry.update();
        } else if (odometry == null) {
            odometry = new Odometry(robot, null);
            try {
                java.lang.reflect.Field hRadField = Odometry.class.getDeclaredField("hRad");
                hRadField.setAccessible(true);
                hRadField.set(odometry, Math.toRadians(90));
            } catch (Exception ignored) {}
        }

        // Update cached lastKnownPose
        try {
            if (odometry != null) {
                Odometry.Position pos = odometry.getPosition();
                lastKnownPose = new Pose(pos.xMm / 25.4, pos.yMm / 25.4, pos.headingRad);
            } else if (follower != null) {
                lastKnownPose = follower.getPose();
            }
        } catch (Exception ignored) {}

        // ============== DRIVE INPUT ==============
        double forward = -gamepad1.left_stick_y * DRIVE_POWER_SCALE;
        double strafe = gamepad1.left_stick_x * STRAFE_POWER_SCALE;
        double rotate = gamepad1.right_stick_x * ROTATE_POWER_SCALE;

        // Manual input override for follower
        boolean hasManualInput = Math.abs(forward) > 0.05 || Math.abs(strafe) > 0.05 || Math.abs(rotate) > 0.05;
        if (follower != null && hasManualInput) {
            follower.breakFollowing();
            follower = null;
        }

        // ============== ZL: SET SCORING POSITION (NO SERVO) ==============
        boolean zlPressed = gamepad1.left_trigger > 0.5;
        boolean zlJustPressed = zlPressed && !prevDriver1ZL;

        if (zlJustPressed) {
            // Set/update scoring position
            if (odometryEnabled && odometry != null) {
                Odometry.Position pos = odometry.getPosition();
                double headingRad = pos.headingRad;
                scoringPose = new Pose(pos.xMm / 25.4, pos.yMm / 25.4, headingRad);
                hasSetScoringPosition = true;
            } else if (follower != null) {
                follower.update();
                Pose pose = follower.getPose();
                scoringPose = new Pose(pose.getX(), pose.getY(), pose.getHeading());
                hasSetScoringPosition = true;
            }
        }

        prevDriver1ZL = zlPressed;

        // ============== ZR: SERVO CONTROL (HOLD TO SHOOT) ==============
        boolean zrPressed = gamepad1.right_trigger > 0.5;

        if (zrPressed) {
            // Shooting position
            if (robot.transferServo != null) {
                robot.transferServo.setPosition(SERVO_POSITION_SHOOTING);
            }
        } else {
            // Idle position
            if (robot.transferServo != null) {
                robot.transferServo.setPosition(SERVO_POSITION_IDLE);
            }
        }

        prevDriver1ZR = zrPressed;

        // ============== A BUTTON: RUN INTAKES ==============
        boolean aPressed = gamepad1.a;

        if (aPressed) {
            if (robot.intake1 != null) robot.intake1.setPower(1.0);
            if (robot.intake2 != null) robot.intake2.setPower(-INTAKE2_SCALE);
        } else {
            if (robot.intake1 != null) robot.intake1.setPower(0.0);
            if (robot.intake2 != null) robot.intake2.setPower(0.0);
        }

        // ============== LB: AUTO-DRIVE TO SCORING POSITION ==============
        boolean lbPressed = gamepad1.left_bumper;
        boolean lbJustPressed = lbPressed && !prevDriver1LB;

        if (lbJustPressed && hasSetScoringPosition && scoringPose != null) {
            Pose startPose;
            try {
                if (odometryEnabled && odometry != null) {
                    Odometry.Position pos = odometry.getPosition();
                    startPose = new Pose(pos.xMm / 25.4, pos.yMm / 25.4, pos.headingRad);
                } else if (follower != null) {
                    follower.update();
                    startPose = follower.getPose();
                } else if (lastKnownPose != null) {
                    startPose = lastKnownPose;
                } else {
                    // Default start pose (center of field)
                    startPose = new Pose(72.0, 72.0, 0);
                }

                if (follower == null) {
                    follower = Constants.createFollower(hardwareMap);
                }
                follower.setMaxPower(1.0);
                follower.setStartingPose(startPose);

                try {
                    com.pedropathing.paths.PathChain pathToScoring = follower.pathBuilder()
                            .addPath(new com.pedropathing.geometry.BezierLine(startPose, scoringPose))
                            .setLinearHeadingInterpolation(startPose.getHeading(), scoringPose.getHeading())
                            .build();
                    follower.followPath(pathToScoring);
                } catch (Exception e) {
                    telemetry.addLine("Path error: " + e.getMessage());
                }
            } catch (Exception e) {
                telemetry.addLine("Follower error: " + e.getMessage());
            }
        }
        prevDriver1LB = lbPressed;

        // ============== DRIVE CONTROL ==============
        if (follower != null) {
            try {
                follower.update();
                if (!follower.isBusy()) {
                    follower.setTeleOpDrive(forward, -strafe, -rotate, true);
                }
            } catch (Exception e) {
                drive.drive(forward, strafe, rotate, false, gamepad1.right_bumper);
            }
        } else {
            drive.drive(forward, strafe, rotate, false, gamepad1.right_bumper);
        }

        // ============== SPEED ADJUSTMENT ==============
        boolean yPressed = gamepad1.y;
        boolean dpadUp = gamepad1.dpad_up;
        boolean dpadDown = gamepad1.dpad_down;
        boolean dpadLeft = gamepad1.dpad_left;
        boolean dpadRight = gamepad1.dpad_right;

        if (!yPressed) {
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

        // ============== SESSION RECORDING (removed separate ZL session control) ==============
        // Session starts/stops with ZL, which now also controls servo and intakes
        if (zlJustPressed && depositActive) {
            startSession();
        } else if (!zlPressed && lastZLState) {
            if (sessionActive) endSession();
        }
        lastZLState = zlPressed;

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

            robot.setDepositVelocity(depositTargetVelocity + appliedBoost);
        }

        displayTelemetry();
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

    private void startSession() {
        sessionActive = true;
        sessionData.clear();
        currentBallInSession = 0;
        lastBallDetectionIndex = 0;
    }

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

        if (nnEnabled) {
            trainNeuralNetwork(analysis);
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

    private double calculateSessionLoss(SessionAnalysis analysis) {
        double loss = 0.0;

        if (analysis.ball1 != null) {
            loss += dropWeight * Math.pow(analysis.ball1.maxDrop - targetDropMagnitude, 2);
            loss += recoveryWeight * Math.pow(analysis.ball1.recoveryTime - targetRecoveryTime, 2);
            loss += overshootWeight * Math.pow(analysis.ball1.overshoot - targetOvershoot, 2);
        }

        if (analysis.ball2 != null) {
            loss += dropWeight * Math.pow(analysis.ball2.maxDrop - targetDropMagnitude, 2);
            loss += recoveryWeight * Math.pow(analysis.ball2.recoveryTime - targetRecoveryTime, 2);
            loss += overshootWeight * Math.pow(analysis.ball2.overshoot - targetOvershoot, 2);
        }

        if (analysis.ball3Plus != null) {
            loss += dropWeight * Math.pow(analysis.ball3Plus.maxDrop - targetDropMagnitude, 2);
            loss += recoveryWeight * Math.pow(analysis.ball3Plus.recoveryTime - targetRecoveryTime, 2);
            loss += overshootWeight * Math.pow(analysis.ball3Plus.overshoot - targetOvershoot, 2);
        }

        loss += stabilityWeight * Math.pow(analysis.overallStability - targetStabilityScore, 2);

        return loss;
    }

    /**
     * Train neural network on session results
     */
    private void trainNeuralNetwork(SessionAnalysis analysis) {
        currentLoss = analysis.totalLoss;

        // Prepare input features (12 inputs)
        double[] inputs = new double[12];
        inputs[0] = normalize(analysis.ball1 != null ? analysis.ball1.maxDrop : 0, 0, 200);
        inputs[1] = normalize(analysis.ball1 != null ? analysis.ball1.recoveryTime : 0, 0, 1);
        inputs[2] = normalize(analysis.ball1 != null ? analysis.ball1.overshoot : 0, 0, 100);
        inputs[3] = normalize(analysis.ball2 != null ? analysis.ball2.maxDrop : 0, 0, 200);
        inputs[4] = normalize(analysis.ball2 != null ? analysis.ball2.recoveryTime : 0, 0, 1);
        inputs[5] = normalize(analysis.ball2 != null ? analysis.ball2.overshoot : 0, 0, 100);
        inputs[6] = normalize(analysis.ball3Plus != null ? analysis.ball3Plus.maxDrop : 0, 0, 200);
        inputs[7] = normalize(analysis.ball3Plus != null ? analysis.ball3Plus.recoveryTime : 0, 0, 1);
        inputs[8] = normalize(analysis.ball3Plus != null ? analysis.ball3Plus.overshoot : 0, 0, 100);
        inputs[9] = normalize(analysis.overallStability, 0, 100);
        inputs[10] = normalize(depositTargetVelocity, 0, 1500);
        inputs[11] = normalize(currentLoss, 0, 10000);

        // Calculate target adjustments (13 outputs)
        double[] targetAdjustments = new double[13];

        // Ball 1 adjustments
        if (analysis.ball1 != null) {
            double dropError = analysis.ball1.maxDrop - targetDropMagnitude;
            targetAdjustments[0] = normalize(dropError > 0 ? 0.05 : -0.05, -0.1, 0.1); // ball1Multiplier
            targetAdjustments[1] = normalize(analysis.ball1.recoveryTime > targetRecoveryTime ? 0.03 : -0.03, -0.1, 0.1); // ball1Exponent
        }

        // Ball 2 adjustments
        if (analysis.ball2 != null) {
            double dropError = analysis.ball2.maxDrop - targetDropMagnitude;
            targetAdjustments[2] = normalize(dropError > 0 ? 0.05 : -0.05, -0.1, 0.1);
            targetAdjustments[3] = normalize(analysis.ball2.recoveryTime > targetRecoveryTime ? 0.03 : -0.03, -0.1, 0.1);
        }

        // Ball 3+ adjustments
        if (analysis.ball3Plus != null) {
            double dropError = analysis.ball3Plus.maxDrop - targetDropMagnitude;
            targetAdjustments[4] = normalize(dropError > 0 ? 0.05 : -0.05, -0.1, 0.1);
            targetAdjustments[5] = normalize(analysis.ball3Plus.recoveryTime > targetRecoveryTime ? 0.03 : -0.03, -0.1, 0.1);
        }

        // PIDF adjustments based on stability
        targetAdjustments[6] = normalize(analysis.overallStability < 70 ? 0.5 : -0.2, -1, 1); // kP
        targetAdjustments[7] = normalize(currentLoss > 1000 ? 0.01 : 0, -0.05, 0.05); // kI
        targetAdjustments[8] = normalize(analysis.overallStability < 70 ? 0.05 : -0.02, -0.5, 0.5); // kD
        targetAdjustments[9] = normalize(currentLoss > 500 ? 0.001 : 0, -0.01, 0.01); // kF

        // Boost limits
        targetAdjustments[10] = 0; // boostTriggerThreshold - usually don't adjust
        targetAdjustments[11] = 0; // boostMinTicks
        targetAdjustments[12] = 0; // boostMaxTicks

        // Train network
        neuralNet.train(inputs, targetAdjustments, learningRate);

        // Get network predictions and apply
        double[] predictions = neuralNet.forward(inputs);
        applyNeuralNetworkAdjustments(predictions);

        trainingIterations++;

        if (currentLoss < bestLoss) {
            bestLoss = currentLoss;
        }
    }

    /**
     * Apply neural network predicted adjustments
     */
    private void applyNeuralNetworkAdjustments(double[] adjustments) {
        // Denormalize and apply adjustments
        ball1Multiplier += denormalize(adjustments[0], -0.1, 0.1);
        ball1Exponent += denormalize(adjustments[1], -0.1, 0.1);
        ball2Multiplier += denormalize(adjustments[2], -0.1, 0.1);
        ball2Exponent += denormalize(adjustments[3], -0.1, 0.1);
        ball3Multiplier += denormalize(adjustments[4], -0.1, 0.1);
        ball3Exponent += denormalize(adjustments[5], -0.1, 0.1);
        kP += denormalize(adjustments[6], -1, 1);
        kI += denormalize(adjustments[7], -0.05, 0.05);
        kD += denormalize(adjustments[8], -0.5, 0.5);
        kF += denormalize(adjustments[9], -0.01, 0.01);

        // Enforce bounds
        ball1Multiplier = clamp(ball1Multiplier, 0.3, 5.0);
        ball1Exponent = clamp(ball1Exponent, 0.8, 3.5);
        ball2Multiplier = clamp(ball2Multiplier, 0.3, 5.0);
        ball2Exponent = clamp(ball2Exponent, 0.8, 3.5);
        ball3Multiplier = clamp(ball3Multiplier, 0.3, 5.0);
        ball3Exponent = clamp(ball3Exponent, 0.8, 3.5);
        kP = clamp(kP, 0.0, 50.0);
        kI = clamp(kI, 0.0, 5.0);
        kD = clamp(kD, 0.0, 10.0);
        kF = clamp(kF, 0.0, 1.0);

        updatePIDFCoefficients();
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

    private void updatePIDFCoefficients() {
        if (robot.deposit1 != null) {
            robot.deposit1.setVelocityPIDFCoefficients(kP, kI, kD, kF);
        }
        if (robot.deposit2 != null) {
            robot.deposit2.setVelocityPIDFCoefficients(kP, kI, kD, kF);
        }
    }

    private void displayTelemetry() {
        telemetry.addLine("═══════════════════════════════");
        telemetry.addLine("🧠 NEURAL NETWORK AUTO-TUNE");
        telemetry.addLine("═══════════════════════════════");
        telemetry.addLine("");

        // Scoring position status
        if (!hasSetScoringPosition) {
            telemetry.addLine("⚠ Press ZL at scoring spot");
        } else if (scoringPose != null) {
            telemetry.addData("Scoring Pos", "✓ (%.1f, %.1f) @ %.1f°",
                    scoringPose.getX(), scoringPose.getY(), Math.toDegrees(scoringPose.getHeading()));
        }

        if (follower != null && follower.isBusy() && scoringPose != null) {
            telemetry.addData("Auto-Drive", "✓ ACTIVE");
            Pose currentPose = follower.getPose();
            double distToTarget = Math.sqrt(
                    Math.pow(scoringPose.getX() - currentPose.getX(), 2) +
                            Math.pow(scoringPose.getY() - currentPose.getY(), 2)
            );
            telemetry.addData("Distance", "%.1f in", distToTarget);
        }
        telemetry.addLine("");

        telemetry.addLine("─── CONTROLS ───");
        telemetry.addData("Deposit (X)", depositActive ? "✓" : "✗");
        telemetry.addData("Servo (ZR)", (gamepad1.right_trigger > 0.5) ? "SHOOT" : "IDLE");
        telemetry.addData("Intakes (A)", gamepad1.a ? "✓" : "✗");
        telemetry.addData("Speed", "%.0f", depositTargetVelocity);
        telemetry.addLine("ZL: set pos | LB: auto-drive");
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

        telemetry.addLine("─── NEURAL NET ───");
        telemetry.addData("Enabled", nnEnabled ? "YES" : "NO");
        telemetry.addData("Architecture", "12→10→13");
        telemetry.addData("Iterations", "%d", trainingIterations);
        telemetry.addData("Loss", "%.1f", currentLoss);
        telemetry.addData("Best", "%.1f", bestLoss);
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
            telemetry.addData("Stability", "%.1f/100", last.overallStability);
            telemetry.addLine("");
        }

        telemetry.addLine("─── LEARNED PARAMS ───");
        telemetry.addData("B1", "M%.2f E%.2f", ball1Multiplier, ball1Exponent);
        telemetry.addData("B2", "M%.2f E%.2f", ball2Multiplier, ball2Exponent);
        telemetry.addData("B3", "M%.2f E%.2f", ball3Multiplier, ball3Exponent);
        telemetry.addData("PIDF", "%.1f/%.3f/%.2f/%.4f", kP, kI, kD, kF);
        telemetry.addLine("");

        // Odometry position
        if (odometryEnabled && odometry != null) {
            Odometry.Position pos = odometry.getPosition();
            telemetry.addLine("─── POSITION ───");
            telemetry.addData("X, Y", "%.1f, %.1f in", pos.xMm / 25.4, pos.yMm / 25.4);
            telemetry.addData("Heading", "%.1f°", Math.toDegrees(pos.headingRad));
            telemetry.addLine("");
        }

        telemetry.addLine("Configure at:");
        telemetry.addLine("panels.bylazar.com");
        telemetry.addLine("═══════════════════════════════");

        telemetry.update();
    }

    @Override
    public void stop() {
        robot.stop();
        drive.stop();
        if (follower != null) {
            follower.breakFollowing();
        }

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
        telemetry.addLine("PIDF:");
        telemetry.addData("kP", "%.3f", kP);
        telemetry.addData("kI", "%.4f", kI);
        telemetry.addData("kD", "%.3f", kD);
        telemetry.addData("kF", "%.5f", kF);
        telemetry.addLine("");
        telemetry.addLine("BOOST LIMITS:");
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