package org.firstinspires.ftc.teamcode.Trowel.Autonomous;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.Trowel.Configs.TrowelHardware;
import org.firstinspires.ftc.teamcode.Trowel.pedroPathing.Constants;

@Autonomous(name = "Trowel Auto", group = "Autonomous")
@Configurable
public class StraightAuto extends OpMode {

    public enum Team { NONE, RED, BLUE }
    private Team selectedTeam = Team.NONE;

    private TelemetryManager panelsTelemetry;
    public Follower follower;
    private int pathState = 0;
    private Paths paths;
    private ElapsedTime timer;
    private TrowelHardware robot;

    // Pedro field is 144x144; used for alliance mirroring
    private static final double FIELD_SIZE = 144.0;

    // ============== TUNED VALUES (placeholders) ==============
    // Paste numbers from AutomaticDepoTuner here. Keep names short and clear.
    public static double DEPO_SERVO_POSITION_SHOOTING = 0.0; // paste SERVO_POSITION_SHOOTING
    public static double DEPO_SERVO_POSITION_IDLE = 0.0;     // paste SERVO_POSITION_IDLE

    // sensible defaults so auto can run without pasted values
    public static double DEPO_DEPOSIT_TARGET_VELOCITY = 840.0; // default deposit target velocity (ticks/sec)
    public static double DEPO_DEPOSIT_TOLERANCE = 0.0;       // paste DEPOSIT_TOLERANCE
    public static double DEPO_SPEED_INCREMENT_SMALL = 0.0;   // paste SPEED_INCREMENT_SMALL
    public static double DEPO_SPEED_INCREMENT_LARGE = 0.0;   // paste SPEED_INCREMENT_LARGE
    public static double DEPO_MIN_DEPOSIT_SPEED = 0.0;       // paste MIN_DEPOSIT_SPEED
    public static double DEPO_MAX_DEPOSIT_SPEED = 0.0;       // paste MAX_DEPOSIT_SPEED

    public static double DEPO_DRIVE_POWER_SCALE = 0.0;      // paste DRIVE_POWER_SCALE
    public static double DEPO_STRAFE_POWER_SCALE = 0.0;     // paste STRAFE_POWER_SCALE
    public static double DEPO_ROTATE_POWER_SCALE = 0.0;     // paste ROTATE_POWER_SCALE
    public static double DEPO_INTAKE2_SCALE = 0.0;          // paste INTAKE2_SCALE

    public static double DEPO_BOOST_TRIGGER_THRESHOLD = 0.0; // paste boostTriggerThreshold
    public static double DEPO_BOOST_MIN_TICKS = 0.0;         // paste boostMinTicks
    public static double DEPO_BOOST_MAX_TICKS = 0.0;         // paste boostMaxTicks

    public static double DEPO_BALL1_MULTIPLIER = 0.0; // paste ball1Multiplier
    public static double DEPO_BALL1_EXPONENT = 0.0;   // paste ball1Exponent
    public static double DEPO_BALL2_MULTIPLIER = 0.0; // paste ball2Multiplier
    public static double DEPO_BALL2_EXPONENT = 0.0;   // paste ball2Exponent
    public static double DEPO_BALL3_MULTIPLIER = 0.0; // paste ball3Multiplier
    public static double DEPO_BALL3_EXPONENT = 0.0;   // paste ball3Exponent

    public static double DEPO_KP = 10.0; // default kP
    public static double DEPO_KI = 0.0;  // default kI
    public static double DEPO_KD = 0.0;  // default kD
    public static double DEPO_KF = 0.0;  // default kF (keep manual)

    // Additional ML / tuning params are intentionally available here as placeholders
    public static boolean DEPO_ML_ENABLED = false;
    public static double DEPO_LEARNING_RATE = 0.0;

    // ============== AUTO shooting defaults (will use tuned values when pasted) ==============
    // These default values were used previously; paste tuned values above to override
    public static double AUTO_DEPOSIT_FF_FACTOR = 0.05; // multiplicative FF (kept as default)
    public static double AUTO_DEPOSIT_FF_BOOST_TICKS = 241.0; // absolute FF boost ticks
    public static long AUTO_SHOOT_DURATION_MS = 1800;    // tuneable shoot duration (was 2000)
    public static long AUTO_SHOOT_RECOVERY_MS = 100;     // optional recovery idle after shot (was 200)
    public static long PRE_SHOOT_DELAY_MS = 100;          // wait before shooting to stabilize (was 130)
    private static final double AUTO_INTAKE1_POWER = 1.0;
    private static final double AUTO_INTAKE2_POWER = -1.0;

    // ============== RUN-TIME STATE ==============
    private long shootHoldEndMs = 0;
    private long preShootEndMs = 0;
    private boolean shooting = false;
    private boolean shootingStarted = false;

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        follower = Constants.createFollower(hardwareMap);

        // Telemetry prompt for team selection; paths and pose set in start()
        panelsTelemetry.debug("Status", "Initialized - Select Team (X=BLUE, A=RED)");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void start() {
        // Initialize auxiliary hardware (intakes, deposit, transfer servo)
        robot = new TrowelHardware(hardwareMap);
        robot.resetDepositEncoders();
        robot.initTransferServo();

        // Ensure servo starts at tuned idle position if provided, otherwise keep neutral
        if (robot.transferServo != null) {
            if (DEPO_SERVO_POSITION_IDLE != 0.0) robot.transferServo.setPosition(DEPO_SERVO_POSITION_IDLE);
            else robot.transferServo.setPosition(0.5);
        }

        // Apply tuned PIDF if user has pasted values
        updateDepositPIDFCoefficients();

        // Apply feedforward tuning values (these help spin up the deposit faster)
        robot.setDepositFeedforwardFactor(AUTO_DEPOSIT_FF_FACTOR);
        robot.setDepositFeedforwardBoostTicks(AUTO_DEPOSIT_FF_BOOST_TICKS);

        // Keep deposit running for the entire auto at tuned velocity
        double targetVel = (DEPO_DEPOSIT_TARGET_VELOCITY != 0.0) ? DEPO_DEPOSIT_TARGET_VELOCITY : 160.0;
        robot.setDepositVelocity(targetVel);

        if (robot.intake1 != null) robot.intake1.setPower(0.0);
        if (robot.intake2 != null) robot.intake2.setPower(0.0);
        // Leave transfer neutral until shooting
        setTransferNeutral();
        panelsTelemetry.debug("Aux Init", robot.getInitializationStatus());

        // Apply starting pose based on selected team
        Pose startPose;
        if (selectedTeam == Team.BLUE) {
            startPose = new Pose(26.467, 129.584, Math.toRadians(145));
            panelsTelemetry.debug("Team Selected", "BLUE");
            panelsTelemetry.debug("StartPose", "BLUE (26.467, 129.584, 145°)");
        } else {
            startPose = new Pose(117.533, 129.584, Math.toRadians(35));
            panelsTelemetry.debug("Team Selected", "RED");
            panelsTelemetry.debug("StartPose", "RED (117.533, 129.584, 35°)");
        }
        follower.setStartingPose(startPose);
        // Ensure follower internal pose is initialized to match the starting pose
        try { follower.update(); } catch (Exception ignored) {}
        // Use full power for both alliances to keep behavior consistent
        try { follower.setMaxPower(1.0); } catch (Exception ignored) {}

        // Build paths for selected team (red paths mirrored for blue)
        paths = new Paths(follower, selectedTeam);
        pathState = 0;
        timer = new ElapsedTime();

        // Initialize drive motors and add telemetry for their power multipliers
        Constants.setMotorPowerMultiplier("LEFT_FRONT", Constants.LEFT_FRONT_POWER);
        Constants.setMotorPowerMultiplier("LEFT_REAR", Constants.LEFT_REAR_POWER);
        Constants.setMotorPowerMultiplier("RIGHT_FRONT", Constants.RIGHT_FRONT_POWER);
        Constants.setMotorPowerMultiplier("RIGHT_REAR", Constants.RIGHT_REAR_POWER);

        panelsTelemetry.debug("Motor Power Multipliers", "LF: " + Constants.LEFT_FRONT_POWER + ", LR: " + Constants.LEFT_REAR_POWER + ", RF: " + Constants.RIGHT_FRONT_POWER + ", RR: " + Constants.RIGHT_REAR_POWER);
        panelsTelemetry.debug("StartPose X", follower.getPose().getX());
        panelsTelemetry.debug("StartPose Y", follower.getPose().getY());
        panelsTelemetry.debug("StartPose Heading", follower.getPose().getHeading());
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void init_loop() {
        if (gamepad1.x) {
            selectedTeam = Team.BLUE;
        } else if (gamepad1.a) {
            selectedTeam = Team.RED;
        }

        telemetry.addLine("=== TEAM SELECTION ===");
        switch (selectedTeam) {
            case BLUE:
                telemetry.addData("Selected Team", "BLUE");
                break;
            case RED:
                telemetry.addData("Selected Team", "RED");
                break;
            default:
                telemetry.addData("Selected Team", "NONE - Please select!");
                telemetry.addLine("Press X for BLUE team");
                telemetry.addLine("Press A for RED team");
                break;
        }
        telemetry.addLine("");
        telemetry.addLine("Press START when ready");
        telemetry.update();
    }

    @Override
    public void loop() {
        follower.update();
        pathState = autonomousPathUpdate();

        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", follower.getPose().getHeading());
        if (shooting) {
            panelsTelemetry.debug("Shooting", "ACTIVE (ends in " + (shootHoldEndMs - System.currentTimeMillis()) + "ms)");
        }
        panelsTelemetry.update(telemetry);
    }

    private void startShooting() {
        shooting = true;
        shootingStarted = false;
        long now = System.currentTimeMillis();
        preShootEndMs = now + PRE_SHOOT_DELAY_MS;
        shootHoldEndMs = preShootEndMs + AUTO_SHOOT_DURATION_MS;

        // Ensure deposit is at target velocity (already spinning in start())
        double targetVel = (DEPO_DEPOSIT_TARGET_VELOCITY != 0.0) ? DEPO_DEPOSIT_TARGET_VELOCITY : 160.0;
        robot.setDepositVelocity(targetVel);

        panelsTelemetry.debug("Shoot", "Waiting " + PRE_SHOOT_DELAY_MS + "ms then holding for " + AUTO_SHOOT_DURATION_MS + "ms");

        // Servo should ALREADY be open from traveling to Depo position
        // Just ensure it's in shooting position
        setTransferShooting();
    }

    private boolean updateShooting() {
        if (!shooting) return false;
        long now = System.currentTimeMillis();

        // Wait for pre-shoot settle before starting intakes
        if (!shootingStarted && now >= preShootEndMs) {
            if (robot.intake1 != null) robot.intake1.setPower(AUTO_INTAKE1_POWER);
            if (robot.intake2 != null) robot.intake2.setPower(AUTO_INTAKE2_POWER);
            shootingStarted = true;
        }

        // Keep deposit spinning during entire auto
        if (shootingStarted && now >= shootHoldEndMs) {
            shooting = false;
            shootingStarted = false;

            // Stop intakes after shooting
            if (robot.intake1 != null) robot.intake1.setPower(0.0);
            if (robot.intake2 != null) robot.intake2.setPower(0.0);

            // Close servo after shooting (go to intake position)
            setTransferIdle();

            panelsTelemetry.debug("Shoot", "Completed");

            // Optional recovery pause before next sequence
            if (AUTO_SHOOT_RECOVERY_MS > 0) {
                try { Thread.sleep(AUTO_SHOOT_RECOVERY_MS); } catch (InterruptedException ignored) {}
            }
            return true;
        }
        return false;
    }

    public class Paths {
        public PathChain Depo1;
        public PathChain Gate1;
        public PathChain Rotate;
        public PathChain IntakeStart1;
        public PathChain IntakeEnd1;
        public PathChain Depo2;
        public PathChain IntakeStart2;
        public PathChain IntakeEnd2;
        public PathChain Depo3;
        public PathChain IntakeStart3;
        public PathChain IntakeEnd3;
        public PathChain Depo4;
        public PathChain Gate2;

        public Paths(Follower follower, Team team) {
            if (team == Team.BLUE) {
                buildBluePaths(follower);
            } else {
                buildRedPaths(follower);
            }
        }

        private void buildRedPaths(Follower follower) {
            Depo1 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(117.533, 129.584),
                                    new Pose(86.121, 88.138)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(35), Math.toRadians(45))
                    .build();

            Gate1 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(86.121, 88.138),
                                    new Pose(76.703, 73.739),
                                    new Pose(127.033, 72.148)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(90))
                    .build();

            Rotate = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(127.033, 72.148),
                                    new Pose(83.813, 69.774),
                                    new Pose(97.564, 85.935)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(90))
                    .build();

            IntakeStart1 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(97.564, 85.935),
                                    new Pose(102.644, 86.010)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(180))
                    .build();

            IntakeEnd1 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(102.644, 86.010),
                                    new Pose(126.609, 86.823)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            Depo2 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(126.609, 86.823),
                                    new Pose(86.190, 88.414)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(45))
                    .build();

            IntakeStart2 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(86.190, 88.414),
                                    new Pose(84.724, 83.552),
                                    new Pose(101.241, 59.207)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(180))
                    .build();

            IntakeEnd2 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(101.241, 59.207),
                                    new Pose(134.247, 59.092)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            Depo3 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(134.247, 59.092),
                                    new Pose(90.265, 65.451),
                                    new Pose(86.017, 88.466)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(45))
                    .build();

            IntakeStart3 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(86.017, 88.466),
                                    new Pose(102.155, 35.954)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(180))
                    .build();

            IntakeEnd3 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(102.155, 35.954),
                                    new Pose(134.047, 35.442)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            Depo4 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(134.047, 35.442),
                                    new Pose(86.138, 88.517)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(45))
                    .build();

            Gate2 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(86.138, 88.517),
                                    new Pose(117.860, 72.035)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(90))
                    .build();

            // No automatic mirroring here; blue/red paths are built explicitly in their respective builders
         }

        private void buildBluePaths(Follower follower) {
            Depo1 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(26.467, 129.584),
                                    new Pose(57.879, 88.138)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(145), Math.toRadians(135))
                    .build();

            Gate1 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(57.879, 88.138),
                                    new Pose(67.297, 73.739),
                                    new Pose(16.967, 72.148)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(90))
                    .build();

            Rotate = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(16.967, 72.148),
                                    new Pose(60.187, 69.774),
                                    new Pose(46.436, 85.935)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(90))
                    .build();

            IntakeStart1 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(46.436, 85.935),
                                    new Pose(41.356, 86.010)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(0))
                    .build();

            IntakeEnd1 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(41.356, 86.010),
                                    new Pose(17.391, 86.823)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            Depo2 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(17.391, 86.823),
                                    new Pose(57.810, 88.414)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(135))
                    .build();

            IntakeStart2 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(57.810, 88.414),
                                    new Pose(59.276, 83.552),
                                    new Pose(42.759, 59.207)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(0))
                    .build();

            IntakeEnd2 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(42.759, 59.207),
                                    new Pose(9.753, 59.092)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            Depo3 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(9.753, 59.092),
                                    new Pose(53.735, 65.451),
                                    new Pose(57.983, 88.466)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(135))
                    .build();

            IntakeStart3 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(57.983, 88.466),
                                    new Pose(41.845, 35.954)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(0))
                    .build();

            IntakeEnd3 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(41.845, 35.954),
                                    new Pose(9.953, 35.442)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            Depo4 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(9.953, 35.442),
                                    new Pose(57.862, 88.517)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(135))
                    .build();

            Gate2 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(57.862, 88.517),
                                    new Pose(26.140, 72.035)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(90))
                    .build();
        }
    }

    // Update the state machine to follow each path in sequence
    public int autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                // Traveling toward Depo1 - OPEN SERVO (servo needs to be out of the way by shooting time)
                setTransferShooting();
                follower.followPath(paths.Depo1);
                pathState = 1;
                panelsTelemetry.debug("Transition", "Started Depo1 - Servo OPEN");
                break;
            case 1:
                if (!follower.isBusy()) {
                    startShooting(); // Shoot at Depo1 (servo already open)
                    pathState = 14;
                    panelsTelemetry.debug("Transition", "Shooting after Depo1");
                }
                break;
            case 14: // Wait shooting after Depo1
                if (updateShooting()) {
                    // After shooting, servo closes automatically in updateShooting()
                    // Moving to Gate1 - only deposit running, servo closed
                    follower.followPath(paths.Gate1);
                    pathState = 15;
                    panelsTelemetry.debug("Transition", "Started Gate1 - Only deposit running");
                }
                break;
            case 15:
                if (!follower.isBusy()) {
                    // Pause briefly at the gate before continuing
                    timer.reset();
                    pathState = 150; // gate wait state
                    panelsTelemetry.debug("Transition", "Reached Gate1 - waiting 0.7s");
                }
                break;
            case 150: // gate wait (0.4s) then continue to Rotate
                if (timer.seconds() >= 0.4) {
                    follower.followPath(paths.Rotate);
                    pathState = 16;
                    panelsTelemetry.debug("Transition", "Gate1 wait complete - Started Rotate");
                }
                break;
            case 16:
                if (!follower.isBusy()) {
                    // Starting intake sequence - servo MUST be closed, intakes ON
                    setTransferIdle(); // Ensure servo is closed
                    startIntakeSequence();
                    follower.followPath(paths.IntakeStart1);
                    pathState = 2;
                    panelsTelemetry.debug("Transition", "Started IntakeStart1 - Intakes ON, Servo CLOSED");
                }
                break;
            case 2:
                if (!follower.isBusy()) {
                    // Continue intaking - servo stays closed
                    follower.followPath(paths.IntakeEnd1);
                    pathState = 3;
                    panelsTelemetry.debug("Transition", "Started IntakeEnd1 - Intakes ON, Servo CLOSED");
                }
                break;
            case 3:
                if (!follower.isBusy()) {
                    stopIntakes(); // Stop intakes
                    // Heading to Depo2 - OPEN SERVO
                    setTransferShooting();
                    follower.followPath(paths.Depo2);
                    pathState = 4;
                    panelsTelemetry.debug("Transition", "Started Depo2 - Servo OPEN");
                }
                break;
            case 4:
                if (!follower.isBusy()) {
                    startShooting(); // Shoot at Depo2
                    pathState = 17;
                    panelsTelemetry.debug("Transition", "Shooting after Depo2");
                }
                break;
            case 17:
                if (updateShooting()) {
                    // Servo closes automatically, start intaking
                    setTransferIdle();
                    startIntakeSequence();
                    follower.followPath(paths.IntakeStart2);
                    pathState = 6;
                    panelsTelemetry.debug("Transition", "Started IntakeStart2 - Intakes ON, Servo CLOSED");
                }
                break;
            case 6:
                if (!follower.isBusy()) {
                    follower.followPath(paths.IntakeEnd2);
                    pathState = 7;
                    panelsTelemetry.debug("Transition", "Started IntakeEnd2 - Intakes ON, Servo CLOSED");
                }
                break;
            case 7:
                if (!follower.isBusy()) {
                    stopIntakes();
                    // Heading to Depo3 - OPEN SERVO
                    setTransferShooting();
                    follower.followPath(paths.Depo3);
                    pathState = 8;
                    panelsTelemetry.debug("Transition", "Started Depo3 - Servo OPEN");
                }
                break;
            case 8:
                if (!follower.isBusy()) {
                    startShooting(); // Shoot at Depo3
                    pathState = 18;
                    panelsTelemetry.debug("Transition", "Shooting after Depo3");
                }
                break;
            case 18:
                if (updateShooting()) {
                    // Servo closes automatically, start intaking
                    setTransferIdle();
                    startIntakeSequence();
                    follower.followPath(paths.IntakeStart3);
                    pathState = 9;
                    panelsTelemetry.debug("Transition", "Started IntakeStart3 - Intakes ON, Servo CLOSED");
                }
                break;
            case 9:
                if (!follower.isBusy()) {
                    follower.followPath(paths.IntakeEnd3);
                    pathState = 10;
                    panelsTelemetry.debug("Transition", "Started IntakeEnd3 - Intakes ON, Servo CLOSED");
                }
                break;
            case 10:
                if (!follower.isBusy()) {
                    stopIntakes();
                    // Heading to Depo4 - OPEN SERVO
                    setTransferShooting();
                    follower.followPath(paths.Depo4);
                    pathState = 11;
                    panelsTelemetry.debug("Transition", "Started Depo4 - Servo OPEN");
                }
                break;
            case 11:
                if (!follower.isBusy()) {
                    startShooting(); // Shoot at Depo4
                    pathState = 19;
                    panelsTelemetry.debug("Transition", "Shooting after Depo4");
                }
                break;
            case 19:
                if (updateShooting()) {
                    // Servo closes automatically
                    // Moving to Gate2 - only deposit running
                    follower.followPath(paths.Gate2);
                    pathState = 12;
                    panelsTelemetry.debug("Transition", "Started Gate2 - Only deposit running");
                }
                break;
            case 12:
                if (!follower.isBusy()) {
                    // Pause at Gate2 for a short duration before finishing
                    timer.reset();
                    pathState = 120; // gate2 wait
                    panelsTelemetry.debug("Transition", "Reached Gate2 - waiting 0.7s before finish");
                }
                break;
            case 120:
                if (timer.seconds() >= 0.4) {
                    pathState = 13; // finished
                    panelsTelemetry.debug("Transition", "Gate2 wait complete - Auto Complete");
                }
                break;
            case 13:
                // finished - keep deposit running, all else idle
                break;
        }
        return pathState;
    }

    // Transfer servo helpers use tuned positions when available
    private void setTransferOut() {
        if (robot.transferServo != null) {
            if (DEPO_SERVO_POSITION_SHOOTING != 0.0) robot.transferServo.setPosition(DEPO_SERVO_POSITION_SHOOTING);
            else robot.transferServo.setPosition(1.0);
        }
    }

    private void setTransferNeutral() {
        if (robot.transferServo != null) {
            if (DEPO_SERVO_POSITION_IDLE != 0.0) robot.transferServo.setPosition(DEPO_SERVO_POSITION_IDLE);
            else robot.transferServo.setPosition(0.5);
        }
    }

    private void setTransferIn() {
        if (robot.transferServo != null) {
            if (DEPO_SERVO_POSITION_IDLE != 0.0) robot.transferServo.setPosition(DEPO_SERVO_POSITION_IDLE);
            else robot.transferServo.setPosition(0.0);
        }
    }

    // Explicit named helpers for clarity (shooting vs idle)
    private void setTransferShooting() {
        if (robot.transferServo != null) {
            if (DEPO_SERVO_POSITION_SHOOTING != 0.0) robot.transferServo.setPosition(DEPO_SERVO_POSITION_SHOOTING);
            else robot.transferServo.setPosition(0.7);
        }
    }

    private void setTransferIdle() {
        if (robot.transferServo != null) {
            if (DEPO_SERVO_POSITION_IDLE != 0.0) robot.transferServo.setPosition(DEPO_SERVO_POSITION_IDLE);
            else robot.transferServo.setPosition(0.3);
        }
    }

    private void startIntakeSequence() {
        // Run intakes at full power
        if (robot.intake1 != null) robot.intake1.setPower(AUTO_INTAKE1_POWER);
        if (robot.intake2 != null) robot.intake2.setPower(AUTO_INTAKE2_POWER);

        // Ensure transfer servo is CLOSED during intake
        setTransferIdle();
    }

    private void stopIntakes() {
        // Stop both intakes
        if (robot.intake1 != null) robot.intake1.setPower(0.0);
        if (robot.intake2 != null) robot.intake2.setPower(0.0);

        // Keep servo in idle/closed position
        setTransferIdle();
    }

    // Update deposit PIDF coefficients on the DcMotorEx directly (mirrors AutomaticDepoTuner)
    private void updateDepositPIDFCoefficients() {
        try {
            if (robot != null && robot.deposit1 != null) robot.deposit1.setVelocityPIDFCoefficients(DEPO_KP, DEPO_KI, DEPO_KD, DEPO_KF);
            if (robot != null && robot.deposit2 != null) robot.deposit2.setVelocityPIDFCoefficients(DEPO_KP, DEPO_KI, DEPO_KD, DEPO_KF);
        } catch (Exception ignored) {}
    }

    /**
     * Calculate boost based on current ball number and parameters (copied from AutomaticDepoTuner)
     */
    private double calculateBoost(double drop, int ballNum) {
        if (drop < DEPO_BOOST_TRIGGER_THRESHOLD) return 0.0;

        double multiplier, exponent;
        if (ballNum == 0) {
            multiplier = DEPO_BALL1_MULTIPLIER;
            exponent = DEPO_BALL1_EXPONENT;
        } else if (ballNum == 1) {
            multiplier = DEPO_BALL2_MULTIPLIER;
            exponent = DEPO_BALL2_EXPONENT;
        } else {
            multiplier = DEPO_BALL3_MULTIPLIER;
            exponent = DEPO_BALL3_EXPONENT;
        }

        double boost = multiplier * Math.pow(drop, exponent);
        return Math.max(DEPO_BOOST_MIN_TICKS, Math.min(DEPO_BOOST_MAX_TICKS, boost));
    }
}

