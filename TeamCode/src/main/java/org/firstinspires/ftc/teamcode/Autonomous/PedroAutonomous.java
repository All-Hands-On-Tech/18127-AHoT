package org.firstinspires.ftc.teamcode.Autonomous;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Autonomous Path Selection", group = "Autonomous")
@Configurable // Panels
public class PedroAutonomous extends OpMode {

    private TelemetryManager panelsTelemetry; // Panels Telemetry instance
    public Follower follower; // Pedro Pathing follower instance
    private int pathState; // Current autonomous path state (state machine)
    private Paths paths; // Paths defined in the Paths class

    // Hardware for intake and deposit
    private DcMotor intakeMotor;
    private DcMotorEx depositMotorL;
    private DcMotorEx depositMotorR;
    private Servo cam;

    // ========== TUNING VARIABLES ==========
    // Intake Configuration
    private static final double INTAKE_POWER = -1; // Negative to run in correct direction
    private static final double INTAKE_DURATION_S = 6.5;

    // Cam Configuration
    private static final double CAM_POSITION = 0.5124;

    // Deposit Configuration - using motor's built-in velocity control
    private double depositTargetVelocity = 662.7; // ticks per second (adjustable via dpad)

    // State Machine Timing Configuration
    private static final double WAIT_BEFORE_INTAKE_S = 3; // Wait 2 seconds before starting intake

    // Tuning parameters for dpad adjustment
    private static final double MIN_VELOCITY = 0.0;
    private static final double MAX_VELOCITY = 5000.0;
    private static final double STEP_SMALL = 10.0;
    private static final double STEP_LARGE = 100.0;

    // For D-pad edge detection and hold-repeat
    private boolean prevDpadUp = false, prevDpadDown = false, prevDpadLeft = false, prevDpadRight = false;
    private long lastDpadChange = System.currentTimeMillis();
    private static final long FIRST_REPEAT_DELAY_MS = 350;
    private static final long REPEAT_INTERVAL_MS = 120;

    // Path Selection Enum
    public enum PathSelection {
        BLUE_BOTTOM,
        BLUE_TOP,
        RED_BOTTOM,
        RED_TOP
    }

    // ========== STATE MACHINE CONSTANTS ==========
    private static final int STATE_START_PATH1 = 0;
    private static final int STATE_PATH1_AND_DEPOSIT = 1;
    private static final int STATE_WAIT_BEFORE_INTAKE = 2;
    private static final int STATE_INTAKE = 3;
    private static final int STATE_START_PATH2 = 4;
    private static final int STATE_PATH2_MOVING = 5;
    private static final int STATE_DONE = -1;

    // ========== INSTANCE VARIABLES ==========
    private ElapsedTime actionTimer;
    private PathSelection selectedPath = PathSelection.BLUE_BOTTOM; // Default selection
    private boolean pathSelected = false;

    // Deposit manual/auto control
    private boolean depositEnabled = false; // when true, motors try to reach target ticks/sec
    private boolean lastAState = false;     // for edge-detecting the A button

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        paths = new Paths(follower); // Build paths

        // Initialize action timer
        actionTimer = new ElapsedTime();

        // Initialize intake motor
        try {
            intakeMotor = hardwareMap.get(DcMotor.class, "intake");
            if (intakeMotor != null) {
                intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                intakeMotor.setPower(0);
            }
        } catch (Exception e) {
            intakeMotor = null;
        }

        // Initialize deposit motors for velocity control (matching DepositTuner)
        try {
            depositMotorL = hardwareMap.get(DcMotorEx.class, "DepositMotorL");
            if (depositMotorL != null) {
                depositMotorL.setDirection(DcMotorSimple.Direction.REVERSE); // Reverse left motor for auto
                depositMotorL.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
                depositMotorL.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
                depositMotorL.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
                depositMotorL.setPower(0);
            }
        } catch (Exception e) {
            depositMotorL = null;
        }

        try {
            depositMotorR = hardwareMap.get(DcMotorEx.class, "DepositMotorR");
            if (depositMotorR != null) {
                depositMotorR.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
                depositMotorR.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
                depositMotorR.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
                depositMotorR.setPower(0);
            }
        } catch (Exception e) {
            depositMotorR = null;
        }

        // Initialize cam servo and set position
        try {
            cam = hardwareMap.get(Servo.class, "Cam");
            if (cam != null) {
                cam.setPosition(CAM_POSITION);
            }
        } catch (Exception e) {
            cam = null;
        }

        // Show path-button mapping permanently (also echoed every loop)
        panelsTelemetry.debug("Status", "Initialized - Select a path");
        panelsTelemetry.debug("Paths (permanent):", "DPad Up=Blue Bottom, Right=Blue Top, Down=Red Bottom, Left=Red Top");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void init_loop() {
        if (!pathSelected) {
            // Handle path selection via D-Pad
            if (gamepad1.dpad_up) {
                selectedPath = PathSelection.BLUE_BOTTOM;
                pathSelected = true;
            } else if (gamepad1.dpad_right) {
                selectedPath = PathSelection.BLUE_TOP;
                pathSelected = true;
            } else if (gamepad1.dpad_down) {
                selectedPath = PathSelection.RED_BOTTOM;
                pathSelected = true;
            } else if (gamepad1.dpad_left) {
                selectedPath = PathSelection.RED_TOP;
                pathSelected = true;
            }

            if (pathSelected) {
                // Set starting pose based on selected path
                Pose startPose = paths.getStartPose(selectedPath);
                follower.setStartingPose(startPose);
                panelsTelemetry.debug("Status", "Path Selected: " + selectedPath);
                panelsTelemetry.debug("Start Pose", startPose);
            } else {
                panelsTelemetry.debug("Status", "Waiting for path selection...");
            }
        }
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void loop() {
        follower.update(); // Update Pedro Pathing
        pathState = autonomousPathUpdate(); // Update autonomous state machine

        // ===== DPAD TUNING FOR DEPOSIT SPEED =====
        boolean dpadUp = gamepad1.dpad_up;
        boolean dpadDown = gamepad1.dpad_down;
        boolean dpadLeft = gamepad1.dpad_left;
        boolean dpadRight = gamepad1.dpad_right;

        long now = System.currentTimeMillis();

        // Dpad Up = Increase by large step
        if (dpadUp && (!prevDpadUp || now - lastDpadChange > FIRST_REPEAT_DELAY_MS)) {
            depositTargetVelocity = Math.min(MAX_VELOCITY, depositTargetVelocity + STEP_LARGE);
            lastDpadChange = now;
        }
        // Dpad Down = Decrease by large step
        if (dpadDown && (!prevDpadDown || now - lastDpadChange > FIRST_REPEAT_DELAY_MS)) {
            depositTargetVelocity = Math.max(MIN_VELOCITY, depositTargetVelocity - STEP_LARGE);
            lastDpadChange = now;
        }
        // Dpad Right = Increase by small step
        if (dpadRight && (!prevDpadRight || now - lastDpadChange > FIRST_REPEAT_DELAY_MS)) {
            depositTargetVelocity = Math.min(MAX_VELOCITY, depositTargetVelocity + STEP_SMALL);
            lastDpadChange = now;
        }
        // Dpad Left = Decrease by small step
        if (dpadLeft && (!prevDpadLeft || now - lastDpadChange > FIRST_REPEAT_DELAY_MS)) {
            depositTargetVelocity = Math.max(MIN_VELOCITY, depositTargetVelocity - STEP_SMALL);
            lastDpadChange = now;
        }

        // Enable faster repeat when holding dpad
        if ((dpadUp || dpadDown || dpadLeft || dpadRight) && now - lastDpadChange > REPEAT_INTERVAL_MS) {
            lastDpadChange = now - (REPEAT_INTERVAL_MS + 1);
        }

        // Update previous dpad states
        prevDpadUp = dpadUp;
        prevDpadDown = dpadDown;
        prevDpadLeft = dpadLeft;
        prevDpadRight = dpadRight;

        // Manual toggle for deposit control: single button (A) toggles on/off
        boolean aPressed = gamepad1.a;
        if (aPressed && !lastAState) {
            depositEnabled = !depositEnabled;
        }
        lastAState = aPressed;

        // If deposit is enabled (either manually toggled or set by autonomous), use motor's built-in velocity control
        if (depositEnabled) {
            runDepositAtVelocity();
        } else {
            stopDeposit();
        }

        // Log values to Panels and Driver Station
        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", follower.getPose().getHeading());

        // Deposit tuning telemetry
        panelsTelemetry.debug("Deposit Target", String.format(java.util.Locale.US, "%.0f ticks/s", depositTargetVelocity));
        panelsTelemetry.debug("Deposit Enabled", depositEnabled ? "YES (Press A to toggle)" : "NO (Press A to toggle)");

        if (depositMotorL != null) {
            double vL = depositMotorL.getVelocity();
            double rpmL = (vL / 28.0) * 60.0;
            panelsTelemetry.debug("DepositL", String.format(java.util.Locale.US, "%.0f ticks/s (%.0f RPM)", vL, rpmL));
        }
        if (depositMotorR != null) {
            double vR = depositMotorR.getVelocity();
            double rpmR = (vR / 28.0) * 60.0;
            panelsTelemetry.debug("DepositR", String.format(java.util.Locale.US, "%.0f ticks/s (%.0f RPM)", vR, rpmR));
        }

        if (intakeMotor != null) panelsTelemetry.debug("Intake Power", intakeMotor.getPower());
        panelsTelemetry.debug("ActionTimer", String.format(java.util.Locale.US, "%.1f s", actionTimer.seconds()));

        panelsTelemetry.debug("Controls", "DPAD: Up/Down=±100 Left/Right=±10 | A=Toggle Deposit");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void stop() {
        // Stop all motors
        if (intakeMotor != null) intakeMotor.setPower(0);
        if (depositMotorL != null) depositMotorL.setPower(0);
        if (depositMotorR != null) depositMotorR.setPower(0);
    }

    private void runDepositAtVelocity() {
        if (depositMotorL == null || depositMotorR == null) return;

        // Use motor's built-in velocity PID control - both motors get same target
        depositMotorL.setVelocity(depositTargetVelocity);
        depositMotorR.setVelocity(depositTargetVelocity);
    }

    private void stopDeposit() {
        if (depositMotorL == null || depositMotorR == null) return;
        depositMotorL.setPower(0);
        depositMotorR.setPower(0);
    }

    private void startIntake() {
        if (intakeMotor != null) intakeMotor.setPower(INTAKE_POWER);
    }

    private void stopIntake() {
        if (intakeMotor != null) intakeMotor.setPower(0);
    }

    public static class Paths {

        // Split paths into segment 1 (to first point) and segment 2 (to second point)
        public PathChain blueBottomPath1;
        public PathChain blueBottomPath2;
        public PathChain blueTopPath1;
        public PathChain blueTopPath2;
        public PathChain redBottomPath1;
        public PathChain redBottomPath2;
        public PathChain redTopPath1;
        public PathChain redTopPath2;

        public Paths(Follower follower) {
            // Blue Bottom Path - Segment 1: Start to Point 1
            blueBottomPath1 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(55.083, 6.632), new Pose(58.180, 90.105))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(136))
                    .build();

            // Blue Bottom Path - Segment 2: Point 1 to Point 2
            blueBottomPath2 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(66.180, 88.105), new Pose(49.263, 52.647))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(0))
                    .build();

            // Blue Top Path - Segment 1: Start to Point 1
            blueTopPath1 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(23.549, 128.842), new Pose(58.075, 90.128))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(323), Math.toRadians(136))
                    .build();

            // Blue Top Path - Segment 2: Point 1 to Point 2
            blueTopPath2 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(68.075, 85.128), new Pose(56.707, 68.752))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(138), Math.toRadians(90))
                    .build();

            // Red Bottom Path - Segment 1: Start to Point 1
            redBottomPath1 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(88.917, 6.767), new Pose(82.744, 91.218))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(45))
                    .build();

            // Red Bottom Path - Segment 2: Point 1 to Point 2
            redBottomPath2 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(81.744, 91.218), new Pose(97.850, 68.617))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(90))
                    .build();

            // Red Top Path - Segment 1: Start to Point 1
            redTopPath1 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(120.722, 128.977), new Pose(83.744, 92.218))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(217), Math.toRadians(46))
                    .build();

            // Red Top Path - Segment 2: Point 1 to Point 2
            redTopPath2 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(81.744, 91.218), new Pose(93.789, 57.248))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(46), Math.toRadians(90))
                    .build();
        }

        public PathChain getPath1(PathSelection pathSelection) {
            return switch (pathSelection) {
                case BLUE_BOTTOM -> blueBottomPath1;
                case BLUE_TOP -> blueTopPath1;
                case RED_BOTTOM -> redBottomPath1;
                case RED_TOP -> redTopPath1;
            };
        }

        public PathChain getPath2(PathSelection pathSelection) {
            return switch (pathSelection) {
                case BLUE_BOTTOM -> blueBottomPath2;
                case BLUE_TOP -> blueTopPath2;
                case RED_BOTTOM -> redBottomPath2;
                case RED_TOP -> redTopPath2;
            };
        }

        public Pose getStartPose(PathSelection pathSelection) {
            return switch (pathSelection) {
                case BLUE_BOTTOM -> new Pose(55.083, 6.632, Math.toRadians(90));
                case BLUE_TOP -> new Pose(23.549, 128.842, Math.toRadians(323));
                case RED_BOTTOM -> new Pose(88.917, 6.767, Math.toRadians(90));
                case RED_TOP -> new Pose(120.722, 128.977, Math.toRadians(217));
            };
        }
    }

    public int autonomousPathUpdate() {
        switch (pathState) {
            case STATE_START_PATH1:
                // Start following path to point 1
                follower.followPath(paths.getPath1(selectedPath));
                // enable deposit during path
                depositEnabled = true;
                pathState = STATE_PATH1_AND_DEPOSIT;
                break;

            case STATE_PATH1_AND_DEPOSIT:
                // Deposit is enabled (will run from loop if depositEnabled==true)
                // Wait until follower finishes path to point 1
                if (!follower.isBusy()) {
                    // Reached point 1, start wait timer
                    actionTimer.reset();
                    pathState = STATE_WAIT_BEFORE_INTAKE;
                }
                break;

            case STATE_WAIT_BEFORE_INTAKE:
                // deposit remains enabled while waiting at point 1
                // Wait before starting intake
                if (actionTimer.seconds() >= WAIT_BEFORE_INTAKE_S) {
                    actionTimer.reset();
                    startIntake();
                    pathState = STATE_INTAKE;
                }
                break;

            case STATE_INTAKE:
                // Run intake and keep deposit enabled at point 1
                if (actionTimer.seconds() >= INTAKE_DURATION_S) {
                    stopIntake();
                    // Start path to point 2
                    pathState = STATE_START_PATH2;
                }
                break;

            case STATE_START_PATH2:
                // Start following path to point 2
                follower.followPath(paths.getPath2(selectedPath));
                pathState = STATE_PATH2_MOVING;
                break;

            case STATE_PATH2_MOVING:
                // Moving to point 2, deposit still enabled
                if (!follower.isBusy()) {
                    // Reached point 2, stop everything
                    stopDeposit();
                    depositEnabled = false;
                    pathState = STATE_DONE;
                }
                break;

            case STATE_DONE:
                // All motors stopped
                break;

            default:
                // Idle
                break;
        }

        return pathState;
    }
}
