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
    private static final double INTAKE_DURATION_S = 3.5;

    // Cam Configuration
    private static final double CAM_POSITION = 0.5194;

    // Deposit Configuration - using motor's built-in velocity control
    private double depositTargetVelocity = 677.0; // ticks per second (adjustable via dpad)

    // State Machine Timing Configuration
    private static final double WAIT_BEFORE_INTAKE_S = 2.5; // Wait 2 seconds before starting intake

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
    private static final int STATE_PATH_AND_DEPOSIT = 1;
    private static final int STATE_WAIT_BEFORE_INTAKE = 2;
    private static final int STATE_INTAKE = 3;
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

        // Show initialization complete message
        panelsTelemetry.debug("Status", "✓ Initialized - Use D-Pad to select path");
        panelsTelemetry.debug("Info", "You can change selection anytime before pressing START");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void init_loop() {
        // Handle path selection via D-Pad - always allow changing selection
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

        // Always update the starting pose based on current selection
        if (pathSelected) {
            Pose startPose = paths.getStartPose(selectedPath);
            follower.setStartingPose(startPose);
            panelsTelemetry.debug("Status", "✓ Path Selected (can change before start)");
            panelsTelemetry.debug("Selected Path", selectedPath.toString());
            panelsTelemetry.debug("Start Pose", String.format(java.util.Locale.US, "X:%.1f Y:%.1f H:%.1f°",
                startPose.getX(), startPose.getY(), Math.toDegrees(startPose.getHeading())));
        } else {
            panelsTelemetry.debug("Status", "⚠ Waiting for path selection...");
            panelsTelemetry.debug("Selected Path", "NONE - Use D-Pad to select");
        }

        // Show available paths with visual indicators
        panelsTelemetry.debug("", "─────────────────────────────");
        panelsTelemetry.debug("DPad Up", (selectedPath == PathSelection.BLUE_BOTTOM ? "→ " : "  ") + "Blue Bottom");
        panelsTelemetry.debug("DPad Right", (selectedPath == PathSelection.BLUE_TOP ? "→ " : "  ") + "Blue Top");
        panelsTelemetry.debug("DPad Down", (selectedPath == PathSelection.RED_BOTTOM ? "→ " : "  ") + "Red Bottom");
        panelsTelemetry.debug("DPad Left", (selectedPath == PathSelection.RED_TOP ? "→ " : "  ") + "Red Top");
        panelsTelemetry.debug("", "─────────────────────────────");

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

        public PathChain blueBottomPath;
        public PathChain blueTopPath;
        public PathChain redBottomPath;
        public PathChain redTopPath;

        public Paths(Follower follower) {
            // Blue Bottom Path
            blueBottomPath = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(54.677, 7.038), new Pose(73.218, 88.346))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(138))
                    .build();

            // Blue Top Path
            blueTopPath = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(22.195, 126.812), new Pose(71.323, 79.038))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(325), Math.toRadians(135))
                    .build();

            // Red Bottom Path
            redBottomPath = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(88.917, 6.632), new Pose(79.579, 87.564))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(45))
                    .build();

            // Red Top Path
            redTopPath = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(121.398, 128.030), new Pose(79.579, 82.692))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(218), Math.toRadians(45))
                    .build();
        }

        public PathChain getPath(PathSelection pathSelection) {
            return switch (pathSelection) {
                case BLUE_BOTTOM -> blueBottomPath;
                case BLUE_TOP -> blueTopPath;
                case RED_BOTTOM -> redBottomPath;
                case RED_TOP -> redTopPath;
            };
        }

        public Pose getStartPose(PathSelection pathSelection) {
            return switch (pathSelection) {
                case BLUE_BOTTOM -> new Pose(54.677, 7.038, Math.toRadians(90));
                case BLUE_TOP -> new Pose(22.195, 126.812, Math.toRadians(325));
                case RED_BOTTOM -> new Pose(88.917, 6.632, Math.toRadians(90));
                case RED_TOP -> new Pose(121.398, 128.030, Math.toRadians(218));
            };
        }
    }

    public int autonomousPathUpdate() {
        switch (pathState) {
            case STATE_START_PATH1:
                // Start following the selected path
                follower.followPath(paths.getPath(selectedPath));
                // enable deposit during path
                depositEnabled = true;
                pathState = STATE_PATH_AND_DEPOSIT;
                break;

            case STATE_PATH_AND_DEPOSIT:
                // Deposit is enabled (will run from loop if depositEnabled==true)
                // Wait until follower finishes the path
                if (!follower.isBusy()) {
                    // Path complete, start wait timer
                    actionTimer.reset();
                    pathState = STATE_WAIT_BEFORE_INTAKE;
                }
                break;

            case STATE_WAIT_BEFORE_INTAKE:
                // deposit remains enabled while waiting
                // Wait before starting intake
                if (actionTimer.seconds() >= WAIT_BEFORE_INTAKE_S) {
                    actionTimer.reset();
                    startIntake();
                    pathState = STATE_INTAKE;
                }
                break;

            case STATE_INTAKE:
                // Run intake and keep deposit enabled
                if (actionTimer.seconds() >= INTAKE_DURATION_S) {
                    stopIntake();
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
