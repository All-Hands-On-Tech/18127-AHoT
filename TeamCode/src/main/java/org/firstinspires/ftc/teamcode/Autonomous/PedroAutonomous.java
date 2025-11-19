package org.firstinspires.ftc.teamcode.Autonomous;

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
    private DcMotor intake1;  // Lower stage intake
    private DcMotor intake2;  // Upper stage intake
    private DcMotorEx depositMotorL;
    private DcMotorEx depositMotorR;
    private Servo cam;

    // ========== TUNING VARIABLES ==========
    // Intake Configuration - Two Stage System
    private static final double INTAKE1_POWER = -1.0; // Lower stage - runs continuously
    private static final double INTAKE2_POWER = 1.0; // Upper stage - runs in cycles

    // Deposit Cycle Configuration
    private static final double DEPOSIT_CYCLE_PAUSE_INTAKE1_S = 2.5;  // Wait for motors to stabilize at 656 RPM (first time, longer stabilization)
    private static final double DEPOSIT_CYCLE_INTAKE2_RUN_S = 2.0;    // Run intake2 (extended shooting time)
    private static final double DEPOSIT_CYCLE_INTAKE1_RUN_S = 1.5;    // Run intake1
    private static final double DEPOSIT_CYCLE_PAUSE_S = 1.5;          // Pause
    private static final double DEPOSIT_CYCLE_INTAKE2_RUN2_S = 2.0;   // Run intake2 again (extended shooting time)

    // Cam Configuration
    private static final double CAM_POSITION = 0.5124;

    // Deposit Configuration - using motor's built-in velocity control
    private double depositTargetVelocity = 656.7; // ticks per second (adjustable via dpad)

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


    // ========== STATE MACHINE CONSTANTS ==========
    private static final int STATE_START_PATH1 = 0;
    private static final int STATE_PATH1_MOVING = 1;
    private static final int STATE_DEPOSIT_CYCLE1_PAUSE = 2;
    private static final int STATE_DEPOSIT_CYCLE1_INTAKE2_RUN1 = 3;
    private static final int STATE_DEPOSIT_CYCLE1_INTAKE1_RUN = 4;
    private static final int STATE_DEPOSIT_CYCLE1_PAUSE2 = 5;
    private static final int STATE_DEPOSIT_CYCLE1_INTAKE2_RUN2 = 6;
    private static final int STATE_START_PATH2 = 7;
    private static final int STATE_PATH2_MOVING = 8;
    private static final int STATE_START_PATH3 = 9;
    private static final int STATE_PATH3_MOVING = 10;
    private static final int STATE_START_PATH4 = 11;
    private static final int STATE_PATH4_MOVING = 12;
    private static final int STATE_DEPOSIT_CYCLE2_PAUSE = 13;
    private static final int STATE_DEPOSIT_CYCLE2_INTAKE2_RUN1 = 14;
    private static final int STATE_DEPOSIT_CYCLE2_INTAKE1_RUN = 15;
    private static final int STATE_DEPOSIT_CYCLE2_PAUSE2 = 16;
    private static final int STATE_DEPOSIT_CYCLE2_INTAKE2_RUN2 = 17;
    private static final int STATE_START_PATH5 = 18;
    private static final int STATE_PATH5_MOVING = 19;
    private static final int STATE_DONE = -1;

    // ========== INSTANCE VARIABLES ==========
    private ElapsedTime actionTimer;

    // Deposit manual/auto control
    private boolean depositEnabled = true; // when true, motors try to reach target ticks/sec
    private boolean lastAState = false;     // for edge-detecting the A button

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(120.316, 128.707, Math.toRadians(38))); // Red Top starting pose

        paths = new Paths(follower); // Build paths

        // Initialize action timer
        actionTimer = new ElapsedTime();

        // Initialize intake motors
        try {
            intake1 = hardwareMap.get(DcMotor.class, "intake1");
            if (intake1 != null) {
                intake1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                intake1.setPower(0);
            }
        } catch (Exception e) {
            intake1 = null;
        }

        try {
            intake2 = hardwareMap.get(DcMotor.class, "intake2");
            if (intake2 != null) {
                intake2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                intake2.setPower(0);
            }
        } catch (Exception e) {
            intake2 = null;
        }

        // Initialize deposit motors for velocity control (matching DepositTuner)
        try {
            depositMotorL = hardwareMap.get(DcMotorEx.class, "DepositMotorL");
            if (depositMotorL != null) {
                depositMotorL.setDirection(DcMotorSimple.Direction.REVERSE); // Reverse left motor for auto
                depositMotorL.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
                depositMotorL.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER); // Required for velocity control
                depositMotorL.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
                // Set velocity PID coefficients for stability (tune these if needed)
                depositMotorL.setVelocityPIDFCoefficients(15, 0.5, 3, 12.6);
                depositMotorL.setPower(0);
            }
        } catch (Exception e) {
            depositMotorL = null;
        }

        try {
            depositMotorR = hardwareMap.get(DcMotorEx.class, "DepositMotorR");
            if (depositMotorR != null) {
                depositMotorR.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
                depositMotorR.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER); // Required for velocity control
                depositMotorR.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
                // Set velocity PID coefficients for stability (tune these if needed)
                depositMotorR.setVelocityPIDFCoefficients(15, 0.5, 3, 12.6);
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
        panelsTelemetry.debug("Status", "Initialized - Red Top Path");
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

        if (intake1 != null) panelsTelemetry.debug("Intake1 Power", intake1.getPower());
        if (intake2 != null) panelsTelemetry.debug("Intake2 Power", intake2.getPower());
        panelsTelemetry.debug("ActionTimer", String.format(java.util.Locale.US, "%.1f s", actionTimer.seconds()));

        panelsTelemetry.debug("Controls", "DPAD: Up/Down=±100 Left/Right=±10 | A=Toggle Deposit");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void stop() {
        // Stop all motors
        if (intake1 != null) intake1.setPower(0);
        if (intake2 != null) intake2.setPower(0);
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

    private void startIntake1() {
        if (intake1 != null) intake1.setPower(INTAKE1_POWER);
    }

    private void stopIntake1() {
        if (intake1 != null) intake1.setPower(0);
    }

    private void startIntake2() {
        if (intake2 != null) intake2.setPower(INTAKE2_POWER);
    }

    private void stopIntake2() {
        if (intake2 != null) intake2.setPower(0);
    }

    public static class Paths {

        // Red Top path segments
        public PathChain Path1;
        public PathChain Path2;
        public PathChain Path3;
        public PathChain Path4;
        public PathChain Path5;

        public Paths(Follower follower) {
            Path1 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(120.316, 128.707), new Pose(82.286, 86.617))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(38), Math.toRadians(42))
                    .build();

            Path2 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(82.286, 86.617), new Pose(93.925, 84.451))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(180))
                    .build();

            Path3 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(93.925, 84.451), new Pose(128.842, 83.910))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            Path4 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(128.842, 83.910), new Pose(82.421, 86.887))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(43))
                    .build();

            Path5 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(82.421, 86.887),
                                    new Pose(97.038, 70.376),
                                    new Pose(126.271, 72.000)
                            )
                    )
                    .setTangentHeadingInterpolation()
                    .build();
        }
    }

    public int autonomousPathUpdate() {
        switch (pathState) {
            case STATE_START_PATH1:
                // Start following path to point 1, enable deposit and intake1
                follower.followPath(paths.Path1);
                depositEnabled = true;
                startIntake1();
                pathState = STATE_PATH1_MOVING;
                break;

            case STATE_PATH1_MOVING:
                // Deposit and intake1 running while moving to point 1
                if (!follower.isBusy()) {
                    // Reached end of Path1, start deposit cycle 1
                    stopIntake1(); // Pause intake1
                    actionTimer.reset();
                    pathState = STATE_DEPOSIT_CYCLE1_PAUSE;
                }
                break;

            // ===== DEPOSIT CYCLE 1 (at end of Path1) =====
            case STATE_DEPOSIT_CYCLE1_PAUSE:
                // Pause before starting deposit cycle
                if (actionTimer.seconds() >= DEPOSIT_CYCLE_PAUSE_INTAKE1_S) {
                    startIntake2(); // Start intake2
                    actionTimer.reset();
                    pathState = STATE_DEPOSIT_CYCLE1_INTAKE2_RUN1;
                }
                break;

            case STATE_DEPOSIT_CYCLE1_INTAKE2_RUN1:
                // Run intake2 for 1.5 seconds
                if (actionTimer.seconds() >= DEPOSIT_CYCLE_INTAKE2_RUN_S) {
                    stopIntake2();
                    startIntake1(); // Run intake1
                    actionTimer.reset();
                    pathState = STATE_DEPOSIT_CYCLE1_INTAKE1_RUN;
                }
                break;

            case STATE_DEPOSIT_CYCLE1_INTAKE1_RUN:
                // Run intake1 for 1.5 seconds
                if (actionTimer.seconds() >= DEPOSIT_CYCLE_INTAKE1_RUN_S) {
                    stopIntake1();
                    actionTimer.reset();
                    pathState = STATE_DEPOSIT_CYCLE1_PAUSE2;
                }
                break;

            case STATE_DEPOSIT_CYCLE1_PAUSE2:
                // Pause for 1.5 seconds
                if (actionTimer.seconds() >= DEPOSIT_CYCLE_PAUSE_S) {
                    startIntake2(); // Run intake2 again
                    actionTimer.reset();
                    pathState = STATE_DEPOSIT_CYCLE1_INTAKE2_RUN2;
                }
                break;

            case STATE_DEPOSIT_CYCLE1_INTAKE2_RUN2:
                // Run intake2 again for 1.5 seconds
                if (actionTimer.seconds() >= DEPOSIT_CYCLE_INTAKE2_RUN2_S) {
                    stopIntake2();
                    pathState = STATE_START_PATH2;
                }
                break;

            // ===== PATH 2 =====
            case STATE_START_PATH2:
                // Start Path2, resume intake1
                follower.followPath(paths.Path2);
                startIntake1();
                pathState = STATE_PATH2_MOVING;
                break;

            case STATE_PATH2_MOVING:
                // Deposit and intake1 running while moving
                if (!follower.isBusy()) {
                    pathState = STATE_START_PATH3;
                }
                break;

            // ===== PATH 3 =====
            case STATE_START_PATH3:
                // Start Path3
                follower.followPath(paths.Path3);
                pathState = STATE_PATH3_MOVING;
                break;

            case STATE_PATH3_MOVING:
                // Deposit and intake1 running while moving
                if (!follower.isBusy()) {
                    pathState = STATE_START_PATH4;
                }
                break;

            // ===== PATH 4 =====
            case STATE_START_PATH4:
                // Start Path4
                follower.followPath(paths.Path4);
                pathState = STATE_PATH4_MOVING;
                break;

            case STATE_PATH4_MOVING:
                // Deposit and intake1 running while moving
                if (!follower.isBusy()) {
                    // Reached end of Path4, start deposit cycle 2
                    stopIntake1(); // Pause intake1
                    actionTimer.reset();
                    pathState = STATE_DEPOSIT_CYCLE2_PAUSE;
                }
                break;

            // ===== DEPOSIT CYCLE 2 (at end of Path4) =====
            case STATE_DEPOSIT_CYCLE2_PAUSE:
                // Pause before starting deposit cycle
                if (actionTimer.seconds() >= DEPOSIT_CYCLE_PAUSE_INTAKE1_S) {
                    startIntake2(); // Start intake2
                    actionTimer.reset();
                    pathState = STATE_DEPOSIT_CYCLE2_INTAKE2_RUN1;
                }
                break;

            case STATE_DEPOSIT_CYCLE2_INTAKE2_RUN1:
                // Run intake2 for 1.5 seconds
                if (actionTimer.seconds() >= DEPOSIT_CYCLE_INTAKE2_RUN_S) {
                    stopIntake2();
                    startIntake1(); // Run intake1
                    actionTimer.reset();
                    pathState = STATE_DEPOSIT_CYCLE2_INTAKE1_RUN;
                }
                break;

            case STATE_DEPOSIT_CYCLE2_INTAKE1_RUN:
                // Run intake1 for 1.5 seconds
                if (actionTimer.seconds() >= DEPOSIT_CYCLE_INTAKE1_RUN_S) {
                    stopIntake1();
                    actionTimer.reset();
                    pathState = STATE_DEPOSIT_CYCLE2_PAUSE2;
                }
                break;

            case STATE_DEPOSIT_CYCLE2_PAUSE2:
                // Pause for 1.5 seconds
                if (actionTimer.seconds() >= DEPOSIT_CYCLE_PAUSE_S) {
                    startIntake2(); // Run intake2 again
                    actionTimer.reset();
                    pathState = STATE_DEPOSIT_CYCLE2_INTAKE2_RUN2;
                }
                break;

            case STATE_DEPOSIT_CYCLE2_INTAKE2_RUN2:
                // Run intake2 again for 1.5 seconds
                if (actionTimer.seconds() >= DEPOSIT_CYCLE_INTAKE2_RUN2_S) {
                    stopIntake2();
                    pathState = STATE_START_PATH5;
                }
                break;

            // ===== PATH 5 =====
            case STATE_START_PATH5:
                // Start final Path5
                follower.followPath(paths.Path5);
                pathState = STATE_PATH5_MOVING;
                break;

            case STATE_PATH5_MOVING:
                // Deposit still running (intake1 stopped)
                if (!follower.isBusy()) {
                    // Reached final point, stop everything
                    depositEnabled = false;
                    stopDeposit();
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
