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
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.common.PIDController;

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

    // ========== TUNING VARIABLES ==========
    // Intake Configuration
    private static final double INTAKE_POWER = -0.9; // Negative to run in correct direction
    private static final double INTAKE_DURATION_S = 1.5;

    // Deposit Velocity Control Configuration (850 ticks/sec target)
    private static final double DEPOSIT_SPEED_TARGET = 850.0; // encoder ticks per second
    private static final double DEPOSIT_KP = 0.0008; // Proportional gain - increase if response is too slow, decrease if overshooting
    private static final double DEPOSIT_KI = 0.0000015; // Integral gain - helps eliminate steady-state error
    private static final double DEPOSIT_KD = 0.00005; // Derivative gain - helps reduce oscillation
    private static final double DEPOSIT_KFF = 0.00025; // Feedforward - initial push to reach target speed faster
    private static final double DEPOSIT_PID_OUTPUT_LIMIT = 1.0; // Max motor power (0-1.0)
    private static final double DEPOSIT_PID_INTEGRAL_LIMIT = 2000; // Prevent integral windup

    // State Machine Timing Configuration
    private static final double WAIT_BEFORE_INTAKE_S = 2.0; // Wait 2 seconds before starting intake

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
    private PIDController depositPID;
    private PathSelection selectedPath = PathSelection.BLUE_BOTTOM; // Default selection
    private boolean pathSelected = false;

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

        // Initialize deposit motors for velocity control
        try {
            depositMotorL = hardwareMap.get(DcMotorEx.class, "DepositMotorL");
            if (depositMotorL != null) {
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
                depositMotorR.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
                depositMotorR.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
                depositMotorR.setPower(0);
            }
        } catch (Exception e) {
            depositMotorR = null;
        }

        // Initialize PID controller for deposit velocity control
        depositPID = new PIDController(DEPOSIT_KP, DEPOSIT_KI, DEPOSIT_KD);
        depositPID.setOutputLimits(-DEPOSIT_PID_OUTPUT_LIMIT, DEPOSIT_PID_OUTPUT_LIMIT);
        depositPID.setIntegratorLimits(-DEPOSIT_PID_INTEGRAL_LIMIT, DEPOSIT_PID_INTEGRAL_LIMIT);

        panelsTelemetry.debug("Status", "Initialized - Select a path");
        panelsTelemetry.debug("D-Pad Up: Blue Bottom");
        panelsTelemetry.debug("D-Pad Right: Blue Top");
        panelsTelemetry.debug("D-Pad Down: Red Bottom");
        panelsTelemetry.debug("D-Pad Left: Red Top");
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

        // Log values to Panels and Driver Station
        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", follower.getPose().getHeading());
        if (intakeMotor != null) panelsTelemetry.debug("Intake Power", intakeMotor.getPower());
        if (depositMotorL != null) panelsTelemetry.debug("DepositMotorL Vel", depositMotorL.getVelocity());
        if (depositMotorR != null) panelsTelemetry.debug("DepositMotorR Vel", depositMotorR.getVelocity());
        panelsTelemetry.debug("ActionTimer", actionTimer.seconds());
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void stop() {
        // Stop all motors
        if (intakeMotor != null) intakeMotor.setPower(0);
        if (depositMotorL != null) depositMotorL.setPower(0);
        if (depositMotorR != null) depositMotorR.setPower(0);
    }

    private void runDepositAtSpeed() {
        if (depositMotorL == null || depositMotorR == null) return;

        double currentVelocity = (depositMotorL.getVelocity() + depositMotorR.getVelocity()) / 2.0;

        // Calculate PID output (setpoint, measurement, dt)
        double pidOutput = depositPID.update(DEPOSIT_SPEED_TARGET, currentVelocity, 0.016); // ~60 Hz loop
        double feedForward = DEPOSIT_SPEED_TARGET * DEPOSIT_KFF;
        double totalPower = pidOutput + feedForward;

        depositMotorL.setPower(-totalPower); // Negative to reverse direction
        depositMotorR.setPower(totalPower);
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
                            new BezierLine(new Pose(54.677, 7.038), new Pose(73.218, 86.346))
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
                // Start following the selected path and start deposit revving
                follower.followPath(paths.getPath(selectedPath));
                depositPID.reset();
                pathState = STATE_PATH_AND_DEPOSIT;
                break;

            case STATE_PATH_AND_DEPOSIT:
                // Run deposit the whole time while following path
                runDepositAtSpeed();
                // Wait until follower finishes the path
                if (!follower.isBusy()) {
                    // Path complete, start wait timer
                    actionTimer.reset();
                    pathState = STATE_WAIT_BEFORE_INTAKE;
                }
                break;

            case STATE_WAIT_BEFORE_INTAKE:
                // Run deposit while waiting
                runDepositAtSpeed();
                // Wait 1 second before starting intake
                if (actionTimer.seconds() >= WAIT_BEFORE_INTAKE_S) {
                    actionTimer.reset();
                    startIntake();
                    pathState = STATE_INTAKE;
                }
                break;

            case STATE_INTAKE:
                // Run intake for configured duration
                runDepositAtSpeed(); // Keep deposit running
                if (actionTimer.seconds() >= INTAKE_DURATION_S) {
                    stopIntake();
                    pathState = STATE_DONE;
                }
                break;

            case STATE_DONE:
                // Continue running deposit until autonomous ends
                runDepositAtSpeed();
                break;

            default:
                // Idle
                break;
        }

        return pathState;
    }
}

