package org.firstinspires.ftc.teamcode.Autonomous;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name = "Pedro Pathing Autonomous", group = "Autonomous")
@Configurable // Panels
public class PedroAutonomous extends OpMode {

    private TelemetryManager panelsTelemetry; // Panels Telemetry instance
    public Follower follower; // Pedro Pathing follower instance
    private int pathState; // Current autonomous path state (state machine)
    private Paths paths; // Paths defined in the Paths class

    // Timers used by the state machine
    private ElapsedTime pathTimer;
    private ElapsedTime opmodeTimer;
    private ElapsedTime actionTimer;

    // Hardware for intake/deposit (ASSUMPTION: names in robot config)
    // If your config uses different names, update them accordingly.
    private DcMotor intakeMotor;
    private Servo depositServo;

    // Intake/Deposit constants (tweak as needed)
    private static final double INTAKE_POWER = 1.0;
    private static final double OUTTAKE_POWER = -0.6;
    private static final double DEPOSIT_SERVO_DEPLOY = 0.8;
    private static final double DEPOSIT_SERVO_STOW = 0.2;
    private static final double INTAKE_DURATION_S = 1.2; // how long to run intake
    private static final double DEPOSIT_DURATION_S = 0.7; // how long to run outtake

    // Path/state constants
    private static final int STATE_START_PATH1 = 0;
    private static final int STATE_WAIT_FOR_PATH1 = 1;
    private static final int STATE_RUN_INTAKE = 2;
    private static final int STATE_DEPOSIT = 3;
    private static final int STATE_PAUSE_AFTER_DEPOSIT = 4;
    private static final int STATE_DONE = -1;

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = org.firstinspires.ftc.teamcode.pedroPathing.Constants.createFollower(hardwareMap);
        // Set starting pose as requested (72, 8, 90°)
        follower.setStartingPose(new Pose(72, 8, Math.toRadians(90)));

        // Build paths
        paths = new Paths(follower);

        // Timers
        pathTimer = new ElapsedTime();
        opmodeTimer = new ElapsedTime();
        opmodeTimer.reset();
        actionTimer = new ElapsedTime();

        // Initialize intake/deposit hardware
        // Assumption: hardware map names are "intakeMotor" and "depositServo". Change if different.
        try {
            intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        } catch (Exception e) {
            intakeMotor = null; // safe fallback; code checks for null
        }
        try {
            depositServo = hardwareMap.get(Servo.class, "depositServo");
        } catch (Exception e) {
            depositServo = null;
        }

        if (intakeMotor != null) intakeMotor.setPower(0);
        if (depositServo != null) depositServo.setPosition(DEPOSIT_SERVO_STOW);

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void init_loop() {
        // no-op
    }

    @Override
    public void start() {
        // Start the FSM at state 0
        setPathState(STATE_START_PATH1);
        opmodeTimer.reset();
    }

    @Override
    public void loop() {
        // Update follower and FSM
        follower.update();
        autonomousPathUpdate();

        // Log values to Panels and Driver Station
        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", follower.getPose().getHeading());
        panelsTelemetry.debug("PathTimer", pathTimer.seconds());
        panelsTelemetry.debug("OpModeTimer", opmodeTimer.seconds());
        if (intakeMotor != null) panelsTelemetry.debug("IntakePower", intakeMotor.getPower());
        if (depositServo != null) panelsTelemetry.debug("DepositServo", depositServo.getPosition());
        panelsTelemetry.debug("ActionTimer", actionTimer.seconds());
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void stop() {
        // stop actions if needed
        if (intakeMotor != null) intakeMotor.setPower(0);
    }

    public static class Paths {

        public PathChain Path1;

        public Paths(Follower follower) {
            // Updated to use the supplied BezierCurve and heading interpolation
            Path1 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(55.374, 6.763),
                                    new Pose(56.642, 72.282),
                                    new Pose(74.395, 77.777)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(40))
                    .build();
        }
    }

    public void setPathState(int pState) {
        pathState = pState;
        if (pathTimer != null) pathTimer.reset();
        if (actionTimer != null) actionTimer.reset();
    }

    // Helper methods for intake/deposit (safe if hardware missing)
    private void startIntake() {
        if (intakeMotor != null) intakeMotor.setPower(INTAKE_POWER);
    }

    private void stopIntake() {
        if (intakeMotor != null) intakeMotor.setPower(0);
    }

    private void startOuttake() {
        if (intakeMotor != null) intakeMotor.setPower(OUTTAKE_POWER);
    }

    private void deployDepositServo() {
        if (depositServo != null) depositServo.setPosition(DEPOSIT_SERVO_DEPLOY);
    }

    private void stowDepositServo() {
        if (depositServo != null) depositServo.setPosition(DEPOSIT_SERVO_STOW);
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case STATE_START_PATH1:
                // Start following Path1
                follower.followPath(paths.Path1);
                setPathState(STATE_WAIT_FOR_PATH1);
                break;

            case STATE_WAIT_FOR_PATH1:
                // Wait until follower finishes the path
                if (!follower.isBusy()) {
                    // Hold at endpoint and start intake routine
                    follower.followPath(paths.Path1, true); // hold pose
                    actionTimer.reset();
                    startIntake();
                    setPathState(STATE_RUN_INTAKE);
                }
                break;

            case STATE_RUN_INTAKE:
                // run intake for configured duration then stop and perform deposit
                if (actionTimer.seconds() >= INTAKE_DURATION_S) {
                    stopIntake();
                    // perform deposit (at same location)
                    deployDepositServo();
                    startOuttake();
                    actionTimer.reset();
                    setPathState(STATE_DEPOSIT);
                }
                break;

            case STATE_DEPOSIT:
                if (actionTimer.seconds() >= DEPOSIT_DURATION_S) {
                    stopIntake();
                    stowDepositServo();
                    // small pause after deposit, then finish
                    actionTimer.reset();
                    setPathState(STATE_PAUSE_AFTER_DEPOSIT);
                }
                break;

            case STATE_PAUSE_AFTER_DEPOSIT:
                if (actionTimer.seconds() >= 0.5) {
                    setPathState(STATE_DONE);
                }
                break;

            case STATE_DONE:
                // Idle state - do nothing
                break;

            default:
                // Idle
                break;
        }
    }
}
