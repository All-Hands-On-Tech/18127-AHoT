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
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Trowel.pedroPathing.Constants;

@Autonomous(name = "TrowelAuto", group = "Trowel")
@Configurable
public class TrowelAutonomous extends OpMode {

    private TelemetryManager panelsTelemetry;
    public Follower follower;
    private int pathState;
    private Paths paths;

    private DcMotor intake1;
    private DcMotor intake2;
    private DcMotorEx deposit1;
    private DcMotorEx deposit2;
    private Servo transfer1;
    private Servo transfer2;

    // Tuning variables
    public static double INTAKE1_POWER = 1.0;
    public static double INTAKE2_POWER = 1.0;
    public static double DEPOSIT_CYCLE_TIME_S = 1.25;
    public static double FIRST_SHOT_DELAY_S = 2.5;
    public static double SECOND_SHOT_DELAY_S = 0.5;
    public static double TRANSFER_IN = 0.0;
    public static double TRANSFER_OUT = 1.0;
    public static double TRANSFER_NEUTRAL = 0.5;
    public static double depositTargetVelocity = 1800.0;

    // Velocity tuning bounds
    private static final double MIN_VELOCITY = 0.0;
    private static final double MAX_VELOCITY = 5000.0;
    private static final double STEP_SMALL = 10.0;
    private static final double STEP_LARGE = 100.0;

    private boolean prevDpadUp = false, prevDpadDown = false, prevDpadLeft = false, prevDpadRight = false;
    private long lastDpadChange = System.currentTimeMillis();
    private static final long FIRST_REPEAT_DELAY_MS = 350;
    private static final long REPEAT_INTERVAL_MS = 120;

    // Alliance selection
    public enum Alliance { NONE, BLUE, RED }
    private Alliance selectedAlliance = Alliance.NONE;

    // State machine constants
    private static final int STATE_START_PATH1 = 0;
    private static final int STATE_PATH1_MOVING = 1;
    private static final int STATE_DEPOSIT_CYCLE1_FIRST_DELAY = 2;
    private static final int STATE_DEPOSIT_CYCLE1_FIRST_SHOT = 3;
    private static final int STATE_DEPOSIT_CYCLE1_SECOND_DELAY = 4;
    private static final int STATE_DEPOSIT_CYCLE1_SECOND_SHOT = 5;

    private static final int STATE_START_PATH2 = 6;
    private static final int STATE_PATH2_MOVING = 7;
    private static final int STATE_START_PATH3 = 8;
    private static final int STATE_PATH3_MOVING = 9;
    private static final int STATE_START_PATH4 = 10;
    private static final int STATE_PATH4_MOVING = 11;

    private static final int STATE_DEPOSIT_CYCLE2_FIRST_DELAY = 12;
    private static final int STATE_DEPOSIT_CYCLE2_FIRST_SHOT = 13;
    private static final int STATE_DEPOSIT_CYCLE2_SECOND_DELAY = 14;
    private static final int STATE_DEPOSIT_CYCLE2_SECOND_SHOT = 15;

    private static final int STATE_START_PATH5 = 16;
    private static final int STATE_PATH5_MOVING = 17;
    private static final int STATE_START_PATH6 = 18;
    private static final int STATE_PATH6_MOVING = 19;
    private static final int STATE_START_PATH7 = 20;
    private static final int STATE_PATH7_MOVING = 21;

    private static final int STATE_DEPOSIT_CYCLE3_FIRST_DELAY = 22;
    private static final int STATE_DEPOSIT_CYCLE3_FIRST_SHOT = 23;
    private static final int STATE_DEPOSIT_CYCLE3_SECOND_DELAY = 24;
    private static final int STATE_DEPOSIT_CYCLE3_SECOND_SHOT = 25;

    private static final int STATE_START_PATH8 = 26;
    private static final int STATE_PATH8_MOVING = 27;
    private static final int STATE_START_PATH9 = 28;
    private static final int STATE_PATH9_MOVING = 29;
    private static final int STATE_START_PATH10 = 30;
    private static final int STATE_PATH10_MOVING = 31;

    private static final int STATE_DEPOSIT_CYCLE4_FIRST_DELAY = 32;
    private static final int STATE_DEPOSIT_CYCLE4_FIRST_SHOT = 33;
    private static final int STATE_DEPOSIT_CYCLE4_SECOND_DELAY = 34;
    private static final int STATE_DEPOSIT_CYCLE4_SECOND_SHOT = 35;

    private static final int STATE_START_PATH11 = 36;
    private static final int STATE_PATH11_MOVING = 37;

    private static final int STATE_DONE = 38;

    private ElapsedTime actionTimer;
    private boolean depositEnabled = true;
    private boolean lastAState = false;

    // PIDF for deposit motors - tuned for stable velocity control
    private static final PIDFCoefficients DEPOSIT_PIDF = new PIDFCoefficients(30.0, 0.02, 20.0, 14.0);

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        follower = Constants.createFollower(hardwareMap);
        actionTimer = new ElapsedTime();

        // Initialize intake1
        try {
            intake1 = hardwareMap.get(DcMotor.class, "intake1");
            if (intake1 != null) {
                intake1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                intake1.setPower(0);
            }
        } catch (Exception e) {
            intake1 = null;
        }

        // Initialize intake2
        try {
            intake2 = hardwareMap.get(DcMotor.class, "intake2");
            if (intake2 != null) {
                intake2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                intake2.setPower(0);
            }
        } catch (Exception e) {
            intake2 = null;
        }

        // Initialize deposit1
        try {
            deposit1 = hardwareMap.get(DcMotorEx.class, "deposit1");
            if (deposit1 != null) {
                deposit1.setDirection(DcMotor.Direction.FORWARD);
                deposit1.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
                deposit1.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
                deposit1.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
                deposit1.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, DEPOSIT_PIDF);
                deposit1.setPower(0);
            }
        } catch (Exception e) {
            deposit1 = null;
        }

        // Initialize deposit2
        try {
            deposit2 = hardwareMap.get(DcMotorEx.class, "deposit2");
            if (deposit2 != null) {
                deposit2.setDirection(DcMotor.Direction.REVERSE);
                deposit2.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
                deposit2.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
                deposit2.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
                deposit2.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, DEPOSIT_PIDF);
                deposit2.setPower(0);
            }
        } catch (Exception e) {
            deposit2 = null;
        }

        // Initialize transfer servos
        try {
            transfer1 = hardwareMap.get(Servo.class, "transfer1");
            if (transfer1 != null) {
                transfer1.setPosition(TRANSFER_NEUTRAL);
            }
        } catch (Exception e) {
            transfer1 = null;
        }

        try {
            transfer2 = hardwareMap.get(Servo.class, "transfer2");
            if (transfer2 != null) {
                transfer2.setDirection(Servo.Direction.REVERSE);
                transfer2.setPosition(TRANSFER_NEUTRAL);
            }
        } catch (Exception e) {
            transfer2 = null;
        }

        panelsTelemetry.debug("Status", "Initialized - Alliance Selection");
        panelsTelemetry.debug("Controls", "X = BLUE | B = RED");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void init_loop() {
        // Alliance selection during init
        if (gamepad1.x && selectedAlliance != Alliance.BLUE) {
            selectedAlliance = Alliance.BLUE;
            // Blue starting pose from blue path data
            follower.setStartingPose(new Pose(56.165, 9.474, Math.toRadians(90)));
            paths = new Paths(follower, selectedAlliance);
        } else if (gamepad1.b && selectedAlliance != Alliance.RED) {
            selectedAlliance = Alliance.RED;
            // Red starting pose from path data
            follower.setStartingPose(new Pose(87.158, 9.474, Math.toRadians(90)));
            paths = new Paths(follower, selectedAlliance);
        }

        String allianceStr = selectedAlliance == Alliance.BLUE ? "BLUE" :
                             selectedAlliance == Alliance.RED ? "RED" : "NONE";
        panelsTelemetry.debug("Alliance", allianceStr);
        panelsTelemetry.debug("Controls", "X = BLUE | B = RED");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void start() {
        if (selectedAlliance == Alliance.NONE) {
            selectedAlliance = Alliance.RED;
            follower.setStartingPose(new Pose(87.158, 9.474, Math.toRadians(90)));
            paths = new Paths(follower, selectedAlliance);
        }
    }

    @Override
    public void loop() {
        follower.update();
        pathState = autonomousPathUpdate();

        // Velocity tuning with dpad
        boolean dpadUp = gamepad1.dpad_up;
        boolean dpadDown = gamepad1.dpad_down;
        boolean dpadLeft = gamepad1.dpad_left;
        boolean dpadRight = gamepad1.dpad_right;

        long now = System.currentTimeMillis();

        if (dpadUp && (!prevDpadUp || now - lastDpadChange > FIRST_REPEAT_DELAY_MS)) {
            depositTargetVelocity = Math.min(MAX_VELOCITY, depositTargetVelocity + STEP_LARGE);
            lastDpadChange = now;
        }
        if (dpadDown && (!prevDpadDown || now - lastDpadChange > FIRST_REPEAT_DELAY_MS)) {
            depositTargetVelocity = Math.max(MIN_VELOCITY, depositTargetVelocity - STEP_LARGE);
            lastDpadChange = now;
        }
        if (dpadRight && (!prevDpadRight || now - lastDpadChange > FIRST_REPEAT_DELAY_MS)) {
            depositTargetVelocity = Math.min(MAX_VELOCITY, depositTargetVelocity + STEP_SMALL);
            lastDpadChange = now;
        }
        if (dpadLeft && (!prevDpadLeft || now - lastDpadChange > FIRST_REPEAT_DELAY_MS)) {
            depositTargetVelocity = Math.max(MIN_VELOCITY, depositTargetVelocity - STEP_SMALL);
            lastDpadChange = now;
        }

        if ((dpadUp || dpadDown || dpadLeft || dpadRight) && now - lastDpadChange > REPEAT_INTERVAL_MS) {
            lastDpadChange = now - (REPEAT_INTERVAL_MS + 1);
        }

        prevDpadUp = dpadUp;
        prevDpadDown = dpadDown;
        prevDpadLeft = dpadLeft;
        prevDpadRight = dpadRight;

        // Toggle deposit with A button
        boolean aPressed = gamepad1.a;
        if (aPressed && !lastAState) {
            depositEnabled = !depositEnabled;
        }
        lastAState = aPressed;

        if (depositEnabled) {
            runDepositAtVelocity();
        } else {
            stopDeposit();
        }

        // Telemetry
        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", follower.getPose().getHeading());
        panelsTelemetry.debug("Deposit Target", String.format(java.util.Locale.US, "%.0f ticks/s", depositTargetVelocity));
        panelsTelemetry.debug("Deposit Enabled", depositEnabled);

        if (deposit1 != null) {
            double v1 = deposit1.getVelocity();
            panelsTelemetry.debug("Deposit1 Vel", String.format(java.util.Locale.US, "%.0f", v1));
        }
        if (deposit2 != null) {
            double v2 = deposit2.getVelocity();
            panelsTelemetry.debug("Deposit2 Vel", String.format(java.util.Locale.US, "%.0f", v2));
        }

        if (intake1 != null) panelsTelemetry.debug("Intake1", intake1.getPower());
        if (intake2 != null) panelsTelemetry.debug("Intake2", intake2.getPower());
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void stop() {
        if (intake1 != null) intake1.setPower(0);
        if (intake2 != null) intake2.setPower(0);
        if (deposit1 != null) deposit1.setPower(0);
        if (deposit2 != null) deposit2.setPower(0);
    }

    private void runDepositAtVelocity() {
        if (deposit1 == null || deposit2 == null) return;
        deposit1.setVelocity(depositTargetVelocity);
        deposit2.setVelocity(depositTargetVelocity);
    }

    private void stopDeposit() {
        if (deposit1 == null || deposit2 == null) return;
        deposit1.setPower(0);
        deposit2.setPower(0);
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

    private void setTransferIn() {
        if (transfer1 != null) transfer1.setPosition(TRANSFER_IN);
        if (transfer2 != null) transfer2.setPosition(TRANSFER_IN);
    }

    private void setTransferOut() {
        if (transfer1 != null) transfer1.setPosition(TRANSFER_OUT);
        if (transfer2 != null) transfer2.setPosition(TRANSFER_OUT);
    }

    private void setTransferNeutral() {
        if (transfer1 != null) transfer1.setPosition(TRANSFER_NEUTRAL);
        if (transfer2 != null) transfer2.setPosition(TRANSFER_NEUTRAL);
    }

    public static class Paths {
        public PathChain Path1, Path2, Path3, Path4, Path5, Path6, Path7, Path8, Path9, Path10, Path11;

        public Paths(Follower follower, Alliance alliance) {
            if (alliance == Alliance.BLUE) {
                // Blue alliance paths from blue bottom path data
                Path1 = follower.pathBuilder()
                        .addPath(new BezierLine(
                                new Pose(56.165, 9.474),
                                new Pose(50.752, 94.602)))
                        .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(135))
                        .build();

                Path2 = follower.pathBuilder()
                        .addPath(new BezierCurve(
                                new Pose(50.752, 94.602),
                                new Pose(61.444, 29.504),
                                new Pose(46.150, 35.323)))
                        .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(0))
                        .build();

                Path3 = follower.pathBuilder()
                        .addPath(new BezierLine(
                                new Pose(46.150, 35.323),
                                new Pose(9.744, 36.0)))
                        .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                        .build();

                Path4 = follower.pathBuilder()
                        .addPath(new BezierCurve(
                                new Pose(9.744, 36.0),
                                new Pose(62.0, 40.0),
                                new Pose(50.617, 94.602)))
                        .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(135))
                        .build();

                Path5 = follower.pathBuilder()
                        .addPath(new BezierCurve(
                                new Pose(50.617, 94.602),
                                new Pose(69.699, 53.729),
                                new Pose(41.684, 57.519)))
                        .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(0))
                        .build();

                Path6 = follower.pathBuilder()
                        .addPath(new BezierLine(
                                new Pose(41.684, 57.519),
                                new Pose(10.150, 57.383)))
                        .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                        .build();

                Path7 = follower.pathBuilder()
                        .addPath(new BezierCurve(
                                new Pose(10.150, 57.383),
                                new Pose(76.737, 46.692),
                                new Pose(51.158, 94.872)))
                        .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(135))
                        .build();

                Path8 = follower.pathBuilder()
                        .addPath(new BezierCurve(
                                new Pose(51.158, 94.872),
                                new Pose(48.722, 88.917),
                                new Pose(41.143, 84.722)))
                        .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(0))
                        .build();

                Path9 = follower.pathBuilder()
                        .addPath(new BezierLine(
                                new Pose(41.143, 84.722),
                                new Pose(16.376, 84.451)))
                        .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                        .build();

                Path10 = follower.pathBuilder()
                        .addPath(new BezierLine(
                                new Pose(16.376, 84.451),
                                new Pose(51.293, 95.143)))
                        .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(135))
                        .build();

                Path11 = follower.pathBuilder()
                        .addPath(new BezierCurve(
                                new Pose(51.293, 95.143),
                                new Pose(30.586, 67.534),
                                new Pose(19.895, 69.564)))
                        .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(90))
                        .build();

            } else {
                // Red alliance paths from path data
                Path1 = follower.pathBuilder()
                        .addPath(new BezierLine(
                                new Pose(87.158, 9.474),
                                new Pose(85.940, 90.541)))
                        .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(45))
                        .build();

                Path2 = follower.pathBuilder()
                        .addPath(new BezierCurve(
                                new Pose(85.940, 90.541),
                                new Pose(93.519, 84.586),
                                new Pose(101.910, 83.233)))
                        .setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(180))
                        .build();

                Path3 = follower.pathBuilder()
                        .addPath(new BezierLine(
                                new Pose(101.910, 83.233),
                                new Pose(127.489, 83.233)))
                        .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                        .build();

                Path4 = follower.pathBuilder()
                        .addPath(new BezierLine(
                                new Pose(127.489, 83.233),
                                new Pose(86.211, 90.947)))
                        .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(45))
                        .build();

                Path5 = follower.pathBuilder()
                        .addPath(new BezierCurve(
                                new Pose(86.211, 90.947),
                                new Pose(88.105, 59.549),
                                new Pose(101.910, 59.549)))
                        .setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(180))
                        .build();

                Path6 = follower.pathBuilder()
                        .addPath(new BezierLine(
                                new Pose(101.910, 59.549),
                                new Pose(133.579, 58.195)))
                        .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                        .build();

                Path7 = follower.pathBuilder()
                        .addPath(new BezierLine(
                                new Pose(133.579, 58.195),
                                new Pose(86.075, 90.812)))
                        .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(45))
                        .build();

                Path8 = follower.pathBuilder()
                        .addPath(new BezierCurve(
                                new Pose(86.075, 90.812),
                                new Pose(92.301, 35.053),
                                new Pose(102.586, 34.782)))
                        .setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(180))
                        .build();

                Path9 = follower.pathBuilder()
                        .addPath(new BezierLine(
                                new Pose(102.586, 34.782),
                                new Pose(133.850, 35.053)))
                        .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                        .build();

                Path10 = follower.pathBuilder()
                        .addPath(new BezierLine(
                                new Pose(133.850, 35.053),
                                new Pose(86.211, 90.677)))
                        .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(45))
                        .build();

                Path11 = follower.pathBuilder()
                        .addPath(new BezierLine(
                                new Pose(86.211, 90.677),
                                new Pose(125.323, 69.970)))
                        .setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(90))
                        .build();
            }
        }
    }

    public int autonomousPathUpdate() {
        switch (pathState) {
            // ===== CYCLE 1: Path1 -> Deposit =====
            case STATE_START_PATH1:
                follower.followPath(paths.Path1);
                depositEnabled = true;
                startIntake1();
                pathState = STATE_PATH1_MOVING;
                break;

            case STATE_PATH1_MOVING:
                if (!follower.isBusy()) {
                    stopIntake1();
                    actionTimer.reset();
                    pathState = STATE_DEPOSIT_CYCLE1_FIRST_DELAY;
                }
                break;

            case STATE_DEPOSIT_CYCLE1_FIRST_DELAY:
                if (actionTimer.seconds() >= FIRST_SHOT_DELAY_S) {
                    startIntake1();
                    startIntake2();
                    setTransferIn();
                    actionTimer.reset();
                    pathState = STATE_DEPOSIT_CYCLE1_FIRST_SHOT;
                }
                break;

            case STATE_DEPOSIT_CYCLE1_FIRST_SHOT:
                if (actionTimer.seconds() >= DEPOSIT_CYCLE_TIME_S) {
                    stopIntake1();
                    stopIntake2();
                    setTransferNeutral();
                    actionTimer.reset();
                    pathState = STATE_DEPOSIT_CYCLE1_SECOND_DELAY;
                }
                break;

            case STATE_DEPOSIT_CYCLE1_SECOND_DELAY:
                if (actionTimer.seconds() >= SECOND_SHOT_DELAY_S) {
                    startIntake1();
                    startIntake2();
                    setTransferIn();
                    actionTimer.reset();
                    pathState = STATE_DEPOSIT_CYCLE1_SECOND_SHOT;
                }
                break;

            case STATE_DEPOSIT_CYCLE1_SECOND_SHOT:
                if (actionTimer.seconds() >= DEPOSIT_CYCLE_TIME_S) {
                    stopIntake1();
                    stopIntake2();
                    setTransferNeutral();
                    pathState = STATE_START_PATH2;
                }
                break;

            // ===== PATHS 2-4: Pickup first sample =====
            case STATE_START_PATH2:
                follower.followPath(paths.Path2);
                startIntake1();
                pathState = STATE_PATH2_MOVING;
                break;

            case STATE_PATH2_MOVING:
                if (!follower.isBusy()) pathState = STATE_START_PATH3;
                break;

            case STATE_START_PATH3:
                follower.followPath(paths.Path3);
                pathState = STATE_PATH3_MOVING;
                break;

            case STATE_PATH3_MOVING:
                if (!follower.isBusy()) pathState = STATE_START_PATH4;
                break;

            case STATE_START_PATH4:
                follower.followPath(paths.Path4);
                stopIntake1();
                pathState = STATE_PATH4_MOVING;
                break;

            case STATE_PATH4_MOVING:
                if (!follower.isBusy()) {
                    actionTimer.reset();
                    pathState = STATE_DEPOSIT_CYCLE2_FIRST_DELAY;
                }
                break;

            // ===== CYCLE 2: Deposit =====
            case STATE_DEPOSIT_CYCLE2_FIRST_DELAY:
                if (actionTimer.seconds() >= FIRST_SHOT_DELAY_S) {
                    startIntake1();
                    startIntake2();
                    setTransferIn();
                    actionTimer.reset();
                    pathState = STATE_DEPOSIT_CYCLE2_FIRST_SHOT;
                }
                break;

            case STATE_DEPOSIT_CYCLE2_FIRST_SHOT:
                if (actionTimer.seconds() >= DEPOSIT_CYCLE_TIME_S) {
                    stopIntake1();
                    stopIntake2();
                    setTransferNeutral();
                    actionTimer.reset();
                    pathState = STATE_DEPOSIT_CYCLE2_SECOND_DELAY;
                }
                break;

            case STATE_DEPOSIT_CYCLE2_SECOND_DELAY:
                if (actionTimer.seconds() >= SECOND_SHOT_DELAY_S) {
                    startIntake1();
                    startIntake2();
                    setTransferIn();
                    actionTimer.reset();
                    pathState = STATE_DEPOSIT_CYCLE2_SECOND_SHOT;
                }
                break;

            case STATE_DEPOSIT_CYCLE2_SECOND_SHOT:
                if (actionTimer.seconds() >= DEPOSIT_CYCLE_TIME_S) {
                    stopIntake1();
                    stopIntake2();
                    setTransferNeutral();
                    pathState = STATE_START_PATH5;
                }
                break;

            // ===== PATHS 5-7: Pickup second sample =====
            case STATE_START_PATH5:
                follower.followPath(paths.Path5);
                startIntake1();
                pathState = STATE_PATH5_MOVING;
                break;

            case STATE_PATH5_MOVING:
                if (!follower.isBusy()) pathState = STATE_START_PATH6;
                break;

            case STATE_START_PATH6:
                follower.followPath(paths.Path6);
                pathState = STATE_PATH6_MOVING;
                break;

            case STATE_PATH6_MOVING:
                if (!follower.isBusy()) pathState = STATE_START_PATH7;
                break;

            case STATE_START_PATH7:
                follower.followPath(paths.Path7);
                stopIntake1();
                pathState = STATE_PATH7_MOVING;
                break;

            case STATE_PATH7_MOVING:
                if (!follower.isBusy()) {
                    actionTimer.reset();
                    pathState = STATE_DEPOSIT_CYCLE3_FIRST_DELAY;
                }
                break;

            // ===== CYCLE 3: Deposit =====
            case STATE_DEPOSIT_CYCLE3_FIRST_DELAY:
                if (actionTimer.seconds() >= FIRST_SHOT_DELAY_S) {
                    startIntake1();
                    startIntake2();
                    setTransferIn();
                    actionTimer.reset();
                    pathState = STATE_DEPOSIT_CYCLE3_FIRST_SHOT;
                }
                break;

            case STATE_DEPOSIT_CYCLE3_FIRST_SHOT:
                if (actionTimer.seconds() >= DEPOSIT_CYCLE_TIME_S) {
                    stopIntake1();
                    stopIntake2();
                    setTransferNeutral();
                    actionTimer.reset();
                    pathState = STATE_DEPOSIT_CYCLE3_SECOND_DELAY;
                }
                break;

            case STATE_DEPOSIT_CYCLE3_SECOND_DELAY:
                if (actionTimer.seconds() >= SECOND_SHOT_DELAY_S) {
                    startIntake1();
                    startIntake2();
                    setTransferIn();
                    actionTimer.reset();
                    pathState = STATE_DEPOSIT_CYCLE3_SECOND_SHOT;
                }
                break;

            case STATE_DEPOSIT_CYCLE3_SECOND_SHOT:
                if (actionTimer.seconds() >= DEPOSIT_CYCLE_TIME_S) {
                    stopIntake1();
                    stopIntake2();
                    setTransferNeutral();
                    pathState = STATE_START_PATH8;
                }
                break;

            // ===== PATHS 8-10: Pickup third sample =====
            case STATE_START_PATH8:
                follower.followPath(paths.Path8);
                startIntake1();
                pathState = STATE_PATH8_MOVING;
                break;

            case STATE_PATH8_MOVING:
                if (!follower.isBusy()) pathState = STATE_START_PATH9;
                break;

            case STATE_START_PATH9:
                follower.followPath(paths.Path9);
                pathState = STATE_PATH9_MOVING;
                break;

            case STATE_PATH9_MOVING:
                if (!follower.isBusy()) pathState = STATE_START_PATH10;
                break;

            case STATE_START_PATH10:
                follower.followPath(paths.Path10);
                stopIntake1();
                pathState = STATE_PATH10_MOVING;
                break;

            case STATE_PATH10_MOVING:
                if (!follower.isBusy()) {
                    actionTimer.reset();
                    pathState = STATE_DEPOSIT_CYCLE4_FIRST_DELAY;
                }
                break;

            // ===== CYCLE 4: Deposit =====
            case STATE_DEPOSIT_CYCLE4_FIRST_DELAY:
                if (actionTimer.seconds() >= FIRST_SHOT_DELAY_S) {
                    startIntake1();
                    startIntake2();
                    setTransferIn();
                    actionTimer.reset();
                    pathState = STATE_DEPOSIT_CYCLE4_FIRST_SHOT;
                }
                break;

            case STATE_DEPOSIT_CYCLE4_FIRST_SHOT:
                if (actionTimer.seconds() >= DEPOSIT_CYCLE_TIME_S) {
                    stopIntake1();
                    stopIntake2();
                    setTransferNeutral();
                    actionTimer.reset();
                    pathState = STATE_DEPOSIT_CYCLE4_SECOND_DELAY;
                }
                break;

            case STATE_DEPOSIT_CYCLE4_SECOND_DELAY:
                if (actionTimer.seconds() >= SECOND_SHOT_DELAY_S) {
                    startIntake1();
                    startIntake2();
                    setTransferIn();
                    actionTimer.reset();
                    pathState = STATE_DEPOSIT_CYCLE4_SECOND_SHOT;
                }
                break;

            case STATE_DEPOSIT_CYCLE4_SECOND_SHOT:
                if (actionTimer.seconds() >= DEPOSIT_CYCLE_TIME_S) {
                    stopIntake1();
                    stopIntake2();
                    setTransferNeutral();
                    depositEnabled = false;
                    stopDeposit();
                    pathState = STATE_START_PATH11;
                }
                break;

            // ===== PATH 11: Park =====
            case STATE_START_PATH11:
                follower.followPath(paths.Path11);
                pathState = STATE_PATH11_MOVING;
                break;

            case STATE_PATH11_MOVING:
                if (!follower.isBusy()) pathState = STATE_DONE;
                break;

            case STATE_DONE:
                break;
        }
        return pathState;
    }
}

