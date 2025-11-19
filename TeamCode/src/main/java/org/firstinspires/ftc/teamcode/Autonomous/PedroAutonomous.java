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

@Autonomous(name = "PedroAuto", group = "Autonomous")
@Configurable // Panels
public class PedroAutonomous extends OpMode {

    // Path selection enum
    private enum PathChoice { NONE, RED_TOP, BLUE_TOP }

    private TelemetryManager panelsTelemetry;
    public Follower follower;
    private int pathState;
    private Paths paths;
    private PathChoice selectedPath = PathChoice.NONE;

    private DcMotor intake1;
    private DcMotor intake2;
    private DcMotorEx depositMotorL;
    private DcMotorEx depositMotorR;
    private Servo cam;

    // ========== TUNING VARIABLES ==========
    private static final double INTAKE1_POWER = -1.0;
    private static final double INTAKE2_POWER = 1.0;
    private static final double DEPOSIT_CYCLE_PAUSE_INTAKE1_S = 1.8;  // Reduced from 2.5
    private static final double DEPOSIT_CYCLE_INTAKE2_RUN_S = 1.4;   // Reduced from 2.0
    private static final double DEPOSIT_CYCLE_INTAKE1_RUN_S = 1.0;   // Reduced from 1.5
    private static final double DEPOSIT_CYCLE_PAUSE_S = 1.0;         // Reduced from 1.5
    private static final double DEPOSIT_CYCLE_INTAKE2_RUN2_S = 1.4;  // Reduced from 2.0
    private static final double CAM_POSITION = 0.5124;
    private double depositTargetVelocity = 645.7;

    private static final double MIN_VELOCITY = 0.0;
    private static final double MAX_VELOCITY = 5000.0;
    private static final double STEP_SMALL = 10.0;
    private static final double STEP_LARGE = 100.0;

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
    private static final int STATE_START_PATH6 = 20;
    private static final int STATE_PATH6_MOVING = 21;
    private static final int STATE_START_PATH7 = 22;
    private static final int STATE_PATH7_MOVING = 23;

    private static final int STATE_DEPOSIT_CYCLE3_PAUSE = 24;
    private static final int STATE_DEPOSIT_CYCLE3_INTAKE2_RUN1 = 25;
    private static final int STATE_DEPOSIT_CYCLE3_INTAKE1_RUN = 26;
    private static final int STATE_DEPOSIT_CYCLE3_PAUSE2 = 27;
    private static final int STATE_DEPOSIT_CYCLE3_INTAKE2_RUN2 = 28;

    private static final int STATE_START_PATH8 = 29;
    private static final int STATE_PATH8_MOVING = 30;
    private static final int STATE_START_PATH9 = 31;
    private static final int STATE_PATH9_MOVING = 32;
    private static final int STATE_START_PATH10 = 33;
    private static final int STATE_PATH10_MOVING = 34;

    private static final int STATE_DEPOSIT_CYCLE4_PAUSE = 35;
    private static final int STATE_DEPOSIT_CYCLE4_INTAKE2_RUN1 = 36;
    private static final int STATE_DEPOSIT_CYCLE4_INTAKE1_RUN = 37;
    private static final int STATE_DEPOSIT_CYCLE4_PAUSE2 = 38;
    private static final int STATE_DEPOSIT_CYCLE4_INTAKE2_RUN2 = 39;

    private static final int STATE_START_PATH11 = 40;
    private static final int STATE_PATH11_MOVING = 41;
    private static final int STATE_DONE = -1;

    private ElapsedTime actionTimer;
    private boolean depositEnabled = true;
    private boolean lastAState = false;

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        // Path selection during init
        ElapsedTime initTimer = new ElapsedTime();

        telemetry.addLine("=== PATH SELECTION ===");
        telemetry.addLine("Gamepad1 Dpad Right: RED TOP");
        telemetry.addLine("Gamepad1 Dpad Left: BLUE TOP");
        telemetry.update();

        while (initTimer.seconds() < 25) {
            if (gamepad1.dpad_right) {
                selectedPath = PathChoice.RED_TOP;
            } else if (gamepad1.dpad_left) {
                selectedPath = PathChoice.BLUE_TOP;
            }

            String pathName = "NONE";
            if (selectedPath == PathChoice.RED_TOP) {
                pathName = "RED TOP";
            } else if (selectedPath == PathChoice.BLUE_TOP) {
                pathName = "BLUE TOP";
            }

            telemetry.addLine("=== PATH SELECTION ===");
            telemetry.addLine("Gamepad1 Dpad Right: RED TOP");
            telemetry.addLine("Gamepad1 Dpad Left: BLUE TOP");
            telemetry.addLine();
            telemetry.addData("Selected Path", pathName);
            telemetry.update();
        }

        follower = Constants.createFollower(hardwareMap);

        // Set starting pose based on path selection
        if (selectedPath == PathChoice.RED_TOP) {
            follower.setStartingPose(new Pose(120.316, 128.707, Math.toRadians(38)));
        } else if (selectedPath == PathChoice.BLUE_TOP) {
            follower.setStartingPose(new Pose(24.632, 129.383, Math.toRadians(142)));
        }

        paths = new Paths(follower);
        actionTimer = new ElapsedTime();

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

        try {
            depositMotorL = hardwareMap.get(DcMotorEx.class, "DepositMotorL");
            if (depositMotorL != null) {
                depositMotorL.setDirection(DcMotorSimple.Direction.REVERSE);
                depositMotorL.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
                depositMotorL.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
                depositMotorL.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
                // Significantly reduced P gain (7.5 -> 3.0) + increased I gain to prevent overshoot
                // P=3.0 reduces aggressive acceleration, I=1.5 helps reach target smoothly
                depositMotorL.setVelocityPIDFCoefficients(3.0, 1.5, 1.0, 11.0);
                depositMotorL.setPower(0);
            }
        } catch (Exception e) {
            depositMotorL = null;
        }

        try {
            depositMotorR = hardwareMap.get(DcMotorEx.class, "DepositMotorR");
            if (depositMotorR != null) {
                depositMotorR.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
                depositMotorR.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
                depositMotorR.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
                // Significantly reduced P gain (7.5 -> 3.0) + increased I gain to prevent overshoot
                // P=3.0 reduces aggressive acceleration, I=1.5 helps reach target smoothly
                depositMotorR.setVelocityPIDFCoefficients(3.0, 1.5, 1.0, 11.0);
                depositMotorR.setPower(0);
            }
        } catch (Exception e) {
            depositMotorR = null;
        }

        try {
            cam = hardwareMap.get(Servo.class, "Cam");
            if (cam != null) {
                cam.setPosition(CAM_POSITION);
            }
        } catch (Exception e) {
            cam = null;
        }

        panelsTelemetry.debug("Status", "Initialized - Red Top 8-Ball");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void loop() {
        follower.update();
        pathState = autonomousPathUpdate();

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

        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", follower.getPose().getHeading());
        panelsTelemetry.debug("Deposit Target", String.format(java.util.Locale.US, "%.0f ticks/s", depositTargetVelocity));

        if (depositMotorL != null) {
            double vL = depositMotorL.getVelocity();
            double rpmL = (vL / 28.0) * 60.0;
            panelsTelemetry.debug("DepositL", String.format(java.util.Locale.US, "%.0f RPM", rpmL));
        }
        if (depositMotorR != null) {
            double vR = depositMotorR.getVelocity();
            double rpmR = (vR / 28.0) * 60.0;
            panelsTelemetry.debug("DepositR", String.format(java.util.Locale.US, "%.0f RPM", rpmR));
        }

        if (intake1 != null) panelsTelemetry.debug("Intake1", intake1.getPower());
        if (intake2 != null) panelsTelemetry.debug("Intake2", intake2.getPower());
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void stop() {
        if (intake1 != null) intake1.setPower(0);
        if (intake2 != null) intake2.setPower(0);
        if (depositMotorL != null) depositMotorL.setPower(0);
        if (depositMotorR != null) depositMotorR.setPower(0);
    }

    private void runDepositAtVelocity() {
        if (depositMotorL == null || depositMotorR == null) return;
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

    public class Paths {
        public PathChain Path1, Path2, Path3, Path4, Path5, Path6, Path7, Path8, Path9, Path10, Path11;

        public Paths(Follower follower) {
            if (selectedPath == PathChoice.RED_TOP) {
                buildRedTopPaths(follower);
            } else if (selectedPath == PathChoice.BLUE_TOP) {
                buildBlueTopPaths(follower);
            }
        }

        private void buildRedTopPaths(Follower follower) {
            Path1 = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(120.316, 128.707), new Pose(82.286, 86.617)))
                    .setLinearHeadingInterpolation(Math.toRadians(38), Math.toRadians(45))
                    .build();

            Path2 = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(82.286, 86.617), new Pose(93.925, 84.451)))
                    .setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(180))
                    .build();

            Path3 = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(93.925, 84.451), new Pose(128.842, 83.910)))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            Path4 = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(128.842, 83.910), new Pose(82.421, 86.887)))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(45))
                    .build();

            Path5 = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(82.421, 86.887), new Pose(93.383, 60.090)))
                    .setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(180))
                    .build();

            Path6 = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(93.383, 60.090), new Pose(135.880, 59.278)))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            Path7 = follower.pathBuilder()
                    .addPath(new BezierCurve(new Pose(135.880, 59.278), new Pose(79.038, 65.098), new Pose(82.421, 86.887)))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(45))
                    .build();

            Path8 = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(82.421, 86.887), new Pose(93.383, 35.600)))
                    .setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(180))
                    .build();

            Path9 = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(93.383, 35.600), new Pose(135.474, 35.188)))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            Path10 = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(135.474, 35.188), new Pose(82.421, 86.887)))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(45))
                    .build();

            Path11 = follower.pathBuilder()
                    .addPath(new BezierCurve(new Pose(82.421, 86.887), new Pose(92.301, 71.188), new Pose(122.481, 70.917)))
                    .setTangentHeadingInterpolation()
                    .build();
        }

        private void buildBlueTopPaths(Follower follower) {
            Path1 = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(24.632, 129.383), new Pose(59.955, 91.895)))
                    .setLinearHeadingInterpolation(Math.toRadians(142), Math.toRadians(135))
                    .build();

            Path2 = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(59.955, 91.895), new Pose(46.015, 84.451)))
                    .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(0))
                    .build();

            Path3 = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(46.015, 84.451), new Pose(14.752, 84.316)))
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            Path4 = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(14.752, 84.316), new Pose(59.955, 91.895)))
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(135))
                    .build();

            Path5 = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(59.955, 91.895), new Pose(45.609, 60.767)))
                    .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(0))
                    .build();

            Path6 = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(45.609, 60.767), new Pose(10.556, 60.767)))
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            Path7 = follower.pathBuilder()
                    .addPath(new BezierCurve(new Pose(10.556, 60.767), new Pose(58.331, 63.609), new Pose(59.955, 91.895)))
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(135))
                    .build();

            Path8 = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(59.955, 91.895), new Pose(44.526, 36.271)))
                    .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(0))
                    .build();

            Path9 = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(44.526, 36.271), new Pose(7.985, 36.541)))
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            Path10 = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(7.985, 36.541), new Pose(59.955, 91.895)))
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(135))
                    .build();

            Path11 = follower.pathBuilder()
                    .addPath(new BezierCurve(new Pose(59.955, 91.895), new Pose(50.0, 70.0), new Pose(30.0, 70.0)))
                    .setTangentHeadingInterpolation()
                    .build();
        }
    }

    public int autonomousPathUpdate() {
        switch (pathState) {
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
                    pathState = STATE_DEPOSIT_CYCLE1_PAUSE;
                }
                break;

            case STATE_DEPOSIT_CYCLE1_PAUSE:
                if (actionTimer.seconds() >= DEPOSIT_CYCLE_PAUSE_INTAKE1_S) {
                    startIntake2();
                    actionTimer.reset();
                    pathState = STATE_DEPOSIT_CYCLE1_INTAKE2_RUN1;
                }
                break;
            case STATE_DEPOSIT_CYCLE1_INTAKE2_RUN1:
                if (actionTimer.seconds() >= DEPOSIT_CYCLE_INTAKE2_RUN_S) {
                    stopIntake2();
                    startIntake1();
                    actionTimer.reset();
                    pathState = STATE_DEPOSIT_CYCLE1_INTAKE1_RUN;
                }
                break;
            case STATE_DEPOSIT_CYCLE1_INTAKE1_RUN:
                if (actionTimer.seconds() >= DEPOSIT_CYCLE_INTAKE1_RUN_S) {
                    stopIntake1();
                    actionTimer.reset();
                    pathState = STATE_DEPOSIT_CYCLE1_PAUSE2;
                }
                break;
            case STATE_DEPOSIT_CYCLE1_PAUSE2:
                if (actionTimer.seconds() >= DEPOSIT_CYCLE_PAUSE_S) {
                    startIntake2();
                    actionTimer.reset();
                    pathState = STATE_DEPOSIT_CYCLE1_INTAKE2_RUN2;
                }
                break;
            case STATE_DEPOSIT_CYCLE1_INTAKE2_RUN2:
                if (actionTimer.seconds() >= DEPOSIT_CYCLE_INTAKE2_RUN2_S) {
                    stopIntake2();
                    pathState = STATE_START_PATH2;
                }
                break;

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
                pathState = STATE_PATH4_MOVING;
                break;
            case STATE_PATH4_MOVING:
                if (!follower.isBusy()) {
                    stopIntake1();
                    actionTimer.reset();
                    pathState = STATE_DEPOSIT_CYCLE2_PAUSE;
                }
                break;

            case STATE_DEPOSIT_CYCLE2_PAUSE:
                if (actionTimer.seconds() >= DEPOSIT_CYCLE_PAUSE_INTAKE1_S) {
                    startIntake2();
                    actionTimer.reset();
                    pathState = STATE_DEPOSIT_CYCLE2_INTAKE2_RUN1;
                }
                break;
            case STATE_DEPOSIT_CYCLE2_INTAKE2_RUN1:
                if (actionTimer.seconds() >= DEPOSIT_CYCLE_INTAKE2_RUN_S) {
                    stopIntake2();
                    startIntake1();
                    actionTimer.reset();
                    pathState = STATE_DEPOSIT_CYCLE2_INTAKE1_RUN;
                }
                break;
            case STATE_DEPOSIT_CYCLE2_INTAKE1_RUN:
                if (actionTimer.seconds() >= DEPOSIT_CYCLE_INTAKE1_RUN_S) {
                    stopIntake1();
                    actionTimer.reset();
                    pathState = STATE_DEPOSIT_CYCLE2_PAUSE2;
                }
                break;
            case STATE_DEPOSIT_CYCLE2_PAUSE2:
                if (actionTimer.seconds() >= DEPOSIT_CYCLE_PAUSE_S) {
                    startIntake2();
                    actionTimer.reset();
                    pathState = STATE_DEPOSIT_CYCLE2_INTAKE2_RUN2;
                }
                break;
            case STATE_DEPOSIT_CYCLE2_INTAKE2_RUN2:
                if (actionTimer.seconds() >= DEPOSIT_CYCLE_INTAKE2_RUN2_S) {
                    stopIntake2();
                    pathState = STATE_START_PATH5;
                }
                break;

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
                pathState = STATE_PATH7_MOVING;
                break;
            case STATE_PATH7_MOVING:
                if (!follower.isBusy()) {
                    stopIntake1();
                    actionTimer.reset();
                    pathState = STATE_DEPOSIT_CYCLE3_PAUSE;
                }
                break;

            case STATE_DEPOSIT_CYCLE3_PAUSE:
                if (actionTimer.seconds() >= DEPOSIT_CYCLE_PAUSE_INTAKE1_S) {
                    startIntake2();
                    actionTimer.reset();
                    pathState = STATE_DEPOSIT_CYCLE3_INTAKE2_RUN1;
                }
                break;
            case STATE_DEPOSIT_CYCLE3_INTAKE2_RUN1:
                if (actionTimer.seconds() >= DEPOSIT_CYCLE_INTAKE2_RUN_S) {
                    stopIntake2();
                    startIntake1();
                    actionTimer.reset();
                    pathState = STATE_DEPOSIT_CYCLE3_INTAKE1_RUN;
                }
                break;
            case STATE_DEPOSIT_CYCLE3_INTAKE1_RUN:
                if (actionTimer.seconds() >= DEPOSIT_CYCLE_INTAKE1_RUN_S) {
                    stopIntake1();
                    actionTimer.reset();
                    pathState = STATE_DEPOSIT_CYCLE3_PAUSE2;
                }
                break;
            case STATE_DEPOSIT_CYCLE3_PAUSE2:
                if (actionTimer.seconds() >= DEPOSIT_CYCLE_PAUSE_S) {
                    startIntake2();
                    actionTimer.reset();
                    pathState = STATE_DEPOSIT_CYCLE3_INTAKE2_RUN2;
                }
                break;
            case STATE_DEPOSIT_CYCLE3_INTAKE2_RUN2:
                if (actionTimer.seconds() >= DEPOSIT_CYCLE_INTAKE2_RUN2_S) {
                    stopIntake2();
                    pathState = STATE_START_PATH8;
                }
                break;

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
                pathState = STATE_PATH10_MOVING;
                break;
            case STATE_PATH10_MOVING:
                if (!follower.isBusy()) {
                    stopIntake1();
                    actionTimer.reset();
                    pathState = STATE_DEPOSIT_CYCLE4_PAUSE;
                }
                break;

            case STATE_DEPOSIT_CYCLE4_PAUSE:
                if (actionTimer.seconds() >= DEPOSIT_CYCLE_PAUSE_INTAKE1_S) {
                    startIntake2();
                    actionTimer.reset();
                    pathState = STATE_DEPOSIT_CYCLE4_INTAKE2_RUN1;
                }
                break;
            case STATE_DEPOSIT_CYCLE4_INTAKE2_RUN1:
                if (actionTimer.seconds() >= DEPOSIT_CYCLE_INTAKE2_RUN_S) {
                    stopIntake2();
                    startIntake1();
                    actionTimer.reset();
                    pathState = STATE_DEPOSIT_CYCLE4_INTAKE1_RUN;
                }
                break;
            case STATE_DEPOSIT_CYCLE4_INTAKE1_RUN:
                if (actionTimer.seconds() >= DEPOSIT_CYCLE_INTAKE1_RUN_S) {
                    stopIntake1();
                    actionTimer.reset();
                    pathState = STATE_DEPOSIT_CYCLE4_PAUSE2;
                }
                break;
            case STATE_DEPOSIT_CYCLE4_PAUSE2:
                if (actionTimer.seconds() >= DEPOSIT_CYCLE_PAUSE_S) {
                    startIntake2();
                    actionTimer.reset();
                    pathState = STATE_DEPOSIT_CYCLE4_INTAKE2_RUN2;
                }
                break;
            case STATE_DEPOSIT_CYCLE4_INTAKE2_RUN2:
                if (actionTimer.seconds() >= DEPOSIT_CYCLE_INTAKE2_RUN2_S) {
                    stopIntake2();
                    pathState = STATE_START_PATH11;
                }
                break;

            case STATE_START_PATH11:
                follower.followPath(paths.Path11);
                pathState = STATE_PATH11_MOVING;
                break;
            case STATE_PATH11_MOVING:
                if (!follower.isBusy()) {
                    depositEnabled = false;
                    stopDeposit();
                    pathState = STATE_DONE;
                }
                break;

            case STATE_DONE:
                break;
        }
        return pathState;
    }
}

