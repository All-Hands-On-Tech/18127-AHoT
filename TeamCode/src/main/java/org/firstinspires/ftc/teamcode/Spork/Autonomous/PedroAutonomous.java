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
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "PedroAuto", group = "Autonomous")
@Configurable // Panels
public class PedroAutonomous extends OpMode {

    private TelemetryManager panelsTelemetry;
    public Follower follower;
    private int pathState;
    private Paths paths;

    private DcMotor intake1;
    private DcMotor intake2;
    private DcMotorEx depositMotorL;
    private DcMotorEx depositMotorR;
    private Servo cam;

    // ========== TUNING VARIABLES ==========
    private static final double INTAKE1_POWER = -1.0;   // Full power
    private static final double INTAKE2_POWER = 1.0;   // Full power
    private static final double DEPOSIT_CYCLE_TIME_S = 1.25;  // Run both intakes for 1.25 seconds
    private static final double FIRST_SHOT_DELAY_S = 2.5;   // Delay before first shot to prevent overshoot (increased from 1.1)
    private static final double SECOND_SHOT_DELAY_S = 0.5;  // Delay before second shot
    private static final double CAM_POSITION = 0.5124;
    private double depositTargetVelocity = 656.7;

    private static final double MIN_VELOCITY = 0.0;
    private static final double MAX_VELOCITY = 5000.0;
    private static final double STEP_SMALL = 10.0;
    private static final double STEP_LARGE = 100.0;

    private boolean prevDpadUp = false, prevDpadDown = false, prevDpadLeft = false, prevDpadRight = false;
    private long lastDpadChange = System.currentTimeMillis();
    private static final long FIRST_REPEAT_DELAY_MS = 350;
    private static final long REPEAT_INTERVAL_MS = 120;

    // ========== ALLIANCE SELECTION ==========
    public enum Alliance { NONE, BLUE, RED }
    private Alliance selectedAlliance = Alliance.NONE;

    // ========== STATE MACHINE CONSTANTS ==========
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

    private static final int STATE_DONE = 28;

    private ElapsedTime actionTimer;
    private boolean depositEnabled = true;
    private boolean lastAState = false;

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        follower = Constants.createFollower(hardwareMap);
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
                depositMotorL.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
                depositMotorL.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
                // Use same PIDF as RobotHardware/DepositTuner: (80.0, 0.4, 6.0, 12.0)
                depositMotorL.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER,
                    new PIDFCoefficients(80.0, 0.4, 6.0, 12.0));
                depositMotorL.setPower(0);
            }
        } catch (Exception e) {
            depositMotorL = null;
        }

        try {
            depositMotorR = hardwareMap.get(DcMotorEx.class, "DepositMotorR");
            if (depositMotorR != null) {
                depositMotorR.setDirection(DcMotorSimple.Direction.FORWARD);
                depositMotorR.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
                depositMotorR.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
                depositMotorR.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
                // Use same PIDF as RobotHardware/DepositTuner: (80.0, 0.4, 6.0, 12.0)
                depositMotorR.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER,
                    new PIDFCoefficients(80.0, 0.4, 6.0, 12.0));
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

        panelsTelemetry.debug("Status", "Initialized - Alliance Selection");
        panelsTelemetry.debug("Controls", "X = BLUE | B = RED");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void init_loop() {
        // Alliance selection during init
        if (gamepad1.x && selectedAlliance != Alliance.BLUE) {
            selectedAlliance = Alliance.BLUE;
            // Set blue starting pose
            follower.setStartingPose(new Pose(25.579, 129.654, Math.toRadians(143.6)));
            paths = new Paths(follower, selectedAlliance);
        } else if (gamepad1.b && selectedAlliance != Alliance.RED) {
            selectedAlliance = Alliance.RED;
            // Set red starting pose
            follower.setStartingPose(new Pose(119.504, 127.895, Math.toRadians(38)));
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
            selectedAlliance = Alliance.RED; // Default to RED if no selection
            follower.setStartingPose(new Pose(119.504, 127.895, Math.toRadians(38)));
            paths = new Paths(follower, selectedAlliance);
        }
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
        depositMotorL.setVelocity(depositTargetVelocity + 6);
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
        public PathChain Path1;
        public PathChain Path2;
        public PathChain Path3;
        public PathChain Path4;
        public PathChain Path5;
        public PathChain Path6;
        public PathChain Path7;
        public PathChain Path8;

        public Paths(Follower follower, Alliance alliance) {
            if (alliance == Alliance.BLUE) {
                // Blue alliance paths
                Path1 = follower
                        .pathBuilder()
                        .addPath(
                                new BezierLine(new Pose(25.579, 129.383), new Pose(59.955, 89.188))
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(143.6), Math.toRadians(134))
                        .build();

                Path2 = follower
                        .pathBuilder()
                        .addPath(
                                new BezierLine(new Pose(59.955, 89.188), new Pose(46.015, 84.451))
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(0))
                        .build();

                Path3 = follower
                        .pathBuilder()
                        .addPath(
                                new BezierLine(new Pose(46.015, 84.451), new Pose(19.759, 85.128))
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                        .build();

                Path4 = follower
                        .pathBuilder()
                        .addPath(
                                new BezierLine(new Pose(19.759, 85.128), new Pose(57.113, 92.165))
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(134))
                        .build();

                Path5 = follower
                        .pathBuilder()
                        .addPath(
                                new BezierLine(new Pose(57.113, 92.165), new Pose(45.068, 59.955))
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(0))
                        .build();

                Path6 = follower
                        .pathBuilder()
                        .addPath(
                                new BezierLine(new Pose(45.068, 59.955), new Pose(14.211, 59.008))
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                        .build();

                Path7 = follower
                        .pathBuilder()
                        .addPath(
                                new BezierCurve(
                                        new Pose(14.211, 59.008),
                                        new Pose(58.331, 63.609),
                                        new Pose(56.842, 93.789)
                                )
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(135))
                        .build();

                Path8 = follower
                        .pathBuilder()
                        .addPath(
                                new BezierCurve(
                                        new Pose(56.842, 93.789),
                                        new Pose(55.759, 70.647),
                                        new Pose(40.737, 71.594)
                                )
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(90))
                        .build();

            } else {
                // Red alliance paths
                Path1 = follower
                        .pathBuilder()
                        .addPath(
                                new BezierLine(new Pose(117.681, 128.707), new Pose(84.388, 88.511))
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(44))
                        .build();

                Path2 = follower
                        .pathBuilder()
                        .addPath(
                                new BezierLine(new Pose(84.388, 88.511), new Pose(93.726, 83.639))
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(180))
                        .build();

                Path3 = follower
                        .pathBuilder()
                        .addPath(
                                new BezierLine(new Pose(93.726, 83.639), new Pose(123.699, 84.045))
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                        .build();

                Path4 = follower
                        .pathBuilder()
                        .addPath(
                                new BezierLine(new Pose(123.699, 84.045), new Pose(87.501, 93.248))
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(43))
                        .build();

                Path5 = follower
                        .pathBuilder()
                        .addPath(
                                new BezierLine(new Pose(87.501, 93.248), new Pose(93.383, 60.090))
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(43), Math.toRadians(180))
                        .build();

                Path6 = follower
                        .pathBuilder()
                        .addPath(
                                new BezierLine(new Pose(93.383, 60.090), new Pose(123.970, 59.549))
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                        .build();

                Path7 = follower
                        .pathBuilder()
                        .addPath(
                                new BezierCurve(
                                        new Pose(123.970, 59.549),
                                        new Pose(79.038, 65.098),
                                        new Pose(87.636, 93.519)
                                )
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(45))
                        .build();

                Path8 = follower
                        .pathBuilder()
                        .addPath(
                                new BezierCurve(
                                        new Pose(87.636, 93.519),
                                        new Pose(92.508, 72.000),
                                        new Pose(105.699, 71.053)
                                )
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(90))
                        .build();
            }
        }
    }

    public int autonomousPathUpdate() {
        switch (pathState) {
            // ===== CYCLE 1 =====
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
                    actionTimer.reset();
                    pathState = STATE_DEPOSIT_CYCLE1_FIRST_SHOT;
                }
                break;
            case STATE_DEPOSIT_CYCLE1_FIRST_SHOT:
                if (actionTimer.seconds() >= DEPOSIT_CYCLE_TIME_S) {
                    stopIntake1();
                    stopIntake2();
                    actionTimer.reset();
                    pathState = STATE_DEPOSIT_CYCLE1_SECOND_DELAY;
                }
                break;
            case STATE_DEPOSIT_CYCLE1_SECOND_DELAY:
                if (actionTimer.seconds() >= SECOND_SHOT_DELAY_S) {
                    startIntake1();
                    startIntake2();
                    actionTimer.reset();
                    pathState = STATE_DEPOSIT_CYCLE1_SECOND_SHOT;
                }
                break;
            case STATE_DEPOSIT_CYCLE1_SECOND_SHOT:
                if (actionTimer.seconds() >= DEPOSIT_CYCLE_TIME_S) {
                    stopIntake1();
                    stopIntake2();
                    pathState = STATE_START_PATH2;
                }
                break;

            // ===== PATHS 2-4 =====
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
                    stopIntake1();
                    actionTimer.reset();
                    pathState = STATE_DEPOSIT_CYCLE2_FIRST_DELAY;
                }
                break;

            // ===== CYCLE 2 =====
            case STATE_DEPOSIT_CYCLE2_FIRST_DELAY:
                if (actionTimer.seconds() >= FIRST_SHOT_DELAY_S) {
                    startIntake1();
                    startIntake2();
                    actionTimer.reset();
                    pathState = STATE_DEPOSIT_CYCLE2_FIRST_SHOT;
                }
                break;
            case STATE_DEPOSIT_CYCLE2_FIRST_SHOT:
                if (actionTimer.seconds() >= DEPOSIT_CYCLE_TIME_S) {
                    stopIntake1();
                    stopIntake2();
                    actionTimer.reset();
                    pathState = STATE_DEPOSIT_CYCLE2_SECOND_DELAY;
                }
                break;
            case STATE_DEPOSIT_CYCLE2_SECOND_DELAY:
                if (actionTimer.seconds() >= SECOND_SHOT_DELAY_S) {
                    startIntake1();
                    startIntake2();
                    actionTimer.reset();
                    pathState = STATE_DEPOSIT_CYCLE2_SECOND_SHOT;
                }
                break;
            case STATE_DEPOSIT_CYCLE2_SECOND_SHOT:
                if (actionTimer.seconds() >= DEPOSIT_CYCLE_TIME_S) {
                    stopIntake1();
                    stopIntake2();
                    pathState = STATE_START_PATH5;
                }
                break;

            // ===== PATHS 5-7 =====
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
                    stopIntake1();
                    actionTimer.reset();
                    pathState = STATE_DEPOSIT_CYCLE3_FIRST_DELAY;
                }
                break;

            // ===== CYCLE 3 =====
            case STATE_DEPOSIT_CYCLE3_FIRST_DELAY:
                if (actionTimer.seconds() >= FIRST_SHOT_DELAY_S) {
                    startIntake1();
                    startIntake2();
                    actionTimer.reset();
                    pathState = STATE_DEPOSIT_CYCLE3_FIRST_SHOT;
                }
                break;
            case STATE_DEPOSIT_CYCLE3_FIRST_SHOT:
                if (actionTimer.seconds() >= DEPOSIT_CYCLE_TIME_S) {
                    stopIntake1();
                    stopIntake2();
                    actionTimer.reset();
                    pathState = STATE_DEPOSIT_CYCLE3_SECOND_DELAY;
                }
                break;
            case STATE_DEPOSIT_CYCLE3_SECOND_DELAY:
                if (actionTimer.seconds() >= SECOND_SHOT_DELAY_S) {
                    startIntake1();
                    startIntake2();
                    actionTimer.reset();
                    pathState = STATE_DEPOSIT_CYCLE3_SECOND_SHOT;
                }
                break;
            case STATE_DEPOSIT_CYCLE3_SECOND_SHOT:
                if (actionTimer.seconds() >= DEPOSIT_CYCLE_TIME_S) {
                    stopIntake1();
                    stopIntake2();
                    depositEnabled = false;
                    stopDeposit();
                    // Both alliances continue to Path8
                    if (paths.Path8 != null) {
                        pathState = STATE_START_PATH8;
                    } else {
                        pathState = STATE_DONE;
                    }
                }
                break;

            // ===== PATH 8 (EXIT BOX) =====
            case STATE_START_PATH8:
                follower.followPath(paths.Path8);
                pathState = STATE_PATH8_MOVING;
                break;
            case STATE_PATH8_MOVING:
                if (!follower.isBusy()) pathState = STATE_DONE;
                break;

            case STATE_DONE:
                break;
        }
        return pathState;
    }
}

