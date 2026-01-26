package org.firstinspires.ftc.teamcode.Trowel.Drive;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Trowel.Configs.RandyButterNubs;
import org.firstinspires.ftc.teamcode.Trowel.common.Odometry;
import org.firstinspires.ftc.teamcode.Trowel.common.VisionLocalization;
import org.firstinspires.ftc.teamcode.Trowel.Configs.TrowelHardware;
import org.firstinspires.ftc.teamcode.Trowel.pedroPathing.Constants;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;

@Configurable
@TeleOp(name = "Randy Butter Knubs", group = "Trowel")
public class TrowelTeleOp extends OpMode {

    private enum Team { NONE, BLUE, RED }
    private Team selectedTeam = Team.NONE;

    public static int BLUE_TAG_ID = 20;
    public static int RED_TAG_ID = 24;

    // Servo positions - automatic switching based on shooting button
    // For standard servos: 0.0 to 1.0 represents the full range of motion
    public static double SERVO_POSITION_SHOOTING = 0.5;  // Position when ZL is held (shooting)
    public static double SERVO_POSITION_IDLE = 0.225;      // Position when ZL is not held (idle)

    public static double DEPOSIT_TARGET_VELOCITY = 640.0;
    public static double DEPOSIT_TOLERANCE = 10.0;

    // Drop-detection-based boost curve parameters
    public static double BOOST_TRIGGER_THRESHOLD = 50.0;
    public static double BOOST_CURVE_MULTIPLIER = 1.30;
    public static double BOOST_CURVE_EXPONENT = 1.57;
    public static double BOOST_MIN_TICKS = 10.0;
    public static double BOOST_MAX_TICKS = 600.0;

    public static double SPEED_CURVE_EXPONENT = 2.0;

    private TrowelHardware robot;
    private RandyButterNubs drive;

    private Odometry odometry;
    private VisionLocalization visionLocalization;
    private boolean visionEnabled = false;
    private boolean odometryEnabled = false;

    private Follower follower = null;

    private static final double INTAKE2_SCALE = 0.8;

    private boolean depositActive = false;
    private boolean lastXButtonState = false;

    // Gamepad2 D-Pad tuning constants
    private static final double DEPOSIT_STEP_SMALL = 10.0;
    private static final double DEPOSIT_STEP_LARGE = 50.0;
    private static final double DEPOSIT_MIN_VELOCITY = 0.0;
    private static final double DEPOSIT_MAX_VELOCITY = 2000.0;
    private boolean prevGp2DpadUp = false, prevGp2DpadDown = false, prevGp2DpadLeft = false, prevGp2DpadRight = false;
    private long lastGp2DpadChange = 0;
    private static final long FIRST_REPEAT_DELAY_MS = 350;
    private static final long REPEAT_INTERVAL_MS = 120;

    // Auto-aim and auto-drive state
    private double aimHeadingDeg = 0.0;
    private boolean prevDriver1LB = false;
    private boolean prevDriver1ZL = false;
    private boolean hasSetScoringPosition = false;
    private Pose scoringPose = null;

    private Pose lastKnownPose = new Pose(0, 0, 0);

    @Override
    public void init() {
        robot = new TrowelHardware(hardwareMap);
        drive = new RandyButterNubs(robot.frontLeft, robot.frontRight, robot.backLeft, robot.backRight);

        try {
            follower = Constants.createFollower(hardwareMap);
        } catch (Exception ignored) {
            follower = null;
        }

        try {
            robot.initPinpoint();
            if (robot.pinpoint != null) {
                odometryEnabled = true;
                telemetry.addLine("Pinpoint Odometry Enabled");
            }
        } catch (Exception e) {
            telemetry.addLine("Pinpoint Not Found - Odometry Disabled");
            odometryEnabled = false;
        }

        try {
            visionLocalization = new VisionLocalization(hardwareMap);
            visionEnabled = true;
            telemetry.addLine("Vision Localization Enabled");
        } catch (Exception e) {
            telemetry.addLine("Vision Not Available - Vision Disabled");
            visionEnabled = false;
        }

        robot.resetDepositEncoders();

        // Initialize servo to idle position
        if (robot.transferServo != null) {
            robot.transferServo.setPosition(SERVO_POSITION_IDLE);
        }

        telemetry.addLine("Trowel TeleOp Initialized!");
        telemetry.addLine("Press X for BLUE team (Tag 20)");
        telemetry.addLine("Press A for RED team (Tag 24)");
        telemetry.update();
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
                telemetry.addData("Selected Team", "BLUE (Tag %d)", BLUE_TAG_ID);
                break;
            case RED:
                telemetry.addData("Selected Team", "RED (Tag %d)", RED_TAG_ID);
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
    public void start() {
        aimHeadingDeg = (selectedTeam == Team.RED) ? 45.0 : 135.0;
    }

    private double applyCurve(double input) {
        double sign = Math.signum(input);
        double magnitude = Math.abs(input);
        double curved = Math.pow(magnitude, SPEED_CURVE_EXPONENT);
        return sign * curved;
    }

    @Override
    public void loop() {
        // Gamepad2 D-Pad tuning for deposit target velocity
        boolean gp2Up = gamepad2.dpad_up;
        boolean gp2Down = gamepad2.dpad_down;
        boolean gp2Left = gamepad2.dpad_left;
        boolean gp2Right = gamepad2.dpad_right;
        long now = System.currentTimeMillis();

        if (gp2Up && (!prevGp2DpadUp || now - lastGp2DpadChange > FIRST_REPEAT_DELAY_MS)) {
            DEPOSIT_TARGET_VELOCITY = Math.min(DEPOSIT_MAX_VELOCITY, DEPOSIT_TARGET_VELOCITY + DEPOSIT_STEP_LARGE);
            lastGp2DpadChange = now;
        }
        if (gp2Down && (!prevGp2DpadDown || now - lastGp2DpadChange > FIRST_REPEAT_DELAY_MS)) {
            DEPOSIT_TARGET_VELOCITY = Math.max(DEPOSIT_MIN_VELOCITY, DEPOSIT_TARGET_VELOCITY - DEPOSIT_STEP_LARGE);
            lastGp2DpadChange = now;
        }
        if (gp2Right && (!prevGp2DpadRight || now - lastGp2DpadChange > FIRST_REPEAT_DELAY_MS)) {
            DEPOSIT_TARGET_VELOCITY = Math.min(DEPOSIT_MAX_VELOCITY, DEPOSIT_TARGET_VELOCITY + DEPOSIT_STEP_SMALL);
            lastGp2DpadChange = now;
        }
        if (gp2Left && (!prevGp2DpadLeft || now - lastGp2DpadChange > FIRST_REPEAT_DELAY_MS)) {
            DEPOSIT_TARGET_VELOCITY = Math.max(DEPOSIT_MIN_VELOCITY, DEPOSIT_TARGET_VELOCITY - DEPOSIT_STEP_SMALL);
            lastGp2DpadChange = now;
        }

        if ((gp2Up || gp2Down || gp2Left || gp2Right) && now - lastGp2DpadChange > REPEAT_INTERVAL_MS) {
            lastGp2DpadChange = now - (REPEAT_INTERVAL_MS + 1);
        }

        prevGp2DpadUp = gp2Up;
        prevGp2DpadDown = gp2Down;
        prevGp2DpadLeft = gp2Left;
        prevGp2DpadRight = gp2Right;

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

        if (visionEnabled && visionLocalization != null) {
            visionLocalization.update();
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

        int targetTagId = (selectedTeam == Team.BLUE) ? BLUE_TAG_ID :
                (selectedTeam == Team.RED) ? RED_TAG_ID : -1;

        double forward = applyCurve(-gamepad1.left_stick_y);
        double strafe = applyCurve(gamepad1.left_stick_x);
        double rotate = applyCurve(gamepad1.right_stick_x);
        rotate *= 0.5;

        // ZL (left trigger) - Set scoring position, trigger shooting, run intakes, AND control servo
        boolean driver1ZL = gamepad1.left_trigger > 0.5;
        boolean driver1ZLPressed = driver1ZL && !prevDriver1ZL;

        if (driver1ZLPressed) {
            // Set/update scoring position
            if (odometryEnabled && odometry != null) {
                Odometry.Position pos = odometry.getPosition();
                double headingRad = pos.headingRad;
                scoringPose = new com.pedropathing.geometry.Pose(pos.xMm / 25.4, pos.yMm / 25.4, headingRad);
                aimHeadingDeg = Math.toDegrees(headingRad);
                hasSetScoringPosition = true;
            } else if (follower != null) {
                follower.update();
                Pose pose = follower.getPose();
                scoringPose = new com.pedropathing.geometry.Pose(pose.getX(), pose.getY(), pose.getHeading());
                aimHeadingDeg = Math.toDegrees(pose.getHeading());
                hasSetScoringPosition = true;
            }
        }

        // AUTOMATIC SERVO CONTROL: Switch position based on ZL button state
        if (driver1ZL) {
            // Shooting mode - move servo to shooting position and run intakes
            if (robot.transferServo != null) {
                robot.transferServo.setPosition(SERVO_POSITION_SHOOTING);
            }

            if (robot.intake1 != null) robot.intake1.setPower(1.0);
            if (robot.intake2 != null) robot.intake2.setPower(-INTAKE2_SCALE);
        } else {
            // Idle mode - move servo back to idle position
            if (robot.transferServo != null) {
                robot.transferServo.setPosition(SERVO_POSITION_IDLE);
            }
        }

        prevDriver1ZL = driver1ZL;

        // Manual input override for follower
        boolean hasManualInput = Math.abs(forward) > 0.05 || Math.abs(strafe) > 0.05 || Math.abs(rotate) > 0.05;
        if (follower != null && hasManualInput) {
            follower.breakFollowing();
            follower = null;
        }

        // LB (left bumper) - Auto-drive to scoring position
        boolean driver1LB = gamepad1.left_bumper;
        boolean driver1LBPressed = driver1LB && !prevDriver1LB;
        if (driver1LBPressed && hasSetScoringPosition && scoringPose != null) {
            Pose startPose;
            try {
                if (odometryEnabled && odometry != null) {
                    Odometry.Position pos = odometry.getPosition();
                    startPose = new Pose(pos.xMm / 25.4, pos.yMm / 25.4, pos.headingRad);
                    telemetry.addLine("Using odometry for start pose");
                } else if (follower != null) {
                    follower.update();
                    startPose = follower.getPose();
                    telemetry.addLine("Using existing follower pose for start pose");
                } else if (lastKnownPose != null) {
                    startPose = lastKnownPose;
                    telemetry.addLine("Using cached lastKnownPose for start pose");
                } else {
                    if (selectedTeam == Team.BLUE) {
                        startPose = new Pose(26.467, 129.584, Math.toRadians(145));
                        telemetry.addLine("Fallback: BLUE team start pose");
                    } else {
                        startPose = new Pose(117.533, 129.584, Math.toRadians(35));
                        telemetry.addLine("Fallback: RED team start pose");
                    }
                }

                if (follower == null) {
                    follower = Constants.createFollower(hardwareMap);
                }
                follower.setMaxPower(1.0);
                follower.setStartingPose(startPose);
                telemetry.addData("FOLLOWER START", "(%.1f, %.1f) @ %.1f°", startPose.getX(), startPose.getY(), Math.toDegrees(startPose.getHeading()));

                try {
                    if (scoringPose != null) {
                        com.pedropathing.paths.PathChain pathToScoring = follower.pathBuilder()
                                .addPath(new com.pedropathing.geometry.BezierLine(startPose, scoringPose))
                                .setLinearHeadingInterpolation(startPose.getHeading(), scoringPose.getHeading())
                                .build();
                        follower.followPath(pathToScoring);
                        telemetry.addLine("FOLLOWER: started path to scoring position");
                    } else {
                        telemetry.addLine("FOLLOWER: no scoringPose recorded");
                    }
                } catch (Exception e) {
                    telemetry.addLine("FOLLOWER PATH ERROR: " + e.getMessage());
                }

            } catch (Exception e) {
                telemetry.addLine("FOLLOWER INIT ERROR: " + e.getMessage());
            }
        }
        prevDriver1LB = driver1LB;

        // Drive control
        if (follower != null) {
            try {
                follower.update();
                if (!follower.isBusy()) {
                    follower.setTeleOpDrive(forward, -strafe, -rotate, true);
                }
            } catch (Exception e) {
                telemetry.addLine("FOLLOWER ERROR - using basic drive");
                drive.drive(forward, strafe, rotate, false, gamepad1.right_bumper);
            }
        } else {
            drive.drive(forward, strafe, rotate, false, gamepad1.right_bumper);
        }

        // Intake control when ZL not held - gamepad2 has priority
        if (!driver1ZL) {
            if (robot.intake1 != null) {
                if (gamepad2.left_trigger > 0.1) {
                    robot.intake1.setPower(1.0);
                } else if (gamepad2.right_trigger > 0.1) {
                    robot.intake1.setPower(-1.0);
                } else {
                    robot.intake1.setPower(0.0);
                }
            }

            if (robot.intake2 != null) {
                if (gamepad2.a) {
                    robot.intake2.setPower(-INTAKE2_SCALE);
                } else if (gamepad2.b) {
                    robot.intake2.setPower(INTAKE2_SCALE);
                } else {
                    robot.intake2.setPower(0.0);
                }
            }
        }

        // Toggle deposit active state on gamepad2 X button press
        if (gamepad2.x && !lastXButtonState) {
            depositActive = !depositActive;
            if (!depositActive) {
                robot.stopDeposit();
            }
        }
        lastXButtonState = gamepad2.x;

        // Deposit velocity control with drop-based boost curve
        if (depositActive && robot.deposit1 != null && robot.deposit2 != null) {
            double vel1 = robot.getDeposit1Velocity();
            double vel2 = robot.getDeposit2Velocity();
            double avgVel = (vel1 + vel2) / 2.0;

            double targetVelocity = DEPOSIT_TARGET_VELOCITY;

            double drop = Math.max(0.0, targetVelocity - avgVel);

            double appliedBoost = 0.0;
            if (drop >= BOOST_TRIGGER_THRESHOLD) {
                appliedBoost = BOOST_CURVE_MULTIPLIER * Math.pow(drop, BOOST_CURVE_EXPONENT);
                if (appliedBoost < BOOST_MIN_TICKS) appliedBoost = BOOST_MIN_TICKS;
                if (appliedBoost > BOOST_MAX_TICKS) appliedBoost = BOOST_MAX_TICKS;
            }

            double commandedVelocity = targetVelocity + appliedBoost;

            robot.setDepositVelocity(commandedVelocity);

            telemetry.addData("Deposit Drop", "%.1f ticks/s", drop);
            telemetry.addData("Applied Boost", "%.1f ticks/s", appliedBoost);
        }

        // === TELEMETRY ===
        String teamStr = (selectedTeam == Team.BLUE) ? "BLUE" : (selectedTeam == Team.RED) ? "RED" : "NONE";
        telemetry.addLine("═══════════════════════════════");
        telemetry.addData("TEAM", "%s (Tag: %d)", teamStr, targetTagId);
        telemetry.addData("Aim Heading", "%.1f°", aimHeadingDeg);

        if (!hasSetScoringPosition) {
            telemetry.addLine("⚠ Press ZL at scoring spot to record");
        } else if (scoringPose != null) {
            telemetry.addData("Scoring Position", "✓ (%.1f, %.1f) @ %.1f°",
                    scoringPose.getX(), scoringPose.getY(), Math.toDegrees(scoringPose.getHeading()));
        }

        if (follower != null && follower.isBusy() && scoringPose != null) {
            telemetry.addData("Auto-Drive", "✓ ACTIVE (move stick to cancel)");
            Pose currentPose = follower.getPose();
            double distToTarget = Math.sqrt(
                    Math.pow(scoringPose.getX() - currentPose.getX(), 2) +
                            Math.pow(scoringPose.getY() - currentPose.getY(), 2)
            );
            telemetry.addData("Distance to Target", "%.1f in", distToTarget);
        }
        telemetry.addLine("");

        // === SERVO STATUS ===
        telemetry.addLine("─── SERVO (AUTO) ───");
        telemetry.addData("Mode", "AUTOMATIC");
        telemetry.addData("Current State", driver1ZL ? "SHOOTING" : "IDLE");
        if (robot.transferServo != null) {
            telemetry.addData("Servo Position", "%.3f", robot.transferServo.getPosition());
        }
        telemetry.addData("Shooting Pos", "%.3f", SERVO_POSITION_SHOOTING);
        telemetry.addData("Idle Pos", "%.3f", SERVO_POSITION_IDLE);
        telemetry.addLine("");

        // === DEPOSIT STATUS ===
        telemetry.addLine("─── DEPOSIT ───");
        telemetry.addData("Status", depositActive ? "✓ ACTIVE" : "✗ STOPPED");

        if (robot.deposit1 != null && robot.deposit2 != null && depositActive) {
            double vel1 = robot.getDeposit1Velocity();
            double vel2 = robot.getDeposit2Velocity();
            double avgVel = (vel1 + vel2) / 2.0;

            double targetVel = DEPOSIT_TARGET_VELOCITY;

            double error = targetVel - avgVel;
            boolean atTarget = Math.abs(error) < DEPOSIT_TOLERANCE;

            telemetry.addData("Target Velocity", "%.0f ticks/s", targetVel);
            telemetry.addData("Current Velocity", "%.0f / %.0f (Avg: %.0f)", vel1, vel2, avgVel);
            telemetry.addData("Error", "%s%.0f ticks/s", atTarget ? "✓ " : "", error);
            telemetry.addData("Power", "%.2f / %.2f", robot.deposit1.getPower(), robot.deposit2.getPower());

            double drop = Math.max(0.0, targetVel - avgVel);
            double displayedBoost = 0.0;
            if (drop >= BOOST_TRIGGER_THRESHOLD) {
                displayedBoost = BOOST_CURVE_MULTIPLIER * Math.pow(drop, BOOST_CURVE_EXPONENT);
                if (displayedBoost < BOOST_MIN_TICKS) displayedBoost = BOOST_MIN_TICKS;
                if (displayedBoost > BOOST_MAX_TICKS) displayedBoost = BOOST_MAX_TICKS;
            }
            telemetry.addData("Drop", "%.1f", drop);
            telemetry.addData("BoostApplied", "%.1f ticks/s", displayedBoost);

        } else if (depositActive) {
            telemetry.addData("Target Velocity", "%.0f ticks/s", DEPOSIT_TARGET_VELOCITY);
        }
        telemetry.addLine("");

        // === DRIVE DEBUG ===
        telemetry.addLine("─── DRIVE DEBUG ───");
        telemetry.addData("Raw Sticks", "LY:%.2f LX:%.2f RX:%.2f",
                -gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
        telemetry.addData("Curved Input", "F:%.2f S:%.2f R:%.2f", forward, strafe, rotate);
        telemetry.addData("Has Manual Input", hasManualInput ? "YES" : "NO");
        telemetry.addData("Follower Exists", follower != null ? "YES" : "NO");
        if (follower != null) {
            telemetry.addData("Following Path", follower.isBusy() ? "YES ⚠" : "NO ✓");
        }
        try {
            telemetry.addData("LastKnownPose", "(%.1f, %.1f) @ %.1f°", lastKnownPose.getX(), lastKnownPose.getY(), Math.toDegrees(lastKnownPose.getHeading()));
        } catch (Exception ignored) {}
        telemetry.addLine("");

        // === INTAKES ===
        telemetry.addLine("─── INTAKES ───");
        if (robot.intake1 != null) {
            telemetry.addData("Intake 1", "%.2f", robot.intake1.getPower());
        }
        if (robot.intake2 != null) {
            telemetry.addData("Intake 2 (FLIPPED)", "%.2f", robot.intake2.getPower());
        }
        telemetry.addLine("");

        // === ODOMETRY ===
        if (odometryEnabled && odometry != null) {
            Odometry.Position pos = odometry.getPosition();
            telemetry.addLine("─── POSITION ───");
            telemetry.addData("X, Y", "%.1f, %.1f in", pos.xMm / 25.4, pos.yMm / 25.4);
            telemetry.addData("Heading", "%.1f°", Math.toDegrees(pos.headingRad));
            telemetry.addLine("");
        }

        // === TUNING INFO ===
        telemetry.addLine("─── TUNING ───");
        try {
            telemetry.addData("Kp, Ki, Kd, Kf", "%.3f, %.3f, %.3f, %.3f",
                    TrowelHardware.DEPOSIT_PIDF.p,
                    TrowelHardware.DEPOSIT_PIDF.i,
                    TrowelHardware.DEPOSIT_PIDF.d,
                    TrowelHardware.DEPOSIT_PIDF.f);
        } catch (Exception e) {
            telemetry.addData("Kp, Ki, Kd, Kf", "(from hardware)");
        }
        telemetry.addData("Speed Curve Exp", "%.2f", SPEED_CURVE_EXPONENT);
        telemetry.addLine("ZL: set pos/shoot/intakes/servo | LB: auto-drive");
        telemetry.addLine("═══════════════════════════════");

        telemetry.update();
    }

    @Override
    public void stop() {
        drive.stop();
        robot.stop();
        if (follower != null) {
            follower.breakFollowing();
        }
    }
}