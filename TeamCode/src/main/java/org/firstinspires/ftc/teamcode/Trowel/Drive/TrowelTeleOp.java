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

    // PIDF constants for deposit velocity control
    public static double DEPOSIT_KP = 0.01;
    public static double DEPOSIT_KI = 0.0;
    public static double DEPOSIT_KD = 0.0;
    public static double DEPOSIT_KF = 0.0; // Feedforward constant
    public static double DEPOSIT_TARGET_VELOCITY = 600.0; // target velocity in ticks/sec
    public static double DEPOSIT_POWER_MIN = -1.0;
    public static double DEPOSIT_POWER_MAX = 1.0;
    public static double DEPOSIT_TOLERANCE = 10.0; // ticks/sec within target to consider "at velocity"

    // Feedforward boost system
    public static double BOOST_VELOCITY_BALL_1 = 200.0; // Extra velocity for first ball
    public static double BOOST_VELOCITY_BALL_2 = 250.0; // Extra velocity for second ball
    public static double BOOST_VELOCITY_BALL_3 = 200.0; // Extra velocity for third ball
    public static double BOOST_DURATION_MS = 200.0; // How long each boost lasts

    private boolean isBoosting = false;
    private long boostStartTime = 0;
    private int currentBallNumber = 0; // Tracks which ball we're shooting (0-2)

    // Speed curve tuning
    public static double SPEED_CURVE_EXPONENT = 2.0; // Less aggressive curve for better full-speed response

    // PIDF state variables
    private double depositIntegral = 0.0;
    private double depositLastError = 0.0;
    private long depositLastTime = 0;

    private TrowelHardware robot;
    private RandyButterNubs drive;

    private Odometry odometry;
    private VisionLocalization visionLocalization;
    private boolean visionEnabled = false;
    private boolean odometryEnabled = false;

    // Pedro Pathing follower instance
    private Follower follower = null;

    // Scale factor for the second-stage intake
    private static final double INTAKE2_SCALE = 1.0;

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
    private Pose scoringPose = null; // Store the scoring pose (x, y, heading)

    // Add a cached pose field at the class level
    private Pose lastKnownPose = new Pose(0, 0, 0);

    @Override
    public void init() {
        robot = new TrowelHardware(hardwareMap);
        drive = new RandyButterNubs(robot.frontLeft, robot.frontRight, robot.backLeft, robot.backRight);

        // Do NOT create the Pedro Pathing follower here. Creating it can initialize the
        // Pinpoint device early and conflict with hardware initialization. We'll create
        // the follower lazily when LB is pressed (below) after Pinpoint has been initialized.
        follower = null;

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
        // Transfer servos disabled for now
        depositLastTime = System.currentTimeMillis();

        // Set initial aim heading based on team (just heading, no position yet)
        aimHeadingDeg = (selectedTeam == Team.RED) ? 45.0 : 135.0;
    }

    /**
     * Applies an exponential curve to joystick input for finer control at low speeds
     * @param input Raw joystick value (-1.0 to 1.0)
     * @return Curved output (-1.0 to 1.0) with more precision at low inputs
     */
    private double applyCurve(double input) {
        // Preserve the sign of the input
        double sign = Math.signum(input);
        double magnitude = Math.abs(input);

        // Apply exponential curve: output = input^exponent
        double curved = Math.pow(magnitude, SPEED_CURVE_EXPONENT);

        return sign * curved;
    }

    /**
     * Gets the current boost velocity based on which ball is being shot
     * @return The boost velocity for the current ball
     */
    private double getCurrentBoostVelocity() {
        return switch (currentBallNumber) {
            case 0 -> BOOST_VELOCITY_BALL_1;
            case 1 -> BOOST_VELOCITY_BALL_2;
            case 2 -> BOOST_VELOCITY_BALL_3;
            default -> BOOST_VELOCITY_BALL_1;
        };
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

        // Update cached lastKnownPose from the best available source (odometry preferred)
        try {
            if (odometry != null) {
                Odometry.Position pos = odometry.getPosition();
                lastKnownPose = new Pose(pos.xMm / 25.4, pos.yMm / 25.4, pos.headingRad);
            } else if (follower != null) {
                // follower.getPose() returns inches and radians already
                lastKnownPose = follower.getPose();
            }
        } catch (Exception ignored) {}

        int targetTagId = (selectedTeam == Team.BLUE) ? BLUE_TAG_ID :
                (selectedTeam == Team.RED) ? RED_TAG_ID : -1;

        // Apply speed curve to joystick inputs
        double forward = applyCurve(-gamepad1.left_stick_y);
        double strafe = applyCurve(gamepad1.left_stick_x);
        double rotate = applyCurve(gamepad1.right_stick_x);
        // Reduce turning speed for more precise aiming
        rotate *= 0.5;

        // ZL (left trigger) - Set scoring position, trigger shooting, AND run intakes
        boolean driver1ZL = gamepad1.left_trigger > 0.5;
        boolean driver1ZLPressed = driver1ZL && !prevDriver1ZL;

        if (driver1ZLPressed) {
            // Always set/update scoring position from odometry (preferred) or follower (fallback)
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
            // Trigger shooting if deposit is active
            if (depositActive) {
                isBoosting = true;
                boostStartTime = now;
                currentBallNumber = (currentBallNumber + 1) % 3;
            }
        }

        // While ZL is held, run intakes (FLIPPED INTAKE 2)
        if (driver1ZL) {
            if (robot.intake1 != null) robot.intake1.setPower(1.0);
            if (robot.intake2 != null) robot.intake2.setPower(-INTAKE2_SCALE); // FLIPPED
        }

        prevDriver1ZL = driver1ZL;

        // Ensure manual input always overrides follower control
        boolean hasManualInput = Math.abs(forward) > 0.05 || Math.abs(strafe) > 0.05 || Math.abs(rotate) > 0.05;
        if (follower != null && hasManualInput) {
            follower.breakFollowing();
            follower = null; // Completely disable the follower
        }

        // LB (left bumper) - Auto-drive to scoring position
        boolean driver1LB = gamepad1.left_bumper;
        boolean driver1LBPressed = driver1LB && !prevDriver1LB;
        if (driver1LBPressed && hasSetScoringPosition && scoringPose != null) {
            // Determine the best starting pose for the follower
            Pose startPose;
            try {
                // Make sure Pinpoint has a fresh reading before we take a pose from it
                if (odometryEnabled && robot.pinpoint != null) {
                    try { robot.updatePinpoint(); } catch (Exception ignored) {}
                }
                 if (odometryEnabled && odometry != null) {
                     Odometry.Position pos = odometry.getPosition();
                     startPose = new Pose(pos.xMm / 25.4, pos.yMm / 25.4, pos.headingRad);
                     telemetry.addLine("Using odometry for start pose");
                 } else if (follower != null) {
                     // If we already have a follower, prefer its pose
                     follower.update();
                     startPose = follower.getPose();
                     telemetry.addLine("Using existing follower pose for start pose");
                 } else if (lastKnownPose != null) {
                     startPose = lastKnownPose;
                     telemetry.addLine("Using cached lastKnownPose for start pose");
                 } else {
                     // Fallback to team-based starting poses (same as autonomous)
                     if (selectedTeam == Team.BLUE) {
                         startPose = new Pose(26.467, 129.584, Math.toRadians(145));
                         telemetry.addLine("Fallback: BLUE team start pose");
                     } else {
                         startPose = new Pose(117.533, 129.584, Math.toRadians(35));
                         telemetry.addLine("Fallback: RED team start pose");
                     }
                 }

                 // Initialize or re-create follower and apply start pose
                if (follower == null) {
                    try {
                        // Primary attempt: standard factory method (may create its own localizer)
                        follower = Constants.createFollower(hardwareMap);
                    } catch (Exception ePrimary) {
                        telemetry.addLine("FOLLOWER CREATE ERROR: " + ePrimary.getMessage());
                        // Fallback: construct follower without the pinpoint localizer to avoid
                        // conflicts where multiple devices try to initialize the same hardware.
                        try {
                            com.pedropathing.ftc.FollowerBuilder fb = new com.pedropathing.ftc.FollowerBuilder(Constants.followerConstants, hardwareMap);
                            fb.pathConstraints(Constants.pathConstraints);
                            fb.mecanumDrivetrain(Constants.driveConstants);
                            follower = fb.build();
                            telemetry.addLine("FOLLOWER: created fallback without pinpoint localizer");
                        } catch (Exception eFallback) {
                            telemetry.addLine("FOLLOWER FALLBACK ERROR: " + eFallback.getMessage());
                            follower = null;
                        }
                    }
                }
                if (follower != null) {
                    follower.setMaxPower(1.0);
                    follower.setStartingPose(startPose);
                    telemetry.addData("FOLLOWER START", "(%.1f, %.1f) @ %.1f°", startPose.getX(), startPose.getY(), Math.toDegrees(startPose.getHeading()));
                } else {
                    telemetry.addLine("FOLLOWER NOT AVAILABLE - using basic drive");
                }

                 // If we have a scoring pose, create a simple direct path and follow it
                 try {
                     if (scoringPose != null) {
                        if (follower != null) {
                            com.pedropathing.paths.PathChain pathToScoring = follower.pathBuilder()
                                    .addPath(new com.pedropathing.geometry.BezierLine(startPose, scoringPose))
                                    .setLinearHeadingInterpolation(startPose.getHeading(), scoringPose.getHeading())
                                    .build();
                            follower.followPath(pathToScoring);
                        } else {
                            telemetry.addLine("SKIPPING PATH: follower null");
                        }
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

        // Drive using the Pedro follower if available
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
            // Use basic drive if follower not available
            drive.drive(forward, strafe, rotate, false, gamepad1.right_bumper);
        }

        // Intake control - gamepad2 has priority, but ZL is handled above
        if (!driver1ZL) {
            // Intake1 - controlled by gamepad2 triggers when ZL not held
            if (robot.intake1 != null) {
                if (gamepad2.left_trigger > 0.1) {
                    robot.intake1.setPower(1.0);
                } else if (gamepad2.right_trigger > 0.1) {
                    robot.intake1.setPower(-1.0);
                } else {
                    robot.intake1.setPower(0.0);
                }
            }

            // Intake2 - controlled by gamepad2 buttons A (IN) and B (OUT) when ZL not held
            if (robot.intake2 != null) {
                if (gamepad2.a) {
                    robot.intake2.setPower(-INTAKE2_SCALE); // FLIPPED
                } else if (gamepad2.b) {
                    robot.intake2.setPower(INTAKE2_SCALE); // FLIPPED
                } else {
                    robot.intake2.setPower(0.0);
                }
            }
        }

        // Toggle deposit active state on gamepad2 X button press
        if (gamepad2.x && !lastXButtonState) {
            depositActive = !depositActive;
            if (!depositActive) {
                depositIntegral = 0.0;
                depositLastError = 0.0;
                currentBallNumber = 0; // Reset ball counter when deposit turns off
                isBoosting = false;
                robot.stopDeposit();
            }
        }
        lastXButtonState = gamepad2.x;

        // PIDF velocity control with feedforward boost for deposit motors
        if (depositActive && robot.deposit1 != null && robot.deposit2 != null) {
            double vel1 = robot.getDeposit1Velocity();
            double vel2 = robot.getDeposit2Velocity();
            double currentVelocity = (vel1 + vel2) / 2.0;

            // Determine target velocity (with boost if active)
            double targetVelocity = DEPOSIT_TARGET_VELOCITY;
            if (isBoosting) {
                targetVelocity += getCurrentBoostVelocity();
            }

            double error = targetVelocity - currentVelocity;

            long currentTime = System.currentTimeMillis();
            double dt = (currentTime - depositLastTime) / 1000.0;
            if (dt <= 0) dt = 0.02;
            depositLastTime = currentTime;

            // PID terms
            double pTerm = DEPOSIT_KP * error;

            depositIntegral += error * dt;
            double iTerm = DEPOSIT_KI * depositIntegral;

            double derivative = (error - depositLastError) / dt;
            double dTerm = DEPOSIT_KD * derivative;
            depositLastError = error;

            // Feedforward term (based on target velocity)
            double fTerm = DEPOSIT_KF * targetVelocity;

            double power = pTerm + iTerm + dTerm + fTerm;

            power = Math.max(DEPOSIT_POWER_MIN, Math.min(DEPOSIT_POWER_MAX, power));

            robot.deposit1.setPower(power);
            robot.deposit2.setPower(power);
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

        // === DEPOSIT STATUS ===
        telemetry.addLine("─── DEPOSIT ───");
        telemetry.addData("Status", depositActive ? "✓ ACTIVE" : "✗ STOPPED");

        if (robot.deposit1 != null && robot.deposit2 != null && depositActive) {
            double vel1 = robot.getDeposit1Velocity();
            double vel2 = robot.getDeposit2Velocity();
            double avgVel = (vel1 + vel2) / 2.0;

            double targetVel = DEPOSIT_TARGET_VELOCITY;
            if (isBoosting) {
                targetVel += getCurrentBoostVelocity();
            }

            double error = targetVel - avgVel;
            boolean atTarget = Math.abs(error) < DEPOSIT_TOLERANCE;

            telemetry.addData("Target Velocity", "%.0f ticks/s", targetVel);
            telemetry.addData("Current Velocity", "%.0f / %.0f (Avg: %.0f)", vel1, vel2, avgVel);
            telemetry.addData("Error", "%s%.0f ticks/s", atTarget ? "✓ " : "", error);
            telemetry.addData("Power", "%.2f / %.2f", robot.deposit1.getPower(), robot.deposit2.getPower());

            if (isBoosting) {
                telemetry.addData("BOOST", "✓ ACTIVE (Ball %d)", currentBallNumber + 1);
                long timeRemaining = (long)(BOOST_DURATION_MS - (now - boostStartTime));
                telemetry.addData("Boost Time Left", "%d ms", timeRemaining);
            } else {
                telemetry.addData("Next Ball", "#%d", currentBallNumber + 1);
            }
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
        // Debug cached pose
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

        // === PIDF TUNING INFO ===
        telemetry.addLine("─── PIDF TUNING ───");
        telemetry.addData("Kp, Ki, Kd, Kf", "%.3f, %.3f, %.3f, %.3f", DEPOSIT_KP, DEPOSIT_KI, DEPOSIT_KD, DEPOSIT_KF);
        telemetry.addData("Speed Curve Exp", "%.2f", SPEED_CURVE_EXPONENT);
        telemetry.addData("Boost Velocities", "%.0f, %.0f, %.0f", BOOST_VELOCITY_BALL_1, BOOST_VELOCITY_BALL_2, BOOST_VELOCITY_BALL_3);
        telemetry.addData("Boost Duration", "%.0f ms", BOOST_DURATION_MS);
        telemetry.addLine("ZL: set pos/shoot/intakes | LB: auto-drive");
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
