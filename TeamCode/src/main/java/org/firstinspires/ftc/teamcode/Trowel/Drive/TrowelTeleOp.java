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

    // For auto-aim smoothing
    private static final int DERIVATIVE_WINDOW = 4;
    private final double[] derivativeBuffer = new double[DERIVATIVE_WINDOW];
    private int derivativeIndex = 0;
    private int derivativeCount = 0;
    private double aimLastError = 0.0;
    // Auto-aim tunables to smooth oscillation (panels adjustable)
    // Increased KP/KD and max power/slew to speed up auto-aim while
    // keeping damping higher to avoid added oscillation. KI slightly
    // reduced to lower integral windup risk when increasing KP.
    public static double AIM_KP = 0.014;       // increased for faster response
    public static double AIM_KI = 0.00020;     // slightly reduced integral to avoid windup
    public static double AIM_KD = 0.0012;      // increased derivative to add damping
    public static double AIM_DEADBAND_DEG = 3.0; // keep deadband to prevent small hunts
    public static double AIM_STATIC_FF = 0.045;  // slightly higher static kick to overcome stiction
    public static double AIM_MAX_POWER = 0.35;   // allow stronger turns (clamped)
    public static double AIM_MAX_SLEW_PER_LOOP = 0.05; // allow larger slew per loop for faster change
    public static double AIM_SETTLE_ERR_DEG = 2.0;
    public static double AIM_SETTLE_DERIV_DEG = 0.30;   // tighter derivative settle
    public static int AIM_SETTLE_LOOPS = 8;             // require a bit longer settle

    private double aimIntegral = 0.0;
    private double lastAimOutput = 0.0;
    private int aimSettledCounter = 0;
    private boolean prevLeftBumper = false;
    // Track driver1 ZL (left trigger) rising edge to set aim heading
    private boolean prevDriver1ZL = false;

    // Deposit spin-up (open-loop) burst to overcome static friction before switching to velocity control
    public static double DEPOSIT_SPINUP_POWER = 1; // open-loop power during spin-up (0-1)
    public static int DEPOSIT_SPINUP_MS = 645; // duration of open-loop spin-up in milliseconds
    private long depositSpinupEndTime = 0; // 0 means not spinning up; -1 means spin-up finished

    private TrowelHardware robot;
    private RandyButterNubs drive;

    private Odometry odometry;
    private VisionLocalization visionLocalization;
    private boolean visionEnabled = false;
    private boolean odometryEnabled = false;

    // Transfer positions used by transfer servos (keep original semantics)
    private static final double TRANSFER_IN = 0.0;
    private static final double TRANSFER_OUT = 1.0;
    private static final double TRANSFER_NEUTRAL = 0.5;

    // Pedro Pathing follower instance (optional). If available, we'll use it for teleop drive
    // and for heading-controlled auto-aim.
    private Follower follower = null;
    private boolean followerTeleopStarted = false;

    // Simple follower-based heading controller tunables
    public static double FOLLOWER_AIM_KP = 0.008; // maps degrees -> rotate input roughly; tune as needed
    public static double FOLLOWER_AIM_DEADBAND_DEG = 2.0;
    public static double FOLLOWER_AIM_MAX_ROT = 0.6; // clamp rotate command magnitude

    // Scale factor for the second-stage intake (intake2). Reduce by 10% as requested.
    private static final double INTAKE2_SCALE = 1.0;

    private double depositTargetVelocity = RandyButterNubs.DEFAULT_DEPOSIT_VELOCITY;
    private boolean depositActive = false;
    private boolean lastXButtonState = false;

    // Panels-tunable software feedforward factor (fractional). Keep PIDF constants unchanged.
    public static double DEPOSIT_FF_FACTOR = 0.05; // default small multiplicative FF
    // Panels-tunable absolute feedforward boost in ticks/sec (additive)
    public static double DEPOSIT_FF_BOOST_TICKS = 240.0; // default additive boost (ticks/sec)
    // Panels-tunable deposit target velocity so you can tune via the Panels UI
    public static double PANEL_DEPOSIT_TARGET_VELOCITY = RandyButterNubs.DEFAULT_DEPOSIT_VELOCITY;

    // ===== Gamepad2 D-Pad tuning constants/vars =====
    private static final double DEPOSIT_STEP_SMALL = 5.0;
    private static final double DEPOSIT_STEP_LARGE = 25.0;
    private static final double DEPOSIT_MIN_VELOCITY = 0.0;
    private static final double DEPOSIT_MAX_VELOCITY = 5000.0;
    private boolean prevGp2DpadUp = false, prevGp2DpadDown = false, prevGp2DpadLeft = false, prevGp2DpadRight = false;
    private long lastGp2DpadChange = 0;
    private static final long FIRST_REPEAT_DELAY_MS = 350;
    private static final long REPEAT_INTERVAL_MS = 120;
    // =================================================

    // Auto-aim reference heading (can be overwritten by driver)
    private double aimHeadingDeg = 0.0;
    private boolean prevDriver1Y = false;

    @Override
    public void init() {
        robot = new TrowelHardware(hardwareMap);
        drive = new RandyButterNubs(robot.frontLeft, robot.frontRight, robot.backLeft, robot.backRight);

        // Try to create the Pedro Pathing follower. If it fails, we'll silently fall back to
        // the legacy "drive" object.
        try {
            follower = Constants.createFollower(hardwareMap);
        } catch (Exception ignored) {
            follower = null;
        }

        // Apply initial feedforward factor (can be tuned via Panels because this class is @Configurable)
        robot.setDepositFeedforwardFactor(DEPOSIT_FF_FACTOR);

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
        if (robot.transfer1 != null) robot.transfer1.setPosition(TRANSFER_OUT);
        if (robot.transfer2 != null) robot.transfer2.setPosition(TRANSFER_OUT);
    }

    @Override
    public void loop() {
        // If DEPOSIT_FF_FACTOR is changed via panels, propagate it to the robot instance
        if (robot != null && robot.getDepositFeedforwardFactor() != DEPOSIT_FF_FACTOR) {
            robot.setDepositFeedforwardFactor(DEPOSIT_FF_FACTOR);
        }
        // Propagate absolute boost ticks as well
        if (robot != null && robot.getDepositFeedforwardBoostTicks() != DEPOSIT_FF_BOOST_TICKS) {
            robot.setDepositFeedforwardBoostTicks(DEPOSIT_FF_BOOST_TICKS);
        }

        // Sync panels deposit target velocity to runtime variable. This allows tuning via the Panels UI.
        if (PANEL_DEPOSIT_TARGET_VELOCITY != depositTargetVelocity) {
            depositTargetVelocity = PANEL_DEPOSIT_TARGET_VELOCITY;
            // If deposit is currently active, apply immediately
            if (depositActive) robot.setDepositVelocity(depositTargetVelocity);
        }

        // ===== Gamepad2 D-Pad tuning: override/update depositTargetVelocity =====
        boolean gp2Up = gamepad2.dpad_up;
        boolean gp2Down = gamepad2.dpad_down;
        boolean gp2Left = gamepad2.dpad_left;
        boolean gp2Right = gamepad2.dpad_right;
        long now = System.currentTimeMillis();

        if (gp2Up && (!prevGp2DpadUp || now - lastGp2DpadChange > FIRST_REPEAT_DELAY_MS)) {
            depositTargetVelocity = Math.min(DEPOSIT_MAX_VELOCITY, depositTargetVelocity + DEPOSIT_STEP_LARGE);
            lastGp2DpadChange = now;
        }
        if (gp2Down && (!prevGp2DpadDown || now - lastGp2DpadChange > FIRST_REPEAT_DELAY_MS)) {
            depositTargetVelocity = Math.max(DEPOSIT_MIN_VELOCITY, depositTargetVelocity - DEPOSIT_STEP_LARGE);
            lastGp2DpadChange = now;
        }
        if (gp2Right && (!prevGp2DpadRight || now - lastGp2DpadChange > FIRST_REPEAT_DELAY_MS)) {
            depositTargetVelocity = Math.min(DEPOSIT_MAX_VELOCITY, depositTargetVelocity + DEPOSIT_STEP_SMALL);
            lastGp2DpadChange = now;
        }
        if (gp2Left && (!prevGp2DpadLeft || now - lastGp2DpadChange > FIRST_REPEAT_DELAY_MS)) {
            depositTargetVelocity = Math.max(DEPOSIT_MIN_VELOCITY, depositTargetVelocity - DEPOSIT_STEP_SMALL);
            lastGp2DpadChange = now;
        }

        if ((gp2Up || gp2Down || gp2Left || gp2Right) && now - lastGp2DpadChange > REPEAT_INTERVAL_MS) {
            lastGp2DpadChange = now - (REPEAT_INTERVAL_MS + 1);
        }

        prevGp2DpadUp = gp2Up;
        prevGp2DpadDown = gp2Down;
        prevGp2DpadLeft = gp2Left;
        prevGp2DpadRight = gp2Right;

        // Make sure panels UI reflects the runtime change
        if (PANEL_DEPOSIT_TARGET_VELOCITY != depositTargetVelocity) {
            PANEL_DEPOSIT_TARGET_VELOCITY = depositTargetVelocity;
        }

        // If deposit is active, ensure robot receives the updated velocity command immediately
        if (robot != null && depositActive && depositSpinupEndTime == -1) {
            robot.setDepositVelocity(depositTargetVelocity);
        }
        // ==============================================================

        if (odometryEnabled && robot.pinpoint != null) {
            robot.updatePinpoint();
            if (odometry == null) {
                odometry = new Odometry(robot, robot.pinpoint);
                // Set odometry's starting heading to 0 degrees (90 deg left from 90)
                // Odometry does not have setPosition, so set hRad directly
                java.lang.reflect.Field hRadField;
                try {
                    hRadField = Odometry.class.getDeclaredField("hRad");
                    hRadField.setAccessible(true);
                    // Initialize starting heading rotated 90 degrees counter-clockwise
                    hRadField.set(odometry, Math.toRadians(90));
                } catch (Exception ignored) {}
            }
            odometry.update();
        } else if (odometry == null) {
            // create odometry with robot so IMU fallback works even if pinpoint is not initialized
            odometry = new Odometry(robot, null);
            // Set odometry's starting heading to 0 degrees (90 deg left from 90)
            try {
                java.lang.reflect.Field hRadField = Odometry.class.getDeclaredField("hRad");
                hRadField.setAccessible(true);
                // Initialize starting heading rotated 90 degrees counter-clockwise
                hRadField.set(odometry, Math.toRadians(90));
            } catch (Exception ignored) {}
            // do not call update here until pinpoint or IMU is available
        }

        if (visionEnabled && visionLocalization != null) {
            visionLocalization.update();
        }

        int targetTagId = (selectedTeam == Team.BLUE) ? BLUE_TAG_ID :
                (selectedTeam == Team.RED) ? RED_TAG_ID : -1;

        double forward = -gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;
        double rotate = gamepad1.right_stick_x;

        // Overwrite aim heading with current odometry heading when driver 1 presses Y (rising edge)
        boolean driver1Y = gamepad1.y;
        if (driver1Y && !prevDriver1Y) {
            if (odometry != null) {
                aimHeadingDeg = odometry.getPosition().getHeadingDeg();
            }
        }
        prevDriver1Y = driver1Y;

        // Detect rising edge on driver1 ZL (left trigger > 0.5) and set heading instantly to aiming angle
        boolean driver1ZL = gamepad1.right_trigger > 0.5;
        if (driver1ZL && !prevDriver1ZL) {
            double aimAngleDeg = (selectedTeam == Team.RED) ? 45.0 : 135.0;
            // Set odometry internal heading via reflection
            try {
                if (odometry != null) {
                    java.lang.reflect.Field hRadField = Odometry.class.getDeclaredField("hRad");
                    hRadField.setAccessible(true);
                    hRadField.set(odometry, Math.toRadians(aimAngleDeg));
                }
            } catch (Exception ignored) {}

            // Update Pedro follower starting pose if available
            try {
                if (follower != null) {
                    follower.update();
                    Pose p = follower.getPose();
                    Pose newPose = new Pose(p.getX(), p.getY(), Math.toRadians(aimAngleDeg));
                    follower.setStartingPose(newPose);
                }
            } catch (Exception ignored) {}
        }
        prevDriver1ZL = driver1ZL;

        double autoRotate = 0.0;

        // Remove all vision-based auto-aiming logic
        // Keep the structure for LB (left bumper) pressed
        if (gamepad1.left_bumper) {
            // TODO: Insert auto-aiming logic here (currently vision-based code removed)
            // For now, just indicate LB is pressed
            telemetry.addLine("LB (auto-aim trigger) is pressed");
        }

        // NOTE: autoRotate sign chosen to match drive.rotate sign convention

        // Speed scaling: LB = 70% slow, RB = 30% slow. These scale translational and driver rotate input
        double speedScale = 1.0;
        if (gamepad1.left_bumper) speedScale = 0.7; // 70% slow mode
        else if (gamepad1.right_bumper) speedScale = 0.5; // 30% slow mode

        forward *= speedScale;
        strafe *= speedScale;
        // Keep vision/heading autoRotate unaffected by scaling; scale only driver rotation input
        rotate *= speedScale;

        double finalRotate = rotate + autoRotate;

        // Drive using the Pedro follower if available; otherwise fall back to RandyButterNubs.
        if (follower != null) {
            // Ensure teleop drive mode started once
            if (!followerTeleopStarted) {
                try {
                    follower.startTeleopDrive(true);
                } catch (Exception ignored) {}
                followerTeleopStarted = true;
            }

            try {
                // When left bumper is held we want auto-aim: keep translational joystick control but
                // override the rotate input with follower-based heading control (target 45 or 135 deg)
                if (gamepad1.left_bumper) {
                    double targetHeading = aimHeadingDeg;
                    // Make sure follower pose is updated before reading
                    follower.update();
                    Pose fPose = follower.getPose();
                    double currentHeadingDeg = Math.toDegrees(fPose.getHeading());
                    double error = targetHeading - currentHeadingDeg;
                    while (error > 180) error -= 360;
                    while (error < -180) error += 360;

                    double rotCmd = 0.0;
                    if (Math.abs(error) > FOLLOWER_AIM_DEADBAND_DEG) {
                        rotCmd = FOLLOWER_AIM_KP * error;
                        // clamp
                        rotCmd = Math.max(-FOLLOWER_AIM_MAX_ROT, Math.min(FOLLOWER_AIM_MAX_ROT, rotCmd));
                    }

                    // NOTE: Tuning.setTeleOpDrive uses: (-forward, -strafe, -rotate) sign convention
                    // We pass forward as already-negated, but flip strafe/rotate signs to match the follower API.
                    // Pass rotCmd directly (sign chosen so follower steers toward reducing the heading error).
                    follower.setTeleOpDrive(forward, -strafe, rotCmd, true);
                    // Let the follower consume the command and update internal controllers
                    follower.update();
                } else {
                    // Normal teleop: pass driver inputs through to follower
                    follower.setTeleOpDrive(forward, -strafe, -rotate, true);
                    follower.update();
                }
            } catch (Exception e) {
                // On any follower error, fall back to legacy drive
                drive.drive(forward, strafe, finalRotate, gamepad1.left_bumper, gamepad1.right_bumper);
            }

        } else {
            drive.drive(forward, strafe, finalRotate, gamepad1.left_bumper, gamepad1.right_bumper);
        }

        // DRIVER1 transfer + intake override: holding gamepad1 triggers runs intakes + sets transfer servos
        boolean driver1IntakeIn = gamepad1.left_trigger > 0.1;
        boolean driver1IntakeOut = gamepad1.right_trigger > 0.1;

        if (driver1IntakeIn) {
            // Driver1 ZL: run intakes IN and set transfers to IN
            setTransferIn();
            if (robot.intake1 != null) robot.intake1.setPower(1.0);
            if (robot.intake2 != null) robot.intake2.setPower(-INTAKE2_SCALE);
        } else if (driver1IntakeOut) {
            // Driver1 ZR: run intakes OUT (reverse) and set transfers to OUT
            setTransferOut();
            if (robot.intake1 != null) robot.intake1.setPower(-1.0);
            if (robot.intake2 != null) robot.intake2.setPower(-INTAKE2_SCALE);
        } else {
            // No driver1 override - transfers go neutral and restore gamepad2 controls (unchanged)
            setTransferNeutral();

            // Intake1 - controlled by gamepad2 triggers/bumpers: left_trigger = IN, right_trigger = OUT
            if (robot.intake1 != null) {
                if (gamepad2.left_trigger > 0.1 || gamepad2.left_bumper) {
                    robot.intake1.setPower(1.0);
                } else if (gamepad2.right_trigger > 0.1) {
                    robot.intake1.setPower(-1.0);
                } else {
                    robot.intake1.setPower(0.0);
                }
            }

            // Intake2 - controlled by gamepad2 buttons A (IN) and B (OUT)
            if (robot.intake2 != null) {
                if (gamepad2.a) {
                    robot.intake2.setPower(INTAKE2_SCALE);
                } else if (gamepad2.b) {
                    robot.intake2.setPower(-1.0 * INTAKE2_SCALE);
                } else {
                    robot.intake2.setPower(0.0);
                }
            }
        }

        if (gamepad2.x && !lastXButtonState) {
            depositActive = !depositActive;
            if (depositActive) {
                // start open-loop spin-up burst, then we'll switch to velocity control after DEPOSIT_SPINUP_MS
                depositSpinupEndTime = System.currentTimeMillis() + DEPOSIT_SPINUP_MS;
                // Apply open-loop power to both deposit motors to get them spinning
                if (robot.deposit1 != null) robot.deposit1.setPower(DEPOSIT_SPINUP_POWER);
                if (robot.deposit2 != null) robot.deposit2.setPower(DEPOSIT_SPINUP_POWER);
            } else {
                // stopped
                depositSpinupEndTime = 0;
                robot.stopDeposit();
            }
        }
        lastXButtonState = gamepad2.x;

        // Handle non-blocking spin-up transition
        if (depositActive) {
            if (depositSpinupEndTime > 0) {
                long nowTs = System.currentTimeMillis();
                if (nowTs >= depositSpinupEndTime) {
                    // Spin-up finished: switch to closed-loop velocity control
                    robot.setDepositVelocity(depositTargetVelocity);
                    depositSpinupEndTime = -1; // mark finished
                }
            } else if (depositSpinupEndTime == -1) {
                // already in velocity control; ensure velocity command is maintained
                robot.setDepositVelocity(depositTargetVelocity);
            }
        }

        // Auto-aim using odometry heading (fast, bidirectional) when LB is held
        double autoRotationPower = 0.0;
        if (gamepad1.left_bumper) {
            // Legacy fallback auto-aim (only used if follower is unavailable). Keep for safety.
            // Use base angles (negative values) but flip sign for BLUE team only
            double targetAngle = aimHeadingDeg;
            double currentAngle = 0.0;
            if (odometry != null) {
                currentAngle = odometry.getPosition().getHeadingDeg();
            }
            double error = targetAngle - currentAngle;
            // Normalize error to [-180, 180]
            while (error > 180) error -= 360;
            while (error < -180) error += 360;

            // Derivative smoothing
            double rawDerivative = error - aimLastError;
            derivativeBuffer[derivativeIndex] = rawDerivative;
            derivativeIndex = (derivativeIndex + 1) % DERIVATIVE_WINDOW;
            if (derivativeCount < DERIVATIVE_WINDOW) derivativeCount++;
            double sum = 0.0;
            for (int i = 0; i < derivativeCount; i++) sum += derivativeBuffer[i];
            double smoothedDerivative = sum / derivativeCount;

            // Integral with simple anti-windup
            if (Math.abs(error) <= AIM_DEADBAND_DEG) {
                aimSettledCounter++;
                aimIntegral = 0.0;
            } else {
                aimSettledCounter = 0;
                aimIntegral += error;
                double integralMax = AIM_MAX_POWER / Math.max(1e-6, AIM_KI);
                aimIntegral = Math.max(-integralMax, Math.min(integralMax, aimIntegral));
            }

            // Compute rotation
            double rotationCmd = (AIM_KP * error) + (AIM_KD * smoothedDerivative) + (AIM_KI * aimIntegral);
            if (Math.abs(error) > AIM_DEADBAND_DEG && Math.abs(rotationCmd) < AIM_STATIC_FF) {
                rotationCmd = AIM_STATIC_FF * Math.signum(rotationCmd == 0 ? error : rotationCmd);
            }
            // Deadband clamp
            if (Math.abs(error) <= AIM_DEADBAND_DEG) {
                rotationCmd = 0.0;
            }
            // Slew limit and clamp
            double maxStep = AIM_MAX_SLEW_PER_LOOP;
            rotationCmd = Math.max(lastAimOutput - maxStep, Math.min(lastAimOutput + maxStep, rotationCmd));
            rotationCmd = Math.max(-AIM_MAX_POWER, Math.min(AIM_MAX_POWER, rotationCmd));

            autoRotationPower = rotationCmd;
            lastAimOutput = rotationCmd;
            aimLastError = error;

            boolean aimSettled = Math.abs(error) <= AIM_SETTLE_ERR_DEG && Math.abs(smoothedDerivative) <= AIM_SETTLE_DERIV_DEG && aimSettledCounter >= AIM_SETTLE_LOOPS;
            telemetry.addData("AutoAim Target", targetAngle);
            telemetry.addData("AutoAim Heading", currentAngle);
            telemetry.addData("AutoAim Error", error);
            telemetry.addData("AutoAim Deriv", smoothedDerivative);
            telemetry.addData("AutoAim Power", autoRotationPower);
            telemetry.addData("AutoAim Settled", aimSettled ? "YES" : "NO");
        } else {
            // Reset filters when not aiming
            derivativeCount = 0;
            derivativeIndex = 0;
            aimIntegral = 0.0;
            lastAimOutput = 0.0;
            aimSettledCounter = 0;
            aimLastError = 0.0;
        }

        // Apply auto rotation if LB is held, otherwise use driver input
        // If follower is present we already applied the auto-rotation via follower.setTeleOpDrive above.
        // If the follower is not present, apply the legacy auto rotation directly to the drive.
        if (follower == null && gamepad1.left_bumper) {
            drive.drive(0, 0, autoRotationPower, false, false);
        }

        prevLeftBumper = gamepad1.left_bumper;

        telemetry.addData("=== TEAM & AUTO-TURN ===", "");
        String teamStr = (selectedTeam == Team.BLUE) ? "BLUE" :
                (selectedTeam == Team.RED) ? "RED" : "NONE";
        telemetry.addData("Team", "%s (Target Tag: %d)", teamStr, targetTagId);
        telemetry.addData("Aim Heading (deg)", "%.1f", aimHeadingDeg);

        telemetry.addData("Final Rotate (driver+auto)", "%.2f", finalRotate);

        telemetry.addData("=== DRIVER 1 ===", "");
        telemetry.addData("Drive", "Forward: %.2f, Strafe: %.2f, Rotate: %.2f", forward, strafe, finalRotate);
        if (robot.transfer1 != null && robot.transfer2 != null) {
            telemetry.addData("Transfer Positions", "T1: %.2f, T2: %.2f", robot.transfer1.getPosition(), robot.transfer2.getPosition());
        }

        telemetry.addData("=== DRIVER 2 ===", "");
        if (robot.intake1 != null) {
            telemetry.addData("Intake 1 Power", "%.2f", robot.intake1.getPower());
        }
        if (robot.intake2 != null) {
            telemetry.addData("Intake 2 Power", "%.2f", robot.intake2.getPower());
        }
        telemetry.addData("Deposit", "Active: %s, Target Velocity: %.1f", depositActive ? "YES" : "NO", depositTargetVelocity);
        if (robot.deposit1 != null && robot.deposit2 != null) {
            telemetry.addData("Deposit Power", "D1: %.2f, D2: %.2f", robot.deposit1.getPower(), robot.deposit2.getPower());
            telemetry.addData("Deposit Velocity", "D1: %.0f, D2: %.0f ticks/s", robot.getDeposit1Velocity(), robot.getDeposit2Velocity());
            telemetry.addData("Deposit RPM", "D1: %.0f, D2: %.0f, Avg: %.0f", robot.getDeposit1RPM(), robot.getDeposit2RPM(), robot.getAverageDepositRPM());
        }

        // Show feedforward factor and contribution for tuning visibility
        double ffFactor = robot.getDepositFeedforwardFactor();
        double ffContribution = robot.computeDepositFeedforwardContribution(depositTargetVelocity);
        telemetry.addData("Deposit/TargetVelocity", "%.1f ticks/s", depositTargetVelocity);
        telemetry.addData("Deposit/FF Factor", "%.3f", ffFactor);
        telemetry.addData("Deposit/FF BoostTicks", "%.1f ticks/s", robot.getDepositFeedforwardBoostTicks());
        telemetry.addData("Deposit/FF Contribution", "%.1f ticks/s", ffContribution);
        telemetry.addData("Deposit/CommandedVelocity (wFF)", "%.1f ticks/s", depositTargetVelocity + ffContribution);

        if (odometryEnabled && odometry != null) {
            Odometry.Position pos = odometry.getPosition();
            telemetry.addData("=== ODOMETRY ===", "");
            // Position stores mm internally
            double xIn = pos.xMm / 25.4;
            double yIn = pos.yMm / 25.4;
            double headingDeg = Math.toDegrees(pos.headingRad);
            telemetry.addData("X (in)", "%.2f", xIn);
            telemetry.addData("Y (in)", "%.2f", yIn);
            telemetry.addData("Heading (deg)", "%.1f", headingDeg);
        } else {
            telemetry.addData("Odometry", "Disabled");
        }

        if (visionEnabled && visionLocalization != null) {
            telemetry.addData("=== VISION LOCALIZATION ===", "");
            telemetry.addData("Tags Detected", visionLocalization.getDetectedTagCount());
            telemetry.addData("Robot X (in)", "%.2f", visionLocalization.getRobotX());
            telemetry.addData("Robot Y (in)", "%.2f", visionLocalization.getRobotY());
            telemetry.addData("Robot Heading (deg)", "%.2f", visionLocalization.getRobotHeading());
            telemetry.addData("Confidence", "%.2f", visionLocalization.getConfidence());
            telemetry.addData("Recent Detection", visionLocalization.hasRecentDetection() ? "YES" : "NO");

            // 2D range (ftcPose.range) to the selected tag (if any)
            double range2d = Double.NaN;
            if (targetTagId > 0) range2d = visionLocalization.getRangeToTag(targetTagId);

            // 3D euclidean distances (inches)
            double dist3dTarget = Double.NaN;
            if (targetTagId > 0) dist3dTarget = visionLocalization.get3dDistanceToTag(targetTagId);
            double dist3dAvg = visionLocalization.get3dDistanceAvg();

            // Safe formatting using Locale.US to avoid locale issues
            if (!Double.isNaN(range2d)) {
                telemetry.addData("Range to Target (2D, in)", String.format(java.util.Locale.US, "%.2f", range2d));
            } else {
                telemetry.addData("Range to Target (2D, in)", "N/A");
            }

            if (!Double.isNaN(dist3dTarget)) {
                telemetry.addData("3D Distance to Target (in)", String.format(java.util.Locale.US, "%.2f", dist3dTarget));
            } else {
                telemetry.addData("3D Distance to Target (in)", "N/A");
            }

            if (!Double.isNaN(dist3dAvg)) {
                telemetry.addData("3D Distance Avg (in)", String.format(java.util.Locale.US, "%.2f", dist3dAvg));
            } else {
                telemetry.addData("3D Distance Avg (in)", "N/A");
            }
        } else {
            telemetry.addData("Vision", "Disabled");
        }

        // Add explicit camera/vision diagnostics
        if (visionLocalization != null) {
            String camState = visionLocalization.getCameraState();
            telemetry.addData("=== VISION DIAGNOSTICS ===", "");
            telemetry.addData("Camera State", camState);
            telemetry.addData("Vision Initialized", visionLocalization.isReady() ? "YES" : "NO");
            if (!visionLocalization.isReady()) {
                telemetry.addLine("WARNING: Vision not initialized or camera not streaming!");
                telemetry.addLine("Check camera connection and config name (should be 'Webcam 1')");
            }
        } else {
            telemetry.addData("=== VISION DIAGNOSTICS ===", "");
            telemetry.addLine("VisionLocalization instance is NULL!");
        }

        telemetry.addLine(robot.getMotorPowers());
        telemetry.addLine(robot.getMotorConfigurations());
        telemetry.addLine(robot.getInitializationStatus());

        telemetry.update();
    }

    @Override
    public void stop() {
        drive.stop();
        robot.stop();
    }


    // Helper: set transfer actuators to IN (use CRServo.setPower if the hardware is a CRServo; otherwise set Servo position)
    private void setTransferIn() {
        try {
            // Try CRServo first
            if (robot.transfer1 instanceof com.qualcomm.robotcore.hardware.CRServo) {
                ((com.qualcomm.robotcore.hardware.CRServo) robot.transfer1).setPower(1.0);
            } else if (robot.transfer1 != null) {
                robot.transfer1.setPosition(TRANSFER_IN);
            }
        } catch (Exception ignored) {
        }

        try {
            if (robot.transfer2 instanceof com.qualcomm.robotcore.hardware.CRServo) {
                // transfer2 was configured in hardware as reversed direction; send full power forward to match position semantics
                ((com.qualcomm.robotcore.hardware.CRServo) robot.transfer2).setPower(1.0);
            } else if (robot.transfer2 != null) {
                robot.transfer2.setPosition(TRANSFER_IN);
            }
        } catch (Exception ignored) {
        }
    }

    // Helper: set transfer actuators to OUT
    private void setTransferOut() {
        try {
            if (robot.transfer1 instanceof com.qualcomm.robotcore.hardware.CRServo) {
                ((com.qualcomm.robotcore.hardware.CRServo) robot.transfer1).setPower(-1.0);
            } else if (robot.transfer1 != null) {
                robot.transfer1.setPosition(TRANSFER_OUT);
            }
        } catch (Exception ignored) {
        }

        try {
            if (robot.transfer2 instanceof com.qualcomm.robotcore.hardware.CRServo) {
                ((com.qualcomm.robotcore.hardware.CRServo) robot.transfer2).setPower(-1.0);
            } else if (robot.transfer2 != null) {
                robot.transfer2.setPosition(TRANSFER_OUT);
            }
        } catch (Exception ignored) {
        }
    }

    // Helper: set transfer actuators to neutral/stop
    private void setTransferNeutral() {
        try {
            if (robot.transfer1 instanceof com.qualcomm.robotcore.hardware.CRServo) {
                ((com.qualcomm.robotcore.hardware.CRServo) robot.transfer1).setPower(0.0);
            } else if (robot.transfer1 != null) {
                robot.transfer1.setPosition(TRANSFER_NEUTRAL);
            }
        } catch (Exception ignored) {
        }

        try {
            if (robot.transfer2 instanceof com.qualcomm.robotcore.hardware.CRServo) {
                ((com.qualcomm.robotcore.hardware.CRServo) robot.transfer2).setPower(0.0);
            } else if (robot.transfer2 != null) {
                robot.transfer2.setPosition(TRANSFER_NEUTRAL);
            }
        } catch (Exception ignored) {
        }
    }

}
