package org.firstinspires.ftc.teamcode.Trowel.Drive;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Trowel.Configs.RandyButterNubs;
import org.firstinspires.ftc.teamcode.Trowel.common.Odometry;
import org.firstinspires.ftc.teamcode.Trowel.common.VisionLocalization;
import org.firstinspires.ftc.teamcode.Trowel.Configs.TrowelHardware;

@Configurable
@TeleOp(name = "Randy Butter Knubs", group = "Trowel")
public class TrowelTeleOp extends OpMode {

    private enum Team { NONE, BLUE, RED }
    private Team selectedTeam = Team.NONE;

    public static int BLUE_TAG_ID = 20;
    public static int RED_TAG_ID = 24;

    public static double AUTO_TURN_SPEED = 0.3;
    public static double YAW_THRESHOLD = 2.0;
    public static double HEADING_RESET_SPEED = 0.4;
    public static double HEADING_THRESHOLD = 2.0;
    // PD controller settings for vision auto-aim (tuned conservative)
    // Reduced gains and max power to slow down auto-aim speed
    public static double SNAP_KP = 0.035;      // reduced proportional gain
    public static double SNAP_KD = 0.006;      // reduced derivative gain
    public static double SNAP_MAX_POWER = 0.15; // Lowered max rotation power
    public static double SNAP_DEADZONE_DEG = 0.25; // Lowered deadzone for tighter aiming
    // Hardcoded shift magnitude: 2 degrees. Sign will be applied per-team (blue -> left, red -> right)
    public static final double HARD_AIM_SHIFT_DEG = 2.0;
    private double aimLastError = 0.0;
    private long aimLastTime = 0;
    private int aimLastErrorSign = 0;
    private boolean autoTurnEnabled = false;
    // Deposit spin-up (open-loop) burst to overcome static friction before switching to velocity control
    public static double DEPOSIT_SPINUP_POWER = 1; // open-loop power during spin-up (0-1)
    public static int DEPOSIT_SPINUP_MS = 365; // duration of open-loop spin-up in milliseconds
    private long depositSpinupEndTime = 0; // 0 means not spinning up; -1 means spin-up finished

    private TrowelHardware robot;
    private RandyButterNubs drive;

    private Odometry odometry;
    private VisionLocalization visionLocalization;
    private boolean visionEnabled = false;
    private boolean odometryEnabled = false;

    private static final double TRANSFER_IN = 0.0;
    private static final double TRANSFER_OUT = 1.0;
    private static final double TRANSFER_NEUTRAL = 0.5;
    private boolean transferActive = false;

    // Scale factor for the second-stage intake (intake2). Reduce by 10% as requested.
    private static final double INTAKE2_SCALE = 1.0;

    private double depositTargetVelocity = RandyButterNubs.DEFAULT_DEPOSIT_VELOCITY;
    private boolean depositActive = false;
    private boolean lastXButtonState = false;
    private boolean lastDpadUpState = false;
    private boolean lastDpadDownState = false;
    private boolean lastDpadLeftState = false;
    private boolean lastDpadRightState = false;

    // Panels-tunable software feedforward factor (fractional). Keep PIDF constants unchanged.
    public static double DEPOSIT_FF_FACTOR = 0.05; // default small multiplicative FF
    // Panels-tunable absolute feedforward boost in ticks/sec (additive)
    public static double DEPOSIT_FF_BOOST_TICKS = 200.0; // default additive boost (ticks/sec)
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

    @Override
    public void init() {
        robot = new TrowelHardware(hardwareMap);
        drive = new RandyButterNubs(robot.frontLeft, robot.frontRight, robot.backLeft, robot.backRight);

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
        if (depositActive && depositSpinupEndTime == -1) {
            robot.setDepositVelocity(depositTargetVelocity);
        }
        // ==============================================================

        if (odometryEnabled && robot.pinpoint != null) {
            robot.updatePinpoint();
            if (odometry == null) odometry = new Odometry(robot, robot.pinpoint);
            odometry.update();
        } else if (odometry == null) {
            // create odometry with robot so IMU fallback works even if pinpoint is not initialized
            odometry = new Odometry(robot, null);
            // do not call update here until pinpoint or IMU is available
        }

        if (visionEnabled && visionLocalization != null) {
            visionLocalization.update();
        }

        int targetTagId = (selectedTeam == Team.BLUE) ? BLUE_TAG_ID :
                          (selectedTeam == Team.RED) ? RED_TAG_ID : -1;

        autoTurnEnabled = gamepad1.left_bumper;

        double forward = -gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;
        double rotate = gamepad1.right_stick_x;

        double autoRotate = 0.0;
        boolean tagVisible = false;
        double yawToTag = Double.NaN;
        double rangeToTag = Double.NaN;

        // Vision-based auto-aim (hold LB): prefer bearing (accounts for tag orientation) but fall back to yaw.
        if (autoTurnEnabled && visionEnabled && visionLocalization != null && targetTagId > 0) {
            yawToTag = visionLocalization.getYawToTag(targetTagId); // fallback (signed degrees)
            double bearing = visionLocalization.getBearingToTag(targetTagId); // preferred: robot-relative bearing
            rangeToTag = visionLocalization.getRangeToTag(targetTagId);
            double rawError;
            boolean haveBearing = !Double.isNaN(bearing);
            if (haveBearing) rawError = bearing; else rawError = yawToTag;

            // Apply team-specific hard aim shift: Blue -> 7° left, Red -> +5° right
            double appliedShiftDeg = HARD_AIM_SHIFT_DEG;
            if (selectedTeam == Team.BLUE) {
                appliedShiftDeg = 2.0;
            } else if (selectedTeam == Team.RED) {
                appliedShiftDeg = -0.5;
            } else {
                appliedShiftDeg = HARD_AIM_SHIFT_DEG;
            }
            rawError += appliedShiftDeg;
            tagVisible = !Double.isNaN(rawError);

            // Only rotate if tag is visible and error is valid
            if (tagVisible) {
                double error = rawError;
                int errorSign = (error > 0) ? 1 : (error < 0) ? -1 : 0;
                // Prevent oscillation: if error sign changes, stop rotating
                if (aimLastErrorSign != 0 && errorSign != aimLastErrorSign) {
                    autoRotate = 0.0;
                } else if (Math.abs(error) <= SNAP_DEADZONE_DEG) {
                    autoRotate = 0.0;
                } else {
                    long nowT = System.currentTimeMillis();
                    double dt = (aimLastTime > 0) ? (nowT - aimLastTime) / 1000.0 : 0.0;
                    double derivative = 0.0;
                    if (dt > 0) derivative = (error - aimLastError) / dt;
                    double p = SNAP_KP * error + SNAP_KD * derivative;
                    if (p > SNAP_MAX_POWER) p = SNAP_MAX_POWER;
                    if (p < -SNAP_MAX_POWER) p = -SNAP_MAX_POWER;
                    autoRotate = -p;
                }
                aimLastError = error;
                aimLastTime = System.currentTimeMillis();
                aimLastErrorSign = errorSign;
            } else {
                autoRotate = 0.0;
                aimLastErrorSign = 0;
            }
        } else {
            aimLastError = 0.0;
            aimLastTime = 0;
            aimLastErrorSign = 0;
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
        drive.drive(forward, strafe, finalRotate, gamepad1.left_bumper, gamepad1.right_bumper);

        // DRIVER1 transfer + intake override: holding gamepad1 triggers runs intakes + sets transfer servos
        boolean driver1IntakeIn = gamepad1.left_trigger > 0.1;
        boolean driver1IntakeOut = gamepad1.right_trigger > 0.1;

        if (driver1IntakeIn) {
            // Driver1 ZL: run intakes IN and set transfers to IN
            setTransferIn();
            transferActive = true;
            if (robot.intake1 != null) robot.intake1.setPower(1.0);
            if (robot.intake2 != null) robot.intake2.setPower(-1.0 * INTAKE2_SCALE);
        } else if (driver1IntakeOut) {
            // Driver1 ZR: run intakes OUT (reverse) and set transfers to OUT
            setTransferOut();
            transferActive = true;
            if (robot.intake1 != null) robot.intake1.setPower(-1.0);
            if (robot.intake2 != null) robot.intake2.setPower(-1.0 * INTAKE2_SCALE);
        } else {
            // No driver1 override - transfers go neutral and restore gamepad2 controls (unchanged)
            setTransferNeutral();
            transferActive = false;

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
                    robot.intake2.setPower(1.0 * INTAKE2_SCALE);
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

        telemetry.addData("=== TEAM & AUTO-TURN ===", "");
        String teamStr = (selectedTeam == Team.BLUE) ? "BLUE" :
                         (selectedTeam == Team.RED) ? "RED" : "NONE";
        telemetry.addData("Team", "%s (Target Tag: %d)", teamStr, targetTagId);
        telemetry.addData("Auto-Turn", autoTurnEnabled ? "ACTIVE (Hold LB)" : "Hold LB to turn towards tag");
        if (autoTurnEnabled && targetTagId > 0) {
            telemetry.addData("Tag Visible", tagVisible ? "YES" : "NO");
            if (tagVisible) {
                telemetry.addData("Yaw to Tag", "%.1f deg", yawToTag);
                telemetry.addData("Range to Tag", "%.1f in", rangeToTag);
                telemetry.addData("Auto-Rotate Power", "%.2f", autoRotate);
            }
        }

        telemetry.addData("Final Rotate (driver+auto)", "%.2f", finalRotate);

        telemetry.addData("=== DRIVER 1 ===", "");
        telemetry.addData("Drive", "Forward: %.2f, Strafe: %.2f, Rotate: %.2f", forward, strafe, finalRotate);
        telemetry.addData("Transfer Active", transferActive);
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

    // Normalize angle difference to range (-180,180]
    private double normalizeAngleDeg(double angle) {
        double a = angle;
        while (a > 180.0) a -= 360.0;
        while (a <= -180.0) a += 360.0;
        return a;
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
