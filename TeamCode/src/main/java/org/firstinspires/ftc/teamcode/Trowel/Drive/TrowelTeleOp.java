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

    public static double AUTO_TURN_SPEED = 0.45;
    public static double YAW_THRESHOLD = 2.0;
    public static double HEADING_RESET_SPEED = 0.4;
    public static double HEADING_THRESHOLD = 2.0;
    // PD controller settings for vision auto-aim (tuned conservative)
    public static double SNAP_KP = 0.045;      // much higher proportional gain for strong/fast aiming
    public static double SNAP_KD = 0.008;      // slightly higher derivative gain for stability
    public static double SNAP_MAX_POWER = 0.45; // much higher max rotation power for aggressive aiming
    public static double SNAP_DEADZONE_DEG = 0.5; // much smaller deadzone for tighter aiming
    // Hardcoded shift: always treat tag as 2° more to the right (positive = right)
    public static final double HARD_AIM_SHIFT_DEG = 2.0;
    private double aimLastError = 0.0;
    private long aimLastTime = 0;
    private boolean autoTurnEnabled = false;

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

    private double depositTargetVelocity = RandyButterNubs.DEFAULT_DEPOSIT_VELOCITY;
    private boolean depositActive = false;
    private boolean lastXButtonState = false;
    private boolean lastDpadUpState = false;
    private boolean lastDpadDownState = false;
    private boolean lastDpadLeftState = false;
    private boolean lastDpadRightState = false;

    @Override
    public void init() {
        robot = new TrowelHardware(hardwareMap);
        drive = new RandyButterNubs(robot.frontLeft, robot.frontRight, robot.backLeft, robot.backRight);

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
            // choose bearing if valid
            double rawError;
            boolean haveBearing = !Double.isNaN(bearing);
            if (haveBearing) rawError = bearing; else rawError = yawToTag;
            // Hardcode a +2 degree rightward shift for all aiming
            rawError += 2.0;
            tagVisible = !Double.isNaN(rawError);

            if (tagVisible) {
                double error = rawError; // signed degrees: + means tag is to robot's right

                // telemetry for debugging
                telemetry.addData("Vision Source", haveBearing ? "bearing" : "yaw");
                if (haveBearing) telemetry.addData("Bearing (deg)", "%.2f", bearing);
                telemetry.addData("Yaw (deg)", "%.2f", yawToTag);
                telemetry.addData("Hard Aim Shift (deg)", "%.2f", HARD_AIM_SHIFT_DEG);

                // deadzone
                if (Math.abs(error) <= SNAP_DEADZONE_DEG) {
                    autoRotate = 0.0;
                } else {
                    long nowT = System.currentTimeMillis();
                    double dt = (aimLastTime > 0) ? (nowT - aimLastTime) / 1000.0 : 0.0;
                    double derivative = 0.0;
                    if (dt > 0) derivative = (error - aimLastError) / dt;
                    double p = SNAP_KP * error + SNAP_KD * derivative;
                    // clamp
                    if (p > SNAP_MAX_POWER) p = SNAP_MAX_POWER;
                    if (p < -SNAP_MAX_POWER) p = -SNAP_MAX_POWER;
                    // p is signed (positive means tag is to the right). RandyButterNubs.drive expects positive rotate = clockwise.
                    // If p has opposite sign, invert here. Empirically invert to correct direction.
                    autoRotate = -p;
                    aimLastError = error;
                    aimLastTime = nowT;
                }
            }
        } else {
            // not auto aiming: clear integrator state
            aimLastError = 0.0;
            aimLastTime = 0;
        }

        // NOTE: autoRotate sign chosen to match drive.rotate sign convention

        // Speed scaling: LB = 70% slow, RB = 30% slow. These scale translational and driver rotate input
        double speedScale = 1.0;
        if (gamepad1.left_bumper) speedScale = 0.7; // 70% slow mode
        else if (gamepad1.right_bumper) speedScale = 0.3; // 30% slow mode

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
            if (robot.transfer1 != null) robot.transfer1.setPosition(TRANSFER_IN);
            if (robot.transfer2 != null) robot.transfer2.setPosition(TRANSFER_IN);
            transferActive = true;
            if (robot.intake1 != null) robot.intake1.setPower(1.0);
            if (robot.intake2 != null) robot.intake2.setPower(-1.0);
        } else if (driver1IntakeOut) {
            // Driver1 ZR: run intakes OUT (reverse) and set transfers to OUT
            if (robot.transfer1 != null) robot.transfer1.setPosition(TRANSFER_OUT);
            if (robot.transfer2 != null) robot.transfer2.setPosition(TRANSFER_OUT);
            transferActive = true;
            if (robot.intake1 != null) robot.intake1.setPower(-1.0);
            if (robot.intake2 != null) robot.intake2.setPower(-1.0);
        } else {
            // No driver1 override - transfers go neutral and restore gamepad2 controls (unchanged)
            if (robot.transfer1 != null) robot.transfer1.setPosition(TRANSFER_NEUTRAL);
            if (robot.transfer2 != null) robot.transfer2.setPosition(TRANSFER_NEUTRAL);
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
                    robot.intake2.setPower(1.0);
                } else if (gamepad2.b) {
                    robot.intake2.setPower(-1.0);
                } else {
                    robot.intake2.setPower(0.0);
                }
            }
        }

        if (gamepad2.x && !lastXButtonState) {
            depositActive = !depositActive;
            if (depositActive) {
                robot.setDepositVelocity(depositTargetVelocity);
            } else {
                robot.stopDeposit();
            }
        }
        lastXButtonState = gamepad2.x;

        if (gamepad2.dpad_up && !lastDpadUpState) {
            depositTargetVelocity += 5;
            if (depositActive) {
                robot.setDepositVelocity(depositTargetVelocity);
            }
        }
        lastDpadUpState = gamepad2.dpad_up;

        if (gamepad2.dpad_down && !lastDpadDownState) {
            depositTargetVelocity -= 5;
            if (depositTargetVelocity < 0) depositTargetVelocity = 0;
            if (depositActive) {
                robot.setDepositVelocity(depositTargetVelocity);
            }
        }
        lastDpadDownState = gamepad2.dpad_down;

        if (gamepad2.dpad_left && !lastDpadLeftState) {
            depositTargetVelocity -= 50;
            if (depositTargetVelocity < 0) depositTargetVelocity = 0;
            if (depositActive) {
                robot.setDepositVelocity(depositTargetVelocity);
            }
        }
        lastDpadLeftState = gamepad2.dpad_left;

        if (gamepad2.dpad_right && !lastDpadRightState) {
            depositTargetVelocity += 50;
            if (depositActive) {
                robot.setDepositVelocity(depositTargetVelocity);
            }
        }
        lastDpadRightState = gamepad2.dpad_right;

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
        } else {
            telemetry.addData("Vision", "Disabled");
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

}
