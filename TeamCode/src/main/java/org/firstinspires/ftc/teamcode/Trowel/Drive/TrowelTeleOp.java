package org.firstinspires.ftc.teamcode.Trowel.Drive;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Trowel.Configs.RandyButterNubs;
import org.firstinspires.ftc.teamcode.common.Odometry;
import org.firstinspires.ftc.teamcode.common.VisionLocalization;
import org.firstinspires.ftc.teamcode.Trowel.Configs.TrowelHardware;

/**
 * TeleOp program for Trowel robot
 *
 * DRIVER 1 (Gamepad1):
 * - Drive: Left joystick forward/back, Right joystick strafe & rotate
 *
 * DRIVER 2 (Gamepad2):
 * - Transfer: A button toggles transfer servos
 * - Intake 1 (First Stage): ZL (intake) and ZR (outtake)
 * - Intake 2 (Second Stage): B (intake) and Y (outtake)
 * - Deposit: X to toggle deposit to set ticks
 * - Deposit Adjustment: Dpad Up/Down (±1 tick), Dpad Left/Right (±10 ticks)
 * - Default deposit: 700 ticks
 */
@TeleOp(name = "Randy Butter Knubs", group = "Trowel")
public class TrowelTeleOp extends OpMode {

    // Hardware
    private TrowelHardware robot;

    // Drive system
    private RandyButterNubs drive;

    // Odometry and Vision
    private Odometry odometry;
    private VisionLocalization visionLocalization;
    private boolean visionEnabled = false;
    private boolean odometryEnabled = false;

    // Servo positions
    private static final double TRANSFER_IN = 0.0;   // Position when pressing LB
    private static final double TRANSFER_OUT = 1.0;  // Position when pressing RB
    private static final double TRANSFER_NEUTRAL = 0.5;  // Neutral position when no button pressed
    private boolean transferActive = false;

    // Deposit control
    private double depositTargetVelocity = 1800.0;  // Default deposit velocity (ticks per second)
    private boolean depositActive = false;
    private boolean lastXButtonState = false;
    private boolean lastDpadUpState = false;
    private boolean lastDpadDownState = false;
    private boolean lastDpadLeftState = false;
    private boolean lastDpadRightState = false;

    @Override
    public void init() {
        // Initialize robot hardware
        robot = new TrowelHardware(hardwareMap);

        // Initialize drive system
        drive = new RandyButterNubs(robot.frontLeft, robot.frontRight, robot.backLeft, robot.backRight);

        // Initialize Pinpoint for odometry
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

        // Initialize Vision Localization
        try {
            visionLocalization = new VisionLocalization(hardwareMap);
            visionEnabled = true;
            telemetry.addLine("Vision Localization Enabled");
        } catch (Exception e) {
            telemetry.addLine("Vision Not Available - Vision Disabled");
            visionEnabled = false;
        }

        // Set initial servo positions
        if (robot.transfer1 != null) robot.transfer1.setPosition(TRANSFER_OUT);
        if (robot.transfer2 != null) robot.transfer2.setPosition(TRANSFER_OUT);

        // Reset deposit encoders
        robot.resetDepositEncoders();

        telemetry.addLine("Trowel TeleOp Initialized!");
        telemetry.update();
    }

    @Override
    public void loop() {
        // Update odometry
        if (odometryEnabled && robot.pinpoint != null) {
            robot.updatePinpoint();
            odometry = new Odometry(null, robot.pinpoint);
            odometry.update();
        }

        // ===== DRIVER 1 CONTROLS =====
        // Drive controls - left joystick forward/back, right joystick strafe, right trigger rotate
        double forward = -gamepad1.left_stick_y;  // Negate for intuitive forward
        double strafe = gamepad1.left_stick_x;
        double rotate = gamepad1.right_stick_x;

        drive.drive(forward, strafe, -rotate);

        // Transfer servo controls - ZL moves one direction, ZR moves the other (DRIVER 1)
        if (gamepad1.left_trigger > 0.1) {
            // ZL - Transfer IN position
            if (robot.transfer1 != null) robot.transfer1.setPosition(TRANSFER_IN);
            if (robot.transfer2 != null) robot.transfer2.setPosition(TRANSFER_IN);
            transferActive = true;
        } else if (gamepad1.right_trigger > 0.1) {
            // ZR - Transfer OUT position
            if (robot.transfer1 != null) robot.transfer1.setPosition(TRANSFER_OUT);
            if (robot.transfer2 != null) robot.transfer2.setPosition(TRANSFER_OUT);
            transferActive = true;
        } else {
            // No trigger pressed - neutral position
            if (robot.transfer1 != null) robot.transfer1.setPosition(TRANSFER_NEUTRAL);
            if (robot.transfer2 != null) robot.transfer2.setPosition(TRANSFER_NEUTRAL);
            transferActive = false;
        }

        // ===== DRIVER 2 CONTROLS =====
        // Intake 1 (First Stage) - ZL for intake, ZR for outtake (DRIVER 2)
        if (robot.intake1 != null) {
            if (gamepad2.left_trigger > 0.1) {
                // ZL - Intake (full power forward)
                robot.intake1.setPower(1.0);
            } else if (gamepad2.right_trigger > 0.1) {
                // ZR - Outtake (full power reverse)
                robot.intake1.setPower(-1.0);
            } else {
                // No input - stop
                robot.intake1.setPower(0.0);
            }
        }

        // Intake 2 (Second Stage) - A for intake, B for outtake (DRIVER 2)
        if (robot.intake2 != null) {
            if (gamepad2.a) {
                // A - Intake (full power forward)
                robot.intake2.setPower(1.0);
            } else if (gamepad2.b) {
                // B - Outtake (full power reverse)
                robot.intake2.setPower(-1.0);
            } else {
                // No input - stop
                robot.intake2.setPower(0.0);
            }
        }

        // Deposit controls - X to toggle deposit at set velocity (DRIVER 2)
        if (gamepad2.x && !lastXButtonState) {
            depositActive = !depositActive;
            if (depositActive) {
                robot.setDepositVelocity(depositTargetVelocity);
            } else {
                robot.stopDeposit();
            }
        }
        lastXButtonState = gamepad2.x;

        // Deposit velocity adjustment with Dpad (DRIVER 2)
        // Dpad Up: +5 velocity
        if (gamepad2.dpad_up && !lastDpadUpState) {
            depositTargetVelocity += 5;
            if (depositActive) {
                robot.setDepositVelocity(depositTargetVelocity);
            }
        }
        lastDpadUpState = gamepad2.dpad_up;

        // Dpad Down: -5 velocity
        if (gamepad2.dpad_down && !lastDpadDownState) {
            depositTargetVelocity -= 5;
            if (depositTargetVelocity < 0) depositTargetVelocity = 0;
            if (depositActive) {
                robot.setDepositVelocity(depositTargetVelocity);
            }
        }
        lastDpadDownState = gamepad2.dpad_down;

        // Dpad Left: -50 velocity
        if (gamepad2.dpad_left && !lastDpadLeftState) {
            depositTargetVelocity -= 50;
            if (depositTargetVelocity < 0) depositTargetVelocity = 0;
            if (depositActive) {
                robot.setDepositVelocity(depositTargetVelocity);
            }
        }
        lastDpadLeftState = gamepad2.dpad_left;

        // Dpad Right: +50 velocity
        if (gamepad2.dpad_right && !lastDpadRightState) {
            depositTargetVelocity += 50;
            if (depositActive) {
                robot.setDepositVelocity(depositTargetVelocity);
            }
        }
        lastDpadRightState = gamepad2.dpad_right;

        // Telemetry
        telemetry.addData("=== DRIVER 1 ===", "");
        telemetry.addData("Drive", "Forward: %.2f, Strafe: %.2f, Rotate: %.2f", forward, strafe, rotate);
        telemetry.addData("Transfer Active", transferActive);
        if (robot.transfer1 != null && robot.transfer2 != null) {
            telemetry.addData("Transfer Psdkfhjsdfkjhsddfkjhsdfkjhsdfkjhsdfkjhsdfkjhositions", "T1: %.2f, T2: %.2f", robot.transfer1.getPosition(), robot.transfer2.getPosition());
        }

        telemetry.addData("=== DRIVER 2 ===", "");
        if (robot.intake1 != null) {
            telemetry.addData("Intake 1 Power (ZL/ZR)", "%.2f", robot.intake1.getPower());
        }
        if (robot.intake2 != null) {
            telemetry.addData("Intake 2 Power (B/Y)", "%.2f", robot.intake2.getPower());
        }
        telemetry.addData("Deposit", "Active: %s, Target Velocity: %.1f", depositActive ? "YES" : "NO", depositTargetVelocity);
        if (robot.deposit1 != null && robot.deposit2 != null) {
            telemetry.addData("Deposit Power", "D1: %.2f, D2: %.2f", robot.deposit1.getPower(), robot.deposit2.getPower());
        }

        // Display odometry data
        if (odometryEnabled && odometry != null) {
            Odometry.Position pos = odometry.getPosition();
            telemetry.addData("=== ODOMETRY ===", "");
            telemetry.addData("Position", pos);
        } else {
            telemetry.addData("Odometry", "Disabled");
        }

        // Display motor powers
        telemetry.addLine(robot.getMotorPowers());

        // Display motor configurations
        telemetry.addLine(robot.getMotorConfigurations());

        // Display hardware initialization status
        telemetry.addLine(robot.getInitializationStatus());

        telemetry.update();
    }

    @Override
    public void stop() {
        drive.stop();
        robot.stop();
    }
}

