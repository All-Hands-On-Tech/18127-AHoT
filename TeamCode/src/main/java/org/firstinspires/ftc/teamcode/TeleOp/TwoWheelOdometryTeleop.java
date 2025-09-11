/*
package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.common.RobotHardware;
import org.firstinspires.ftc.teamcode.common.RobotConfig;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name = "Two Wheel Odometry TeleOp", group = "TeleOp")
public class TwoWheelOdometryTeleop extends LinearOpMode {
    private RobotHardware hw;
    private RobotConfig config;

    // Odometry variables
    private double x = 0, y = 0, heading = 0;
    private int prevParallel = 0, prevPerpendicular = 0;

    @Override
    public void runOpMode() {
        hw = new RobotHardware(hardwareMap);
        config = new RobotConfig();

        // Reset odometry encoders
        resetOdometry();

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            // Update odometry position
            updateOdometry();

            // Drive controls
            double y_stick = Math.abs(gamepad1.left_stick_y) > 0.05 ? gamepad1.left_stick_y : 0;
            double x_stick = Math.abs(gamepad1.left_stick_x) > 0.05 ? -gamepad1.left_stick_x : 0;
            double rx = Math.abs(gamepad1.right_stick_x) > 0.05 ? -gamepad1.right_stick_x : 0;

            double speedMultiplier = gamepad1.left_bumper ? 0.5 : 1.0;
            y_stick *= speedMultiplier;
            x_stick *= speedMultiplier;
            rx *= speedMultiplier;

            x_stick *= 1.1;

            double denominator = Math.max(Math.abs(y_stick) + Math.abs(x_stick) + Math.abs(rx), 1);
            double frontLeftPower = (y_stick + x_stick + rx) / denominator;
            double backLeftPower = (y_stick - x_stick + rx) / denominator;
            double frontRightPower = (y_stick - x_stick - rx) / denominator;
            double backRightPower = (y_stick + x_stick - rx) / denominator;

            hw.frontLeft.setPower(frontLeftPower);
            hw.backLeft.setPower(backLeftPower);
            hw.frontRight.setPower(frontRightPower);
            hw.backRight.setPower(backRightPower);

            // Reset odometry if gamepad2.start is pressed
            if (gamepad2.start) {
                resetOdometry();
            }

            // Telemetry
            telemetry.addData("=== DRIVE ===", "");
            telemetry.addData("Joystick", "Y: %.2f X: %.2f RX: %.2f", y_stick, x_stick, rx);
            telemetry.addData("FL", "%.2f", frontLeftPower);
            telemetry.addData("BL", "%.2f", backLeftPower);
            telemetry.addData("FR", "%.2f", frontRightPower);
            telemetry.addData("BR", "%.2f", backRightPower);

            telemetry.addData("=== ODOMETRY ===", "");
            telemetry.addData("Position", "X: %.1f mm, Y: %.1f mm", x, y);
            telemetry.addData("Heading", "%.1f degrees", Math.toDegrees(heading));
            telemetry.addData("Encoders", "Par: %d, Perp: %d",
                hw.odoParallel != null ? hw.odoParallel.getCurrentPosition() : 0,
                hw.odoPerpendicular != null ? hw.odoPerpendicular.getCurrentPosition() : 0);
            telemetry.addData("Reset", "Press gamepad2.start to reset position");

            telemetry.update();
        }
    }

    private void updateOdometry() {
        if (hw.odoParallel == null || hw.odoPerpendicular == null) return;

        // Get current encoder positions
        int currParallel = hw.odoParallel.getCurrentPosition();
        int currPerpendicular = hw.odoPerpendicular.getCurrentPosition();

        // Calculate change in encoder positions
        int deltaParallel = currParallel - prevParallel;
        int deltaPerpendicular = currPerpendicular - prevPerpendicular;

        // Convert encoder ticks to millimeters
        double deltaY_robot = (deltaParallel / config.ticksPerRev) * Math.PI * config.wheelDiameterMM;
        double deltaX_robot = (deltaPerpendicular / config.ticksPerRev) * Math.PI * config.wheelDiameterMM;

        // Get current heading from IMU
        if (hw.imu != null) {
            heading = hw.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        }

        // Transform robot-relative motion to field-relative coordinates
        double deltaX_field = deltaX_robot * Math.cos(heading) - deltaY_robot * Math.sin(heading);
        double deltaY_field = deltaX_robot * Math.sin(heading) + deltaY_robot * Math.cos(heading);

        // Update position
        x += deltaX_field;
        y += deltaY_field;

        // Update previous encoder positions
        prevParallel = currParallel;
        prevPerpendicular = currPerpendicular;
    }

    private void resetOdometry() {
        if (hw.odoParallel != null) {
            hw.odoParallel.setMode(com.qualcomm.robotcore.hardware.DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            hw.odoParallel.setMode(com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            prevParallel = hw.odoParallel.getCurrentPosition();
        }

        if (hw.odoPerpendicular != null) {
            hw.odoPerpendicular.setMode(com.qualcomm.robotcore.hardware.DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            hw.odoPerpendicular.setMode(com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            prevPerpendicular = hw.odoPerpendicular.getCurrentPosition();
        }

        if (hw.imu != null) {
            hw.imu.resetYaw();
        }

        // Reset position
        x = 0;
        y = 0;
        heading = 0;
    }
}
*/
