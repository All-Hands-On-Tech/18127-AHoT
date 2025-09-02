package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "Two Wheel Odometry TeleOp", group = "TeleOp")
public class TwoWheelOdometryTeleop extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        // Declare our motors
        // Make sure your ID's match your configuration
        DcMotor frontLeftMotor = hardwareMap.get(DcMotor.class, "frontLeft");
        DcMotor backLeftMotor = hardwareMap.get(DcMotor.class, "backLeft");
        DcMotor frontRightMotor = hardwareMap.get(DcMotor.class, "frontRight");
        DcMotor backRightMotor = hardwareMap.get(DcMotor.class, "backRight");

        // Reverse the right side motors
        // Reverse left motors if you are using NeveRests
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            // Invert joystick axes for correct mecanum directions
            double y = Math.abs(gamepad1.left_stick_y) > 0.05 ? gamepad1.left_stick_y : 0;    // Forward/backward
            double x = Math.abs(gamepad1.left_stick_x) > 0.05 ? -gamepad1.left_stick_x : 0;   // Left/right strafe
            double rx = Math.abs(gamepad1.right_stick_x) > 0.05 ? -gamepad1.right_stick_x : 0; // Rotation

            double speedMultiplier = gamepad1.left_bumper ? 0.5 : 1.0;
            y *= speedMultiplier;
            x *= speedMultiplier;
            rx *= speedMultiplier;

            // --- ROBOT-CENTRIC MECANUM DRIVE ---
            x *= 1.1; // Counteract imperfect strafing

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);

            telemetry.addData("Joystick", "Y: %.2f X: %.2f RX: %.2f", y, x, rx);
            telemetry.addData("FL", "%.2f", frontLeftPower);
            telemetry.addData("BL", "%.2f", backLeftPower);
            telemetry.addData("FR", "%.2f", frontRightPower);
            telemetry.addData("BR", "%.2f", backRightPower);
            telemetry.update();
        }
    }
}
