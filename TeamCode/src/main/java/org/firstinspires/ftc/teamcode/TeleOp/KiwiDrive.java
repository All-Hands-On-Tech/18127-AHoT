package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Disabled
@TeleOp(name="Basic Kiwi Drive", group="B")
public class KiwiDrive extends LinearOpMode {

    // Declare OpMode members.
    private DcMotor motorFront;
    private DcMotor motorLeft;
    private DcMotor motorRight;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        motorFront = hardwareMap.get(DcMotor.class, "front");
        motorLeft = hardwareMap.get(DcMotor.class, "left");
        motorRight = hardwareMap.get(DcMotor.class, "right");

        motorFront.setDirection(DcMotor.Direction.FORWARD);
        motorLeft.setDirection(DcMotor.Direction.FORWARD);
        motorRight.setDirection(DcMotor.Direction.FORWARD);

        waitForStart();

        while (opModeIsActive()) {
            double driveX = gamepad1.left_stick_x * -1;
            double driveY = gamepad1.left_stick_y *  1;
            double turn   = gamepad1.right_stick_x * -0.6;

            double powerFront = driveX+turn;
            double powerLeft = -0.5*driveX + 0.866*driveY + turn;
            double powerRight = -0.5*driveX - 0.866*driveY + turn;
            double max = Math.max(Math.abs(powerFront), Math.max(Math.abs(powerLeft), Math.abs(powerRight)));

            if (max<1) {max = 1;}

            telemetry.addData("Power Front: ", powerFront);
            telemetry.addData("Power Left: ", powerLeft);
            telemetry.addData("Power Right: ", powerRight);

            motorFront.setPower(powerFront/max);
            motorLeft.setPower(powerLeft/max);
            motorRight.setPower(powerRight/max);

            if (gamepad1.y) {
                motorFront.setPower(0.75);
            }
            if (gamepad1.x) {
                motorLeft.setPower(0.75);
            }
            if (gamepad1.b) {
                motorRight.setPower(0.75);
            }

            telemetry.update();
        }
    }
}
