package org.firstinspires.ftc.teamcode.Examples;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Simple ElapsedTime Example - Drive Forward
 */
@Autonomous(name = "ElapsedTime Example", group = "Examples")
public class ElapsedTimeExample extends LinearOpMode {

    // Drive motors
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;

    // Timer
    private ElapsedTime timer = new ElapsedTime();

    // Constants - adjust these!
    private static final double DRIVE_POWER = 0.5;
    private static final double DRIVE_TIME_S = 2.0; // Drive for 2 seconds

    @Override
    public void runOpMode() {
        // Initialize motors
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");

        // Set directions (adjust based on your robot)
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);

        telemetry.addLine("Ready to drive forward!");
        telemetry.update();

        waitForStart();

        // Reset timer and start driving
        timer.reset();

        // Drive forward while time < DRIVE_TIME_S
        while (opModeIsActive() && timer.seconds() < DRIVE_TIME_S) {
            frontLeft.setPower(DRIVE_POWER);
            frontRight.setPower(DRIVE_POWER);
            backLeft.setPower(DRIVE_POWER);
            backRight.setPower(DRIVE_POWER);

            telemetry.addData("Driving", "%.2f / %.2f seconds", timer.seconds(), DRIVE_TIME_S);
            telemetry.update();
        }

        // Stop motors
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);

        telemetry.addLine("Done!");
        telemetry.update();
    }
}

