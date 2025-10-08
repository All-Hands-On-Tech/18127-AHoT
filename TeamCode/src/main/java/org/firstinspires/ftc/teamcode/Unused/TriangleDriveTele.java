package org.firstinspires.ftc.teamcode.Unused;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Triangle Drive TeleOp", group = "TeleOp")
public class TriangleDriveTele extends LinearOpMode {
    private DcMotor wheelA, wheelB, wheelC;

    @Override
    public void runOpMode() {
        // Map hardware (update names to match your configuration)
        wheelA = hardwareMap.get(DcMotor.class, "wheelA");
        wheelB = hardwareMap.get(DcMotor.class, "wheelB");
        wheelC = hardwareMap.get(DcMotor.class, "wheelC");

        // Set directions if needed
        wheelA.setDirection(DcMotor.Direction.FORWARD);
        wheelB.setDirection(DcMotor.Direction.FORWARD);
        wheelC.setDirection(DcMotor.Direction.FORWARD);

        telemetry.addLine("Triangle Drive Ready");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            double forward = -gamepad1.left_stick_y; // Forward/back
            double rotate = gamepad1.right_stick_x;  // Rotation

            // Simple drive logic for triangle layout
            // Each wheel gets a mix of forward and rotation
            double powerA = forward + rotate;
            double powerB = forward - rotate;
            double powerC = -forward + rotate;

            // Normalize powers
            double max = Math.max(Math.abs(powerA), Math.max(Math.abs(powerB), Math.abs(powerC)));
            if (max > 1.0) {
                powerA /= max;
                powerB /= max;
                powerC /= max;
            }

            wheelA.setPower(powerA);
            wheelB.setPower(powerB);
            wheelC.setPower(powerC);

            telemetry.addData("WheelA Power", powerA);
            telemetry.addData("WheelB Power", powerB);
            telemetry.addData("WheelC Power", powerC);
            telemetry.update();
        }
    }
}

