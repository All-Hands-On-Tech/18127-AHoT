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
        wheelA.setDirection(DcMotor.Direction.REVERSE);
        wheelB.setDirection(DcMotor.Direction.REVERSE);
        wheelC.setDirection(DcMotor.Direction.REVERSE);

        telemetry.addLine("Triangle Drive Ready");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            // Get driver inputs
            double forward = -gamepad1.left_stick_y;  // Forward/backward (y-axis)
            double strafe = gamepad1.left_stick_x;    // Left/right (x-axis)
            double rotate = -gamepad1.right_stick_x;  // Rotation (inverted for correct direction)

            // Robot-centric triangle holonomic drive kinematics
            // Wheels arranged in equilateral triangle (120° apart):
            // Wheel A: Front (0° from robot front)
            // Wheel B: Back-right (240° from robot front)
            // Wheel C: Back-left (120° from robot front)

            // Inverse kinematics for kiwi/triangle drive
            // Each wheel velocity = dot product of wheel vector with desired velocity + rotation
            // Using proper 120° spacing for balanced omnidirectional movement

            double powerA = forward - rotate;
            double powerB = (-0.5 * forward) - (0.866 * strafe) - rotate;
            double powerC = (-0.5 * forward) + (0.866 * strafe) - rotate;

            // Calculate max power before normalization for telemetry
            double maxRaw = Math.max(Math.abs(powerA), Math.max(Math.abs(powerB), Math.abs(powerC)));

            // Normalize powers to keep within [-1, 1] range while preserving ratios
            double max = maxRaw;
            if (max > 1.0) {
                powerA /= max;
                powerB /= max;
                powerC /= max;
            }

            // Set motor powers
            wheelA.setPower(powerA);
            wheelB.setPower(powerB);
            wheelC.setPower(powerC);

            // Functional telemetry
            telemetry.addData("Status", "ACTIVE");
            telemetry.addData("Loop Time (ms)", "%.1f", getRuntime() * 1000);
            telemetry.addLine();
            telemetry.addData("Input | Fwd", "%.3f", forward);
            telemetry.addData("Input | Str", "%.3f", strafe);
            telemetry.addData("Input | Rot", "%.3f", rotate);
            telemetry.addLine();
            telemetry.addData("Power | A", "%.3f", powerA);
            telemetry.addData("Power | B", "%.3f", powerB);
            telemetry.addData("Power | C", "%.3f", powerC);
            telemetry.addData("Max Raw", "%.3f", maxRaw);
            telemetry.addData("Normalized", max > 1.0 ? "YES" : "NO");
            telemetry.update();
        }
    }
}
