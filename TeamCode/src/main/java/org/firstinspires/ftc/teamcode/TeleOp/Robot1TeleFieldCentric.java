package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.common.RobotHardware;
import org.firstinspires.ftc.teamcode.common.RobotConfig;
import org.firstinspires.ftc.teamcode.common.Odometry;
import org.firstinspires.ftc.teamcode.common.PanelsPublisher;

@TeleOp(name = "Two Wheel Odometry TeleOp FieldCentric", group = "TeleOp")
public class Robot1TeleFieldCentric extends LinearOpMode {
    @Override
    public void runOpMode() {
        RobotHardware hw = new RobotHardware(hardwareMap);
        RobotConfig config = new RobotConfig();
        hw.initPinpoint();
        PanelsPublisher panels = new PanelsPublisher();
        panels.init();

        telemetry.addLine("Init complete - waiting start");
        telemetry.addLine("DRIVER 1: Chassis control");
        telemetry.addLine("DRIVER 2: Mechanisms (intake, cam, sweeper, deposit)");
        telemetry.addData("Pinpoint", hw.pinpoint == null ? "NOT FOUND (expect name '"+config.pinpointName+"')" : hw.pinpoint.getDeviceStatus());
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;
        double lastLoopTime = getRuntime();

        Odometry odometry = new Odometry(hw, hw.pinpoint);

        // Set initial servo positions - cam starts at 0.46 (clamped between 0.4722 and 0.5122)
        if (hw.cam != null) hw.cam.setPosition(0.46);
        if (hw.transferServo != null) hw.transferServo.setPosition(0.5); // Standard servo, neutral position

        boolean prevComboReset = false;
        boolean prevComboRecalibrate = false;

        while (opModeIsActive()) {
            double now = getRuntime();
            double dt = Math.max(0.0, Math.min(0.1, now - lastLoopTime));
            lastLoopTime = now;

            hw.updatePinpoint();
            odometry.update();
            Odometry.Position pos = odometry.getPosition();

            // ===== DRIVER 1: CHASSIS CONTROL =====
            // Get joystick values with proper deadzone
            double y = Math.abs(gamepad1.left_stick_y) > 0.05 ? -gamepad1.left_stick_y : 0;
            double x = Math.abs(gamepad1.left_stick_x) > 0.05 ? gamepad1.left_stick_x : 0;
            double r = Math.abs(gamepad1.right_stick_x) > 0.05 ? gamepad1.right_stick_x : 0;

            double speedMul = gamepad1.left_bumper ? 0.5 : 1.0;
            y *= speedMul;
            x *= speedMul;
            r *= speedMul;
            x *= 1.1;

            // Apply cubic response curve
            y = Math.copySign(y * y * y, y);
            x = Math.copySign(x * x * x, x);
            r = Math.copySign(r * r * r, r);

            // Field-centric transformation
            double headingRad = pos.getHeadingRad();
            double tempX = x * Math.cos(-headingRad) - y * Math.sin(-headingRad);
            double tempY = x * Math.sin(-headingRad) + y * Math.cos(-headingRad);
            x = tempX;
            y = tempY;

            // Correct mecanum drive formulas
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(r), 1);
            double fl = (y + x + r) / denominator;
            double bl = (y - x + r) / denominator;
            double fr = (y - x - r) / denominator;
            double br = (y + x - r) / denominator;
            hw.setDrivePowers(fl, fr, bl, br);

            // Driver 1: Safe combos (pinpoint reset / recalibrate)
            boolean comboReset = gamepad1.start && gamepad1.back;
            if (comboReset && !prevComboReset) {
                if (hw.pinpoint != null) hw.pinpoint.resetPosAndIMU();
                else if (hw.imu != null) hw.imu.resetYaw();
            }
            prevComboReset = comboReset;

            boolean comboRecalibrate = gamepad1.x && gamepad1.y;
            if (comboRecalibrate && !prevComboRecalibrate && hw.pinpoint != null) hw.pinpoint.recalibrateIMU();
            prevComboRecalibrate = comboRecalibrate;

            // ===== DRIVER 2: MECHANISM CONTROL =====
            // Intake (triggers, both => stop)
            boolean rt = gamepad2.right_trigger > 0.1;
            boolean lt = gamepad2.left_trigger > 0.1;
            double intakePower = (rt && lt) ? 0.0 : rt ? 0.9 : lt ? -0.9 : 0.0;
            if (hw.intakeMotor != null) hw.intakeMotor.setPower(intakePower);

            // Launcher (X) velocity control
            double launcherVelocityTarget = gamepad2.x ? 10000.0 : 0.0;
            if (hw.depositMotorL != null) hw.depositMotorL.setVelocity(launcherVelocityTarget);
            if (hw.depositMotorR != null) hw.depositMotorR.setVelocity(-launcherVelocityTarget);

            // Cam (LB/RB) continuous hold-based control - always uses current position + step
            if (hw.cam != null) {
                double camPosition = hw.cam.getPosition(); // Get actual current position from servo

                // Apply step based on current input (prioritizes latest input)
                if (gamepad2.left_bumper) {
                    camPosition -= 0.001; // Move in
                }
                if (gamepad2.right_bumper) {
                    camPosition += 0.001; // Move out
                }

                // Clamp between 0.4722 and 0.5122
                camPosition = Math.max(0.4722, Math.min(0.5122, camPosition));
                hw.cam.setPosition(camPosition);
            }

            // Transfer servo - standard servo position control (not continuous)
            // Y button removed, no transfer servo control for now
            // (Add button mappings here if you want position-based control)

            // Telemetry - each stat on its own line
            telemetry.addData("Left Stick Y Raw", gamepad1.left_stick_y);
            telemetry.addData("Left Stick X Raw", gamepad1.left_stick_x);
            telemetry.addData("Right Stick X Raw", gamepad1.right_stick_x);
            telemetry.addData("Drive Y", y);
            telemetry.addData("Drive X", x);
            telemetry.addData("Drive R", r);
            telemetry.addData("LB Slow Mode", gamepad1.left_bumper ? "ACTIVE" : "off");
            telemetry.addData("Start+Back Reset", comboReset ? "PRESSED" : "");
            telemetry.addData("X+Y Recalibrate", comboRecalibrate ? "PRESSED" : "");

            telemetry.addData("Cam LB (IN)", gamepad2.left_bumper ? "HOLDING" : "--");
            telemetry.addData("Cam RB (OUT)", gamepad2.right_bumper ? "HOLDING" : "--");
            telemetry.addData("Cam Position", hw.cam != null ? String.format("%.4f", hw.cam.getPosition()) : "N/A");
            telemetry.addData("Cam Range", "0.4722 to 0.5122");

            telemetry.addData("Launcher X", gamepad2.x ? "ACTIVE" : "off");
            telemetry.addData("Intake RT", gamepad2.right_trigger);
            telemetry.addData("Intake LT", gamepad2.left_trigger);
            telemetry.addData("Intake Power", intakePower);

            // Transfer servo telemetry
            telemetry.addData("Transfer Servo", hw.transferServo != null ? String.format("Pos: %.2f", hw.transferServo.getPosition()) : "NOT FOUND");

            if (hw.depositMotorL != null && hw.depositMotorR != null) {
                double ticksPerRev = 28.0;
                double velocityL = hw.depositMotorL.getVelocity();
                double velocityR = hw.depositMotorR.getVelocity();
                double rpmL = (velocityL / ticksPerRev) * 60.0;
                double rpmR = (velocityR / ticksPerRev) * 60.0;
                telemetry.addData("Launcher L RPM", rpmL);
                telemetry.addData("Launcher R RPM", rpmR);
            } else {
                telemetry.addData("Launcher", "NOT FOUND");
            }

            telemetry.addData("Loop Hz", 1.0/dt);
            telemetry.addData("Heading", pos.getHeadingDeg());
            telemetry.update();

            // Panels publishing (inches)
            double xIn = pos.getXmm()/25.4;
            double yIn = pos.getYmm()/25.4;
            panels.putText("teleop/pinpoint/status", hw.pinpoint!=null? hw.pinpoint.getDeviceStatus().name():"NONE");
            panels.publishPose("teleop/pose", xIn, yIn, pos.getHeadingDeg(), 50);
        }
    }
}
