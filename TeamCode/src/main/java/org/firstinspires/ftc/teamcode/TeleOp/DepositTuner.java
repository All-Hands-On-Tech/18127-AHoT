package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.common.RobotHardware;
import org.firstinspires.ftc.teamcode.common.Odometry;
import org.firstinspires.ftc.teamcode.common.PanelsPublisher;

import java.util.Locale;
//I need to push ignore this
@TeleOp(name = "Deposit Tuner", group = "TeleOp")
public class DepositTuner extends LinearOpMode {
    @Override
    public void runOpMode() {
        RobotHardware hw = new RobotHardware(hardwareMap);
        hw.initPinpoint();
        PanelsPublisher panels = new PanelsPublisher();
        panels.init();

        telemetry.addLine("Init complete - waiting start");
        telemetry.addLine("Gamepad1: Driving | LB=slow mode | RB=tuning mode");
        telemetry.addLine("Gamepad2: Press X to toggle deposit on/off");
        telemetry.addLine("Gamepad2: Use Dpad to adjust speed");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        double lastLoopTime = getRuntime();

        // ===== CONFIGURABLE SPEED VARIABLES =====
        // Drive speeds (0.0 to 1.0)
        final double DRIVE_SPEED_NORMAL = 1.0;    // Full speed for normal driving
        final double DRIVE_SPEED_SLOW = 0.7;      // 70% speed when holding LB
        final double DRIVE_SPEED_TUNING = 0.35;   // Slower speed for precise tuning/aiming

        // Rotation speeds (0.0 to 1.0)
        final double ROTATE_SPEED_NORMAL = 1.0;   // Full rotation speed
        final double ROTATE_SPEED_SLOW = 0.7;     // 70% rotation when holding LB
        final double ROTATE_SPEED_TUNING = 0.35;  // Slower rotation for tuning

        // Strafe compensation multiplier
        final double STRAFE_COMPENSATION = 1.1;

        // Deposit motor presets (velocity in encoder ticks per second)
        double presetX = 650.0; // default used when gamepad2.x is toggled

        final double MIN_V = 0.0;
        final double MAX_V = 5000.0;
        final double STEP_SMALL = 1.0;
        final double STEP_LARGE = 25.0;

        // Speed mode toggle
        boolean tuningMode = false;
        boolean prevRightBumper = false;

        // For D-pad edge detection and hold-repeat (gamepad2 only for tuning)
        boolean prevGp2DpadUp = false, prevGp2DpadDown = false, prevGp2DpadLeft = false, prevGp2DpadRight = false;
        long gp2LastChange = System.currentTimeMillis();
        final long FIRST_REPEAT_DELAY_MS = 350;
        final long REPEAT_INTERVAL_MS = 120;

        // Odometry
        Odometry odometry = new Odometry(hw, hw.pinpoint);

        // Set initial servo positions - cam
        if (hw.cam != null) hw.cam.setPosition(0.5014);

        // Ensure deposit motors are in velocity mode
        if (hw.depositMotorL != null) {
            hw.depositMotorL.setMode(com.qualcomm.robotcore.hardware.DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            hw.depositMotorL.setMode(com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER);
        }
        if (hw.depositMotorR != null) {
            hw.depositMotorR.setMode(com.qualcomm.robotcore.hardware.DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            hw.depositMotorR.setMode(com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER);
        }

        boolean prevComboReset = false;
        boolean prevComboRecalibrate = false;

        // Toggle states for deposit button X only
        boolean depositRunningX = false;
        boolean prevX = false;


        while (opModeIsActive()) {
            double nowTime = getRuntime();
            double dt = Math.max(0.0, Math.min(0.1, nowTime - lastLoopTime));
            lastLoopTime = nowTime;

            hw.updatePinpoint();
            odometry.update();
            Odometry.Position pos = odometry.getPosition();

            // ===== DRIVER 1: CHASSIS CONTROL =====
            // Toggle tuning mode with right bumper (edge detection)
            if (gamepad1.right_bumper && !prevRightBumper) {
                tuningMode = !tuningMode;
            }
            prevRightBumper = gamepad1.right_bumper;

            // Read raw joystick values (inverted Y for forward)
            double forward = -gamepad1.left_stick_y;
            double strafe = gamepad1.left_stick_x;
            double rotate = -gamepad1.right_stick_x; // Negated to fix turning direction

            // Apply very small deadzone for precision (0.02 is barely noticeable)
            final double DEADZONE = 0.02;
            if (Math.abs(forward) < DEADZONE) forward = 0;
            if (Math.abs(strafe) < DEADZONE) strafe = 0;
            if (Math.abs(rotate) < DEADZONE) rotate = 0;

            // Calculate magnitude and angle for true 360-degree movement
            double magnitude = Math.hypot(forward, strafe);
            double angle = Math.atan2(forward, strafe);

            // Apply speed multiplier based on mode
            double driveMul, rotateMul;
            if (tuningMode) {
                // Tuning mode: slow and precise
                driveMul = DRIVE_SPEED_TUNING;
                rotateMul = ROTATE_SPEED_TUNING;
            } else if (gamepad1.left_bumper) {
                // Slow mode: 70% speed
                driveMul = DRIVE_SPEED_SLOW;
                rotateMul = ROTATE_SPEED_SLOW;
            } else {
                // Normal mode: full speed
                driveMul = DRIVE_SPEED_NORMAL;
                rotateMul = ROTATE_SPEED_NORMAL;
            }

            magnitude *= driveMul;
            rotate *= rotateMul;

            // Apply cubic response curve to magnitude for smooth control
            magnitude = Math.copySign(magnitude * magnitude * magnitude, magnitude);
            rotate = Math.copySign(rotate * rotate * rotate, rotate);

            // Reconstruct forward and strafe from polar coordinates
            forward = magnitude * Math.sin(angle);
            strafe = magnitude * Math.cos(angle) * STRAFE_COMPENSATION;

            // Add slight counter-rotation when moving forward for stability
            // Calculate wheel powers for mecanum drive
            double denominator = Math.max(Math.abs(forward) + Math.abs(strafe) + Math.abs(rotate), 1);
            double fl = (forward + strafe - rotate) / denominator;
            double fr = (forward - strafe + rotate) / denominator;
            double bl = (forward - strafe - rotate) / denominator;
            double br = (forward + strafe + rotate) / denominator;
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

            // Intake (triggers, both => stop)
            boolean rt = gamepad2.right_trigger > 0.1;
            boolean lt = gamepad2.left_trigger > 0.1;
            double intakePower = (rt && lt) ? 0.0 : rt ? 0.9 : lt ? -0.9 : 0.0;
            if (hw.intakeMotor != null) hw.intakeMotor.setPower(intakePower);

            // ===== DPAD TUNING (GAMEPAD2 ONLY) =====
            // Read gamepad2 dpad states
            boolean gp2Up = gamepad2.dpad_up;
            boolean gp2Down = gamepad2.dpad_down;
            boolean gp2Left = gamepad2.dpad_left;
            boolean gp2Right = gamepad2.dpad_right;

            long now = System.currentTimeMillis();

            // Adjust presetX with dpad (no need to hold anything)
            if (gp2Up && (!prevGp2DpadUp || now - gp2LastChange > FIRST_REPEAT_DELAY_MS)) {
                presetX = Math.min(MAX_V, presetX + STEP_LARGE);
                gp2LastChange = now;
            }
            if (gp2Down && (!prevGp2DpadDown || now - gp2LastChange > FIRST_REPEAT_DELAY_MS)) {
                presetX = Math.max(MIN_V, presetX - STEP_LARGE);
                gp2LastChange = now;
            }
            if (gp2Right && (!prevGp2DpadRight || now - gp2LastChange > FIRST_REPEAT_DELAY_MS)) {
                presetX = Math.min(MAX_V, presetX + STEP_SMALL);
                gp2LastChange = now;
            }
            if (gp2Left && (!prevGp2DpadLeft || now - gp2LastChange > FIRST_REPEAT_DELAY_MS)) {
                presetX = Math.max(MIN_V, presetX - STEP_SMALL);
                gp2LastChange = now;
            }

            // Enable faster repeat interval while holding dpad
            if ((gp2Up || gp2Down || gp2Left || gp2Right) && now - gp2LastChange > REPEAT_INTERVAL_MS) {
                gp2LastChange = now - (REPEAT_INTERVAL_MS + 1);
            }

            // Toggle deposit button X (edge detection)
            if (gamepad2.x && !prevX) {
                depositRunningX = !depositRunningX;
            }
            prevX = gamepad2.x;

            // Apply deposit velocity based on X toggle
            double target = depositRunningX ? presetX : 0.0;

            // Read actual velocity from both motors (ticks/sec) for telemetry
            double vL = 0.0;
            double vR = 0.0;
            int posL = 0;
            int posR = 0;

            if (hw.depositMotorL != null) {
                vL = hw.depositMotorL.getVelocity();
                posL = hw.depositMotorL.getCurrentPosition();
            }
            if (hw.depositMotorR != null) {
                vR = hw.depositMotorR.getVelocity();
                posR = hw.depositMotorR.getCurrentPosition();
            }
            double actual = (vL + vR) / 2.0;

            // Use motor's built-in velocity PID control - both motors get same target
            // When target is 0, explicitly stop motors to avoid drift
            if (Math.abs(target) < 1.0) {
                if (hw.depositMotorL != null) hw.depositMotorL.setPower(0);
                if (hw.depositMotorR != null) hw.depositMotorR.setPower(0);
            } else {
                if (hw.depositMotorL != null) hw.depositMotorL.setVelocity(target);
                if (hw.depositMotorR != null) hw.depositMotorR.setVelocity(target);
            }

            // Cam (LB/RB) continuous hold-based control - only update when buttons pressed
            if (hw.cam != null) {
                if (gamepad2.left_bumper || gamepad2.right_bumper) {
                    double camPosition = hw.cam.getPosition();
                    if (gamepad2.left_bumper) camPosition -= 0.001; // Move in
                    if (gamepad2.right_bumper) camPosition += 0.001; // Move out
                    // Clamp between reasonable bounds
                    camPosition = Math.max(0.4622, Math.min(0.5522, camPosition));
                    hw.cam.setPosition(camPosition);
                }
            }

            // Telemetry - Essential Info Only
            telemetry.addData("Target Speed", String.format(Locale.US, "%.0f ticks/s", target));
            telemetry.addData("Actual Speed (avg)", String.format(Locale.US, "%.0f ticks/s", actual));

            // Motor diagnostics - always show
            telemetry.addLine();
            telemetry.addData("Motor Status", String.format(Locale.US, "L: %s | R: %s",
                hw.depositMotorL != null ? "OK" : "NULL",
                hw.depositMotorR != null ? "OK" : "NULL"));

            // Always display motor data regardless of running state
            double ticksPerRev = 28.0;
            double rpmL = (vL / ticksPerRev) * 60.0;
            double rpmR = (vR / ticksPerRev) * 60.0;
            telemetry.addData("Motor RPM", String.format(Locale.US, "L: %.0f | R: %.0f", rpmL, rpmR));
            telemetry.addData("Motor Velocities", String.format(Locale.US, "L: %.0f | R: %.0f ticks/s", vL, vR));
            telemetry.addData("Motor Positions", String.format(Locale.US, "L: %d | R: %d", posL, posR));
            if (hw.depositMotorL != null && hw.depositMotorR != null) {
                telemetry.addData("Motor Power", String.format(Locale.US, "L: %.3f | R: %.3f",
                    hw.depositMotorL.getPower(), hw.depositMotorR.getPower()));
            }

            telemetry.addLine();
            telemetry.addData("Preset X", String.format(Locale.US, "%.0f ticks/s", presetX));
            telemetry.addData("Running Mode", depositRunningX ? "ON" : "OFF");

            // Speed mode display
            String speedMode;
            if (tuningMode) {
                speedMode = String.format(Locale.US, "TUNING (%.0f%%)", DRIVE_SPEED_TUNING * 100);
            } else if (gamepad1.left_bumper) {
                speedMode = String.format(Locale.US, "SLOW (%.0f%%)", DRIVE_SPEED_SLOW * 100);
            } else {
                speedMode = String.format(Locale.US, "NORMAL (%.0f%%)", DRIVE_SPEED_NORMAL * 100);
            }
            telemetry.addData("Drive Speed", speedMode);

            telemetry.addData("Controls", "GP1: Drive | LB=slow | RB=toggle tuning mode");
            telemetry.addData("", "GP2: Press X=toggle deposit on/off | Dpad=adjust speed");
            telemetry.addData("", "GP2: LB/RB=cam | Triggers=intake");
            telemetry.addData("Loop Hz", String.format(Locale.US, "%.1f", 1.0 / Math.max(1e-6, dt)));
            telemetry.update();

            // Panels publishing (inches)
            double xIn = pos.getXmm()/25.4;
            double yIn = pos.getYmm()/25.4;
            panels.putText("teleop/pinpoint/status", hw.pinpoint!=null? hw.pinpoint.getDeviceStatus().name():"NONE");
            panels.publishPose("teleop/pose", xIn, yIn, pos.getHeadingDeg(), 50);

            // update previous dpad states (gamepad2 only)
            prevGp2DpadUp = gp2Up; prevGp2DpadDown = gp2Down; prevGp2DpadLeft = gp2Left; prevGp2DpadRight = gp2Right;

            // small sleep to yield
            sleep(20);
        }
    }
}