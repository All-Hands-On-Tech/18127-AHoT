package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.common.RobotHardware;
import org.firstinspires.ftc.teamcode.common.RobotConfig;
import org.firstinspires.ftc.teamcode.common.Odometry;
import org.firstinspires.ftc.teamcode.GoBildaPinpointDriver;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.GoBildaPinpointDriver.EncoderDirection;

@TeleOp(name = "Pinpoint Localizer Test", group = "Testing")
public class PinpointLocalizerTest extends LinearOpMode {
    @Override
    public void runOpMode() {
        RobotHardware hw = new RobotHardware(hardwareMap);
        RobotConfig config = new RobotConfig();
        hw.initPinpoint();

        telemetry.setMsTransmissionInterval(50); // Update faster for better debugging

        telemetry.addLine("=== PINPOINT LOCALIZER TEST ===");
        telemetry.addLine();
        telemetry.addData("Pinpoint Status", hw.pinpoint == null ? "NOT FOUND" : hw.pinpoint.getDeviceStatus().name());
        if (hw.pinpoint != null) {
            telemetry.addData("Config Name", config.pinpointName);
            telemetry.addData("Perpendicular Offset", config.odoPerpendicularOffsetMM + " mm");
            telemetry.addData("Parallel Offset", config.odoParallelOffsetMM + " mm");
        }
        telemetry.addLine();
        telemetry.addLine("CONTROLS:");
        telemetry.addLine("  START+BACK = Reset Position & IMU");
        telemetry.addLine("  X+Y = Recalibrate IMU");
        telemetry.addLine("  A = Toggle Raw Encoder Display");
        telemetry.addLine("  B = Toggle Detailed Stats");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        Odometry odometry = new Odometry(hw, hw.pinpoint);

        boolean prevResetCombo = false;
        boolean prevRecalibrateCombo = false;
        boolean prevA = false;
        boolean prevB = false;
        boolean showRawEncoders = false;
        boolean showDetailedStats = true;
        // Diagnostic toggles for encoder directions
        boolean prevLB = false;
        boolean prevRB = false;
        boolean prevX = false;
        EncoderDirection forwardDir = EncoderDirection.FORWARD;
        EncoderDirection strafeDir = EncoderDirection.FORWARD;

        double lastLoopTime = getRuntime();
        int loopCount = 0;
        double avgLoopTime = 0;

        while (opModeIsActive()) {
            double now = getRuntime();
            double dt = now - lastLoopTime;
            lastLoopTime = now;

            // Update loop stats
            loopCount++;
            avgLoopTime = avgLoopTime * 0.95 + dt * 0.05; // Exponential moving average

            // Update Pinpoint and Odometry
            hw.updatePinpoint();
            odometry.update();
            Odometry.Position pos = odometry.getPosition();

            // --- Diagnostic controls ---
            // Left bumper: toggle forward encoder direction
            if (gamepad1.left_bumper && !prevLB) {
                if (hw.pinpoint != null) {
                    forwardDir = (forwardDir == EncoderDirection.FORWARD) ? EncoderDirection.REVERSED : EncoderDirection.FORWARD;
                    hw.pinpoint.setEncoderDirections(forwardDir, strafeDir);
                    telemetry.speak("Toggled forward encoder direction to " + forwardDir.name());
                }
            }
            prevLB = gamepad1.left_bumper;

            // Right bumper: toggle strafe encoder direction
            if (gamepad1.right_bumper && !prevRB) {
                if (hw.pinpoint != null) {
                    strafeDir = (strafeDir == EncoderDirection.FORWARD) ? EncoderDirection.REVERSED : EncoderDirection.FORWARD;
                    hw.pinpoint.setEncoderDirections(forwardDir, strafeDir);
                    telemetry.speak("Toggled strafe encoder direction to " + strafeDir.name());
                }
            }
            prevRB = gamepad1.right_bumper;

            // X: reinitialize Pinpoint (useful after changing directions)
            if (gamepad1.x && !prevX) {
                if (hw.pinpoint != null) {
                    hw.pinpoint.initialize();
                    hw.pinpoint.resetPosAndIMU();
                    telemetry.speak("Reinitialized pinpoint");
                }
            }
            prevX = gamepad1.x;

            // === CONTROLS ===
            // Reset position and IMU
            boolean resetCombo = gamepad1.start && gamepad1.back;
            if (resetCombo && !prevResetCombo) {
                if (hw.pinpoint != null) {
                    hw.pinpoint.resetPosAndIMU();
                    telemetry.speak("Reset position and I M U");
                }
            }
            prevResetCombo = resetCombo;

            // Recalibrate IMU
            boolean recalibrateCombo = gamepad1.x && gamepad1.y;
            if (recalibrateCombo && !prevRecalibrateCombo) {
                if (hw.pinpoint != null) {
                    hw.pinpoint.recalibrateIMU();
                    telemetry.speak("Recalibrating I M U");
                }
            }
            prevRecalibrateCombo = recalibrateCombo;

            // Toggle raw encoder display
            if (gamepad1.a && !prevA) {
                showRawEncoders = !showRawEncoders;
            }
            prevA = gamepad1.a;

            // Toggle detailed stats
            if (gamepad1.b && !prevB) {
                showDetailedStats = !showDetailedStats;
            }
            prevB = gamepad1.b;

            // === TELEMETRY ===
            telemetry.clear();
            telemetry.addLine("=== PINPOINT LOCALIZER TEST ===");
            telemetry.addLine();

            // Device Status
            if (hw.pinpoint != null) {
                GoBildaPinpointDriver.DeviceStatus status = hw.pinpoint.getDeviceStatus();
                String statusColor = status == GoBildaPinpointDriver.DeviceStatus.READY ? "✓" : "!";
                telemetry.addData(statusColor + " Device Status", status.name());

                if (hw.pinpointBadReadCount > 0) {
                    telemetry.addData("⚠ Bad Reads", hw.pinpointBadReadCount);
                    if (!hw.pinpointRecoveryAction.isEmpty()) {
                        telemetry.addData("Recovery Action", hw.pinpointRecoveryAction);
                    }
                }

                if (hw.pinpointBusDowngraded) {
                    telemetry.addLine("⚠ I2C Bus Downgraded to 100kHz");
                }

                // Show configured encoder directions (local cached state)
                telemetry.addData("Forward Dir (toggle LB)", forwardDir.name());
                telemetry.addData("Strafe Dir (toggle RB)", strafeDir.name());
            } else {
                telemetry.addLine("✗ PINPOINT NOT FOUND");
                telemetry.addData("Expected Name", config.pinpointName);
                telemetry.addLine();
                telemetry.addLine("Check:");
                telemetry.addLine("  1. Hardware config matches");
                telemetry.addLine("  2. Device is properly connected");
                telemetry.addLine("  3. I2C bus is functional");
            }

            telemetry.addLine();

            // Position Data
            telemetry.addLine("--- POSITION DATA ---");
            telemetry.addData("X (mm)", String.format("%.1f", pos.getXmm()));
            telemetry.addData("Y (mm)", String.format("%.1f", pos.getYmm()));
            telemetry.addData("X (inches)", String.format("%.2f", pos.getXmm() / 25.4));
            telemetry.addData("Y (inches)", String.format("%.2f", pos.getYmm() / 25.4));
            telemetry.addData("Heading (deg)", String.format("%.1f", pos.getHeadingDeg()));
            telemetry.addData("Heading (rad)", String.format("%.3f", pos.getHeadingRad()));
            telemetry.addLine();

            // Pinpoint raw encoders and config
            if (hw.pinpoint != null) {
                telemetry.addLine("--- PINPOINT RAW ---");
                try {
                    telemetry.addData("Encoder X (counts)", hw.pinpoint.getEncoderX());
                    telemetry.addData("Encoder Y (counts)", hw.pinpoint.getEncoderY());
                } catch (Exception ignored) {}
                try {
                    telemetry.addData("Pod X Offset (mm)", String.format("%.1f", config.odoPerpendicularOffsetMM));
                    telemetry.addData("Pod Y Offset (mm)", String.format("%.1f", config.odoParallelOffsetMM));
                } catch (Exception ignored) {}
                telemetry.addLine();
            }

            // Velocity Data (if available)
            if (hw.pinpoint != null) {
                Pose2D vel = hw.pinpoint.getVelocity();
                if (vel != null) {
                    telemetry.addLine("--- VELOCITY DATA ---");
                    telemetry.addData("X Velocity (mm/s)", String.format("%.1f", vel.getX(DistanceUnit.MM)));
                    telemetry.addData("Y Velocity (mm/s)", String.format("%.1f", vel.getY(DistanceUnit.MM)));
                    telemetry.addData("Heading Vel (deg/s)", String.format("%.1f", Math.toDegrees(vel.getHeading(AngleUnit.RADIANS))));
                    telemetry.addLine();
                }
            }

            // Raw Encoder Data
            if (showRawEncoders && hw.pinpoint != null) {
                telemetry.addLine("--- RAW ENCODER DATA ---");
                telemetry.addData("Encoder X", hw.pinpoint.getEncoderX());
                telemetry.addData("Encoder Y", hw.pinpoint.getEncoderY());
                telemetry.addLine();
            }

            // Detailed Stats
            if (showDetailedStats) {
                telemetry.addLine("--- PERFORMANCE STATS ---");
                telemetry.addData("Loop Frequency", String.format("%.1f Hz", 1.0 / avgLoopTime));
                telemetry.addData("Loop Time", String.format("%.1f ms", avgLoopTime * 1000));
                telemetry.addData("Total Loops", loopCount);
                telemetry.addLine();
            }

            // Controls Reminder
            telemetry.addLine("--- CONTROLS ---");
            telemetry.addData("START+BACK", resetCombo ? "RESETTING" : "Reset Pos & IMU");
            telemetry.addData("X+Y", recalibrateCombo ? "RECALIBRATING" : "Recalibrate IMU");
            telemetry.addData("A", "Raw Encoders: " + (showRawEncoders ? "ON" : "OFF"));
            telemetry.addData("B", "Detailed Stats: " + (showDetailedStats ? "ON" : "OFF"));

            telemetry.update();
        }
    }
}
