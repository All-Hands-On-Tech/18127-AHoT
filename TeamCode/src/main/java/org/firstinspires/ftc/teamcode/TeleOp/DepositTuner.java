package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.common.RobotHardware;
import org.firstinspires.ftc.teamcode.common.Odometry;
import org.firstinspires.ftc.teamcode.common.PanelsPublisher;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.Locale;
//I need to push ignore this
@TeleOp(name = "Deposit Tuner", group = "TeleOp")
public class DepositTuner extends LinearOpMode {
    // Vision localization components
    private AprilTagProcessor aprilTagProcessor;
    private VisionPortal visionPortal;

    // Alliance selection
    private enum Alliance { NONE, BLUE, RED }
    private Alliance robotAlliance = Alliance.NONE;

    // Auto-aiming constants
    private static final double AUTO_AIM_MAX_ROTATION_SPEED = 0.25;  // Maximum rotation speed (25%)
    private static final double AUTO_AIM_MIN_ROTATION_SPEED = 0.08;  // Minimum rotation speed to overcome friction
    private static final double AUTO_AIM_HEADING_TOLERANCE = 2.0;  // degrees - stop when within this range
    private static final double AUTO_AIM_SLOW_DOWN_ANGLE = 20.0;  // degrees - start slowing down at this angle
    private static final double AUTO_AIM_BLUE_OFFSET = 7.7;   // Blue alliance heading offset (degrees) - corrected
    private static final double AUTO_AIM_RED_OFFSET = -7.7;    // Red alliance heading offset (degrees) - corrected

    // Camera configuration constants (adjust for your robot)
    private static final double CAMERA_X_OFFSET_MM = 0.0;    // mm right from robot center
    private static final double CAMERA_Y_OFFSET_MM = 140.0;    // mm forward from robot center
    private static final double CAMERA_Z_OFFSET_MM = 220.4;  // mm up (6 inches)
    private static final double CAMERA_YAW = 0.0;            // degrees (0 = forward)
    private static final double CAMERA_PITCH = 20.0;        // degrees (-90 = horizontal)
    private static final double CAMERA_ROLL = 0.0;           // degrees (0 = upright)

    @Override
    public void runOpMode() {
        RobotHardware hw = new RobotHardware(hardwareMap);
        hw.initPinpoint();
        PanelsPublisher panels = new PanelsPublisher();
        panels.init();

        // Initialize vision localization
        initAprilTagVision();

        // Alliance Selection - Driver 1 presses A for Blue or B for Red
        telemetry.addLine("=== ALLIANCE SELECTION ===");
        telemetry.addLine("Driver 1: Press A for BLUE");
        telemetry.addLine("Driver 1: Press B for RED");
        telemetry.addLine();
        telemetry.addData("Current Alliance", "NONE");
        telemetry.update();

        while (!isStarted() && !isStopRequested()) {
            if (gamepad1.a) {
                robotAlliance = Alliance.BLUE;
            } else if (gamepad1.b) {
                robotAlliance = Alliance.RED;
            }

            telemetry.addLine("=== ALLIANCE SELECTION ===");
            telemetry.addLine("Driver 1: Press A for BLUE");
            telemetry.addLine("Driver 1: Press B for RED");
            telemetry.addLine();

            String allianceColor = "NONE";
            if (robotAlliance == Alliance.BLUE) {
                allianceColor = "BLUE";
            } else if (robotAlliance == Alliance.RED) {
                allianceColor = "RED";
            }
            telemetry.addData("Current Alliance", allianceColor);
            telemetry.addLine();
            telemetry.addLine("---");
            telemetry.addLine("Init complete - waiting start");
            telemetry.addLine("Gamepad1: Driving | LB=slow mode | RB=tuning mode");
            telemetry.addLine("Gamepad1: Hold X for auto-aim to alliance goal");
            telemetry.addLine("Gamepad2: Press X to toggle deposit on/off");
            telemetry.addLine("Gamepad2: Use Dpad to adjust speed");
            telemetry.addLine("Vision: AprilTag localization enabled");
            telemetry.update();

            sleep(50);
        }

        telemetry.addLine("=== STARTING ===");
        telemetry.addData("Alliance", robotAlliance == Alliance.BLUE ? "BLUE" :
                                       robotAlliance == Alliance.RED ? "RED" : "NONE");
        telemetry.update();
        if (isStopRequested()) return;

        double lastLoopTime = getRuntime();

        // ===== CONFIGURABLE SPEED VARIABLES =====
        // Drive speeds (0.0 to 1.0)
        final double DRIVE_SPEED_NORMAL = 1.0;    // Full speed for normal driving
        final double DRIVE_SPEED_SLOW = 0.6;      // 70% speed when holding LB
        final double DRIVE_SPEED_TUNING = 0.35;   // Slower speed for precise tuning/aiming

        // Rotation speeds (0.0 to 1.0)
        final double ROTATE_SPEED_NORMAL = 0.9;   // Full rotation speed
        final double ROTATE_SPEED_SLOW = 0.45;     // 70% rotation when holding LB
        final double ROTATE_SPEED_TUNING = 0.35;  // Slower rotation for tuning

        // Strafe compensation multiplier
        final double STRAFE_COMPENSATION = 1.1;

        // Deposit motor presets (velocity in encoder ticks per second)
        double presetX = 665.0; // default used when gamepad2.x is toggled

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
        if (hw.cam != null) hw.cam.setPosition(0.5181);

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

        // Vibration control for gamepad1 dpad_up -> gamepad2 vibration
        boolean prevGp1DpadUp = false;
        long vibrationEndTime = 0; // timestamp when vibration should stop


        while (opModeIsActive()) {
            double nowTime = getRuntime();
            double dt = Math.max(0.0, Math.min(0.1, nowTime - lastLoopTime));
            lastLoopTime = nowTime;

            hw.updatePinpoint();
            odometry.update();
            Odometry.Position pos = odometry.getPosition();

            // Get odometry position in mm
            double odoX_mm = pos.getXmm();
            double odoY_mm = pos.getYmm();
            double odoHeading = pos.getHeadingDeg();

            // Process AprilTag detections for vision localization
            List<AprilTagDetection> visionDetections = null;
            double visionX_mm = 0.0, visionY_mm = 0.0, visionHeading = 0.0;
            int visionTagCount = 0;
            boolean visionValid = false;

            if (aprilTagProcessor != null) {
                visionDetections = aprilTagProcessor.getDetections();
                visionTagCount = visionDetections.size();

                // Average position from all detected tags
                if (!visionDetections.isEmpty()) {
                    double sumX = 0, sumY = 0, sumHeading = 0;
                    int validCount = 0;
                    for (AprilTagDetection detection : visionDetections) {
                        if (detection.metadata != null && detection.ftcPose != null) {
                            // Convert inches to mm (FTC SDK provides inches)
                            sumX += detection.ftcPose.x * 25.4;
                            sumY += detection.ftcPose.y * 25.4;
                            sumHeading += detection.ftcPose.yaw;
                            validCount++;
                        }
                    }
                    if (validCount > 0) {
                        visionX_mm = sumX / validCount;
                        visionY_mm = sumY / validCount;
                        visionHeading = sumHeading / validCount;
                        visionValid = true;
                    }
                }
            }

            // ===== SENSOR FUSION: Calculate Fused Position =====
            // Combine odometry and vision for best position estimate
            double fusedX_mm, fusedY_mm, fusedHeading;
            double visionWeight = 0.0;

            if (visionValid && visionTagCount >= 2) {
                // High confidence with 2+ tags - trust vision more
                visionWeight = 0.7;
            } else if (visionValid && visionTagCount == 1) {
                // Lower confidence with 1 tag - balance vision and odometry
                visionWeight = 0.4;
            }

            if (visionValid) {
                fusedX_mm = visionWeight * visionX_mm + (1.0 - visionWeight) * odoX_mm;
                fusedY_mm = visionWeight * visionY_mm + (1.0 - visionWeight) * odoY_mm;
                fusedHeading = visionWeight * visionHeading + (1.0 - visionWeight) * odoHeading;
            } else {
                // No vision - use odometry only
                fusedX_mm = odoX_mm;
                fusedY_mm = odoY_mm;
                fusedHeading = odoHeading;
            }

            // ===== DRIVER 1: CHASSIS CONTROL =====
            // Toggle tuning mode with right bumper (edge detection)
            if (gamepad1.right_bumper && !prevRightBumper) {
                tuningMode = !tuningMode;
            }
            prevRightBumper = gamepad1.right_bumper;

            // Read raw joystick values (inverted Y for forward)
            double forward = gamepad1.left_stick_y;   // Reverse drive direction
            double strafe = -gamepad1.left_stick_x;   // Reverse strafe direction
            double rotate = -gamepad1.right_stick_x;  // Negated for correct rotation direction

            // ===== AUTO-AIMING LOGIC =====
            // Driver 1 holds X to auto-aim at alliance goal
            boolean autoAiming = false;
            boolean targetTagVisible = false;

            if (gamepad1.x && robotAlliance != Alliance.NONE && visionDetections != null) {
                // Check if the alliance-specific tag is visible
                for (AprilTagDetection detection : visionDetections) {
                    if (detection.metadata != null && detection.ftcPose != null) {
                        String tagName = detection.metadata.name.toLowerCase();

                        // Check if this is the correct alliance tag
                        if ((robotAlliance == Alliance.BLUE && tagName.contains("blue")) ||
                            (robotAlliance == Alliance.RED && tagName.contains("red"))) {
                            targetTagVisible = true;

                            // Get the bearing to the tag (angle from camera to tag)
                            // Bearing is negative when tag is to the left, positive when to the right
                            double tagBearing = detection.ftcPose.bearing;

                            // Apply alliance-specific offset
                            double targetBearing = 0.0; // We want tag centered (bearing = 0)
                            if (robotAlliance == Alliance.BLUE) {
                                targetBearing = AUTO_AIM_BLUE_OFFSET;
                            } else if (robotAlliance == Alliance.RED) {
                                targetBearing = AUTO_AIM_RED_OFFSET;
                            }

                            // Calculate error: how far off from desired bearing
                            // Negative error means tag is to the right of target, need to rotate right (CW, negative)
                            // Positive error means tag is to the left of target, need to rotate left (CCW, positive)
                            double bearingError = targetBearing - tagBearing;  // Inverted to turn towards tag

                            // Normalize to [-180, 180] range
                            while (bearingError > 180) bearingError -= 360;
                            while (bearingError < -180) bearingError += 360;

                            // Only auto-aim if error is significant
                            if (Math.abs(bearingError) > AUTO_AIM_HEADING_TOLERANCE) {
                                autoAiming = true;

                                // Calculate proportional rotation speed based on error
                                double absError = Math.abs(bearingError);
                                double speedScale;

                                if (absError > AUTO_AIM_SLOW_DOWN_ANGLE) {
                                    speedScale = 1.0;
                                } else {
                                    speedScale = absError / AUTO_AIM_SLOW_DOWN_ANGLE;
                                }

                                // Calculate rotation speed with min/max bounds
                                double rotationSpeed = AUTO_AIM_MIN_ROTATION_SPEED +
                                    (AUTO_AIM_MAX_ROTATION_SPEED - AUTO_AIM_MIN_ROTATION_SPEED) * speedScale;

                                rotate = Math.copySign(rotationSpeed, bearingError);

                            } else {
                                // Within tolerance - stop rotating
                                rotate = 0;
                            }
                            break;
                        }
                    }
                }
            }


            // Calculate magnitude and angle for true 360-degree movement
            double magnitude = Math.hypot(forward, strafe);
            double angle = Math.atan2(strafe, forward);  // atan2(x, y) for proper angle in all quadrants

            // Apply speed multiplier based on mode FIRST (before cubic curve)
            double driveMul, rotateMul;
            if (autoAiming) {
                // Auto-aiming: rotation already set, don't modify
                driveMul = 1.0;
                rotateMul = 1.0; // Don't modify auto-aim rotation speed
            } else if (tuningMode) {
                // Tuning mode: slow and precise
                driveMul = DRIVE_SPEED_TUNING;
                rotateMul = ROTATE_SPEED_TUNING;
            } else if (gamepad1.left_bumper) {
                // Slow mode: 60% speed
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
            if (!autoAiming) {
                rotate = Math.copySign(rotate * rotate * rotate, rotate);
            }

            // Reconstruct forward and strafe from polar coordinates
            forward = magnitude * Math.cos(angle);
            strafe = magnitude * Math.sin(angle) * STRAFE_COMPENSATION;
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

            // ===== VIBRATION CONTROL =====
            // Driver 1 presses dpad_up -> Driver 2 controller vibrates for 2 seconds
            if (gamepad1.dpad_up && !prevGp1DpadUp) {
                // Start vibration on gamepad2 for 2000 milliseconds (2 seconds)
                vibrationEndTime = System.currentTimeMillis() + 2000;
                gamepad2.rumble(1.0, 1.0, 2000); // Full power rumble for 2 seconds
            }
            prevGp1DpadUp = gamepad1.dpad_up;

            // Update vibration status (optional - the rumble command handles duration automatically)
            // but we track it for telemetry if needed
            boolean isVibrating = System.currentTimeMillis() < vibrationEndTime;

            // Intake1 (triggers, both => stop)
            boolean lt = gamepad2.right_trigger > 0.1;
            boolean rt = gamepad2.left_trigger > 0.1;
            double intake1Power = (rt && lt) ? 0.0 : rt ? 1.0 : lt ? -1.0 : 0.0;
            if (hw.intake1 != null) hw.intake1.setPower(intake1Power);

            // Intake2 (A and B buttons - full power in/out)
            double intake2Power = 0.0;
            if (gamepad2.a && !gamepad2.b) {
                intake2Power = 1.0;  // A button = forward/in
            } else if (gamepad2.b && !gamepad2.a) {
                intake2Power = -1.0; // B button = reverse/out
            }
            // Both pressed or neither pressed = stop (0.0)
            if (hw.intake2 != null) hw.intake2.setPower(intake2Power);

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
                    camPosition = Math.max(0.4622, Math.min(0.5322, camPosition));
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

            // Cam position tracking
            if (hw.cam != null) {
                telemetry.addData("Cam Position", String.format(Locale.US, "%.4f", hw.cam.getPosition()));
            } else {
                telemetry.addData("Cam Position", "N/A");
            }

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

            // Auto-aiming status
            telemetry.addLine();
            telemetry.addData("Alliance", robotAlliance == Alliance.BLUE ? "BLUE" :
                                          robotAlliance == Alliance.RED ? "RED" : "NONE");
            if (autoAiming) {
                telemetry.addData("Auto-Aim", "ACTIVE - Rotating to 0°");
            } else if (gamepad1.x && robotAlliance != Alliance.NONE) {
                if (!targetTagVisible) {
                    telemetry.addData("Auto-Aim", "Searching for alliance tag...");
                } else {
                    telemetry.addData("Auto-Aim", "Target aligned!");
                }
            }

            // ===== LOCALIZATION DISPLAY =====
            telemetry.addLine();
            telemetry.addLine("=== ROBOT LOCALIZATION ===");

            // Fused Position (Primary - combines odometry and vision)
            telemetry.addLine("FUSED POSITION (Primary):");
            telemetry.addData("  X", String.format(Locale.US, "%.1f mm", fusedX_mm));
            telemetry.addData("  Y", String.format(Locale.US, "%.1f mm", fusedY_mm));
            telemetry.addData("  Heading", String.format(Locale.US, "%.1f°", fusedHeading));
            if (visionValid) {
                telemetry.addData("  Fusion", String.format(Locale.US, "%.0f%% vision, %.0f%% odo",
                    visionWeight * 100, (1.0 - visionWeight) * 100));
            } else {
                telemetry.addData("  Fusion", "100% odometry (no vision)");
            }

            telemetry.addLine();

            // Odometry Position
            telemetry.addLine("ODOMETRY:");
            telemetry.addData("  X", String.format(Locale.US, "%.1f mm", odoX_mm));
            telemetry.addData("  Y", String.format(Locale.US, "%.1f mm", odoY_mm));
            telemetry.addData("  Heading", String.format(Locale.US, "%.1f°", odoHeading));

            telemetry.addLine();

            // Vision Position
            telemetry.addLine("VISION (AprilTag):");
            telemetry.addData("  Tags Detected", visionTagCount);
            if (visionValid) {
                telemetry.addData("  X", String.format(Locale.US, "%.1f mm", visionX_mm));
                telemetry.addData("  Y", String.format(Locale.US, "%.1f mm", visionY_mm));
                telemetry.addData("  Heading", String.format(Locale.US, "%.1f°", visionHeading));

                // Show individual tag IDs
                if (visionDetections != null && !visionDetections.isEmpty()) {
                    StringBuilder tagIds = new StringBuilder();
                    for (AprilTagDetection det : visionDetections) {
                        if (tagIds.length() > 0) tagIds.append(", ");
                        tagIds.append(det.id);
                        if (det.metadata != null) {
                            tagIds.append(" (").append(det.metadata.name).append(")");
                        }
                    }
                    telemetry.addData("  Tag IDs", tagIds.toString());
                }
            } else {
                telemetry.addData("  Status", "No tags visible");
            }
            telemetry.addLine();

            telemetry.addData("Controls", "GP1: Drive | LB=slow | RB=toggle tuning mode");
            telemetry.addData("", "GP2: Press X=toggle deposit on/off | Dpad=adjust speed");
            telemetry.addData("", "GP2: LB/RB=cam | Triggers=intake1 | A/B=intake2");
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

        // Cleanup vision on exit
        if (visionPortal != null) {
            visionPortal.close();
        }
    }

    /**
     * Initialize AprilTag vision processor for field localization
     */
    private void initAprilTagVision() {
        try {
            // Create camera position and orientation on robot
            Position cameraPosition = new Position(DistanceUnit.MM,
                CAMERA_X_OFFSET_MM, CAMERA_Y_OFFSET_MM, CAMERA_Z_OFFSET_MM, 0);
            YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES,
                CAMERA_YAW, CAMERA_PITCH, CAMERA_ROLL, 0);

            // Create AprilTag processor with camera pose for field coordinate calculation
            aprilTagProcessor = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(false)
                .setDrawTagOutline(true)
                .setCameraPose(cameraPosition, cameraOrientation)
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)  // SDK outputs inches
                .build();

            // Set decimation for balance between speed and range
            aprilTagProcessor.setDecimation(2);

            // Build vision portal
            VisionPortal.Builder builder = new VisionPortal.Builder();

            // Try to get webcam, fall back to phone camera
            try {
                builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
            } catch (Exception e) {
                builder.setCamera(org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection.BACK);
            }

            builder.addProcessor(aprilTagProcessor);
            builder.enableLiveView(true);
            builder.setAutoStopLiveView(false);

            visionPortal = builder.build();

        } catch (Exception e) {
            telemetry.addLine("Warning: Vision init failed - " + e.getMessage());
            aprilTagProcessor = null;
            visionPortal = null;
        }
    }
}
