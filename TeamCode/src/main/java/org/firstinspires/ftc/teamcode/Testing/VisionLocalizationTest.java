package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
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

/**
 * Vision Localization Test - AprilTag Detection and Field Position Tracking
 *
 * This OpMode demonstrates AprilTag detection for the DECODE season using the
 * standard FTC Field Coordinate System where AprilTags provide robot position.
 *
 * Features:
 * - AprilTag ID detection
 * - Robot field position (X, Y, Heading) based on tag library
 * - Detailed pose data (Range, Bearing, Elevation)
 * - Camera-relative position (X, Y, Z, Yaw, Pitch, Roll)
 *
 * This is a standalone test - no robot hardware required.
 *
 * Field Coordinate System:
 * - Origin: Center of field
 * - X-axis: Left (negative) to Right (positive) from driver station view
 * - Y-axis: Toward driver station (negative) to Away (positive)
 * - Heading: 0° = facing away from driver station
 *
 * Controls:
 * - Gamepad1 DPAD_UP: Resume camera streaming
 * - Gamepad1 DPAD_DOWN: Stop camera streaming (save CPU)
 * - Gamepad1 DPAD_LEFT: Decrease decimation (better range, slower)
 * - Gamepad1 DPAD_RIGHT: Increase decimation (faster, shorter range)
 * - Gamepad1 A: Toggle detailed detection info
 */
@TeleOp(name = "Vision Localization Test", group = "Vision")
public class VisionLocalizationTest extends LinearOpMode {

    // Vision components
    private AprilTagProcessor aprilTagProcessor;
    private VisionPortal visionPortal;

    // Camera configuration - adjust these for your robot's camera mounting
    // For testing without a robot, these represent a camera at robot center
    private static final double CAMERA_X_OFFSET = 0.0;  // inches right from robot center
    private static final double CAMERA_Y_OFFSET = 0.0;  // inches forward from robot center
    private static final double CAMERA_Z_OFFSET = 6.0;  // inches up from robot center
    private static final double CAMERA_YAW = 0.0;       // degrees (0 = forward)
    private static final double CAMERA_PITCH = -90.0;   // degrees (-90 = horizontal forward)
    private static final double CAMERA_ROLL = 0.0;      // degrees (0 = upright)

    @Override
    public void runOpMode() {
        // Initialize vision system
        initAprilTag();

        // Control variables
        boolean showDetailedInfo = false;
        boolean prevA = false;
        boolean prevDpadLeft = false;
        boolean prevDpadRight = false;
        int decimation = 2;

        // Initial telemetry
        telemetry.addLine("=== APRILTAG VISION TEST ===");
        telemetry.addLine();
        telemetry.addLine("Camera initializing...");
        telemetry.addLine();
        telemetry.addLine("Controls:");
        telemetry.addLine("  DPAD UP/DOWN: Resume/Stop streaming");
        telemetry.addLine("  DPAD LEFT/RIGHT: Adjust decimation");
        telemetry.addLine("  A: Toggle detailed info");
        telemetry.addLine();
        telemetry.addData("Status", "Ready to start");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        // Main loop
        while (opModeIsActive()) {

            // === VISION CONTROLS ===

            // Toggle detailed info
            if (gamepad1.a && !prevA) {
                showDetailedInfo = !showDetailedInfo;
            }
            prevA = gamepad1.a;

            // Control streaming
            if (gamepad1.dpad_up) {
                visionPortal.resumeStreaming();
            } else if (gamepad1.dpad_down) {
                visionPortal.stopStreaming();
            }

            // Adjust decimation
            if (gamepad1.dpad_left && !prevDpadLeft) {
                decimation = Math.max(1, decimation - 1);
                aprilTagProcessor.setDecimation(decimation);
            }
            if (gamepad1.dpad_right && !prevDpadRight) {
                decimation = Math.min(3, decimation + 1);
                aprilTagProcessor.setDecimation(decimation);
            }
            prevDpadLeft = gamepad1.dpad_left;
            prevDpadRight = gamepad1.dpad_right;

            // === PROCESS DETECTIONS ===

            List<AprilTagDetection> detections = aprilTagProcessor.getDetections();

            // === TELEMETRY ===

            telemetry.addLine("=== APRILTAG LOCALIZATION ===");
            telemetry.addData("Camera State", visionPortal.getCameraState());
            telemetry.addData("Decimation", decimation + " (DPAD L/R)");
            telemetry.addData("Tags Detected", detections.size());
            telemetry.addLine();

            // Process each detected tag
            if (detections.isEmpty()) {
                telemetry.addLine("--- NO TAGS DETECTED ---");
                telemetry.addLine("Point camera at AprilTags");
            } else {
                for (AprilTagDetection detection : detections) {
                    telemetry.addLine(String.format(Locale.US, "--- TAG ID %d ---", detection.id));

                    if (detection.metadata != null) {
                        // Tag is in library - has field position data
                        telemetry.addData("Name", detection.metadata.name);
                        telemetry.addLine();

                        // Field position data (robot position on field)
                        if (detection.ftcPose != null) {
                            telemetry.addLine("ROBOT FIELD POSITION:");
                            telemetry.addData("  X (Right)", String.format(Locale.US, "%.1f inches",
                                detection.ftcPose.x));
                            telemetry.addData("  Y (Forward)", String.format(Locale.US, "%.1f inches",
                                detection.ftcPose.y));
                            telemetry.addData("  Z (Up)", String.format(Locale.US, "%.1f inches",
                                detection.ftcPose.z));
                            telemetry.addLine();

                            telemetry.addLine("ROBOT ORIENTATION:");
                            telemetry.addData("  Yaw (Heading)", String.format(Locale.US, "%.1f°",
                                detection.ftcPose.yaw));
                            telemetry.addData("  Pitch", String.format(Locale.US, "%.1f°",
                                detection.ftcPose.pitch));
                            telemetry.addData("  Roll", String.format(Locale.US, "%.1f°",
                                detection.ftcPose.roll));
                            telemetry.addLine();

                            // Additional navigation data
                            telemetry.addLine("NAVIGATION DATA:");
                            telemetry.addData("  Range", String.format(Locale.US, "%.1f inches",
                                detection.ftcPose.range));
                            telemetry.addData("  Bearing", String.format(Locale.US, "%.1f°",
                                detection.ftcPose.bearing));
                            telemetry.addData("  Elevation", String.format(Locale.US, "%.1f°",
                                detection.ftcPose.elevation));
                        }

                        // Show detailed info if toggled
                        if (showDetailedInfo && detection.metadata.fieldPosition != null) {
                            telemetry.addLine();
                            telemetry.addLine("TAG FIELD POSITION:");
                            telemetry.addData("  Tag X", String.format(Locale.US, "%.1f inches",
                                detection.metadata.fieldPosition.get(0)));
                            telemetry.addData("  Tag Y", String.format(Locale.US, "%.1f inches",
                                detection.metadata.fieldPosition.get(1)));
                            telemetry.addData("  Tag Z", String.format(Locale.US, "%.1f inches",
                                detection.metadata.fieldPosition.get(2)));
                        }

                    } else {
                        // Tag not in library - show only camera-relative data
                        telemetry.addLine("(Not in tag library)");
                        telemetry.addData("Center", String.format(Locale.US, "%.0f, %.0f pixels",
                            detection.center.x, detection.center.y));

                        if (showDetailedInfo && detection.ftcPose != null) {
                            telemetry.addLine();
                            telemetry.addLine("CAMERA-RELATIVE:");
                            telemetry.addData("  Range", String.format(Locale.US, "%.1f inches",
                                detection.ftcPose.range));
                            telemetry.addData("  Bearing", String.format(Locale.US, "%.1f°",
                                detection.ftcPose.bearing));
                            telemetry.addData("  Elevation", String.format(Locale.US, "%.1f°",
                                detection.ftcPose.elevation));
                        }
                    }

                    telemetry.addLine();
                }
            }

            // Controls reminder
            telemetry.addLine("--- CONTROLS ---");
            telemetry.addLine("DPAD UP: Resume | DOWN: Stop");
            telemetry.addLine("DPAD L/R: Decimation");
            telemetry.addData("A: Details", showDetailedInfo ? "ON" : "OFF");

            telemetry.update();

            // Small sleep to reduce CPU load
            sleep(20);
        }

        // Clean up
        if (visionPortal != null) {
            visionPortal.close();
        }
    }

    /**
     * Initialize the AprilTag processor and vision portal
     */
    private void initAprilTag() {
        // Create camera position and orientation
        Position cameraPosition = new Position(DistanceUnit.INCH,
            CAMERA_X_OFFSET, CAMERA_Y_OFFSET, CAMERA_Z_OFFSET, 0);
        YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES,
            CAMERA_YAW, CAMERA_PITCH, CAMERA_ROLL, 0);

        // Create AprilTag processor with camera pose
        aprilTagProcessor = new AprilTagProcessor.Builder()
            .setDrawAxes(true)
            .setDrawCubeProjection(true)
            .setDrawTagOutline(true)
            .setCameraPose(cameraPosition, cameraOrientation)
            .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
            .build();

        // Set initial decimation
        aprilTagProcessor.setDecimation(2);

        // Build vision portal
        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Try to get webcam, fall back gracefully if not available
        try {
            builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        } catch (Exception e) {
            // Try built-in camera if webcam not found
            try {
                builder.setCamera(org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection.BACK);
            } catch (Exception e2) {
                telemetry.addLine("ERROR: No camera found!");
                telemetry.update();
            }
        }

        // Configure and build portal
        builder.addProcessor(aprilTagProcessor);
        builder.enableLiveView(true);
        builder.setAutoStopLiveView(false);

        visionPortal = builder.build();
    }
}

