package org.firstinspires.ftc.teamcode.common;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

/**
 * VisionLocalization - A class for determining robot position using AprilTag vision
 *
 * This class uses AprilTag detection to provide accurate field positioning for the robot.
 * It integrates with the robot's existing odometry system for sensor fusion.
 *
 * Features:
 * - Real-time AprilTag detection
 * - Camera position tracking
 * - Field coordinate system integration
 * - Multiple tag detection and averaging
 * - Confidence scoring for detections
 */
public class VisionLocalization {

    private AprilTagProcessor aprilTagProcessor;
    private VisionPortal visionPortal;

    // Camera mounting position on robot (inches from robot center)
    // Adjust these values to match your actual camera placement
    private Position cameraPosition;
    private YawPitchRollAngles cameraOrientation;

    // Latest position data
    private double robotX = 0.0;  // inches
    private double robotY = 0.0;  // inches
    private double robotHeading = 0.0;  // degrees
    private int detectedTagCount = 0;
    private long lastDetectionTime = 0;
    private boolean isInitialized = false;

    // Detection confidence
    private double detectionConfidence = 0.0;

    /**
     * Constructor with default camera position (centered, horizontal, facing forward)
     * @param hardwareMap The robot's hardware map
     */
    public VisionLocalization(HardwareMap hardwareMap) {
        this(hardwareMap, 0, 0, 0, 0, -90, 0);
    }

    /**
     * Constructor with custom camera mounting position
     * @param hardwareMap The robot's hardware map
     * @param camX Camera X offset from robot center (inches, + = right)
     * @param camY Camera Y offset from robot center (inches, + = forward)
     * @param camZ Camera Z offset from robot center (inches, + = up)
     * @param camYaw Camera yaw rotation (degrees, 0 = forward, +90 = left, -90 = right)
     * @param camPitch Camera pitch rotation (degrees, -90 = horizontal forward)
     * @param camRoll Camera roll rotation (degrees, 0 = upright)
     */
    public VisionLocalization(HardwareMap hardwareMap,
                            double camX, double camY, double camZ,
                            double camYaw, double camPitch, double camRoll) {

        // Set camera position and orientation on robot
        cameraPosition = new Position(DistanceUnit.INCH, camX, camY, camZ, 0);
        cameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES, camYaw, camPitch, camRoll, 0);

        // Initialize AprilTag processor
        initializeVision(hardwareMap);
    }

    /**
     * Initialize the vision system with AprilTag detection
     */
    private void initializeVision(HardwareMap hardwareMap) {
        try {
            // Create AprilTag processor with camera pose
            aprilTagProcessor = new AprilTagProcessor.Builder()
                    .setDrawAxes(true)
                    .setDrawCubeProjection(true)
                    .setDrawTagOutline(true)
                    .setCameraPose(cameraPosition, cameraOrientation)
                    .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                    .build();

            // Set decimation for better frame rate
            // Decimation = 1: Best detection range, slower FPS
            // Decimation = 2: Good balance (recommended)
            // Decimation = 3: Faster FPS, shorter range
            aprilTagProcessor.setDecimation(2);

            // Build vision portal
            VisionPortal.Builder builder = new VisionPortal.Builder();

            // Try to get webcam, fall back to phone camera if not available
            try {
                builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
            } catch (Exception e) {
                // Use built-in camera if webcam not found
                builder.setCamera(org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection.BACK);
            }

            // Configure portal settings
            builder.addProcessor(aprilTagProcessor);
            builder.enableLiveView(true);
            builder.setAutoStopLiveView(false);

            // Build the portal
            visionPortal = builder.build();

            isInitialized = true;

        } catch (Exception e) {
            System.err.println("VisionLocalization initialization failed: " + e.getMessage());
            isInitialized = false;
        }
    }

    /**
     * Update vision data - call this regularly in your main loop
     * This processes AprilTag detections and updates position estimates
     */
    public void update() {
        if (!isInitialized || aprilTagProcessor == null) return;

        List<AprilTagDetection> detections = aprilTagProcessor.getDetections();
        detectedTagCount = detections.size();

        if (detectedTagCount > 0) {
            // Average position from all detected tags with valid metadata
            double sumX = 0, sumY = 0, sumHeading = 0;
            int validCount = 0;

            for (AprilTagDetection detection : detections) {
                // Only use tags with metadata (known field positions)
                if (detection.metadata != null && detection.robotPose != null) {
                    // Skip obelisk tags (they're not reliable for localization)
                    if (detection.metadata.name != null && detection.metadata.name.contains("Obelisk")) {
                        continue;
                    }

                    Position pos = detection.robotPose.getPosition();
                    YawPitchRollAngles orientation = detection.robotPose.getOrientation();

                    sumX += pos.x;
                    sumY += pos.y;
                    sumHeading += orientation.getYaw(AngleUnit.DEGREES);
                    validCount++;
                }
            }

            if (validCount > 0) {
                robotX = sumX / validCount;
                robotY = sumY / validCount;
                robotHeading = sumHeading / validCount;
                lastDetectionTime = System.currentTimeMillis();

                // Calculate confidence based on number of tags and detection quality
                detectionConfidence = Math.min(1.0, validCount / 2.0);
            }
        } else {
            // No detections - decay confidence over time
            long timeSinceDetection = System.currentTimeMillis() - lastDetectionTime;
            if (timeSinceDetection > 1000) {
                detectionConfidence = Math.max(0.0, detectionConfidence - 0.1);
            }
        }
    }

    /**
     * Get the robot's X position on the field (inches)
     * @return X coordinate (positive = right side of field)
     */
    public double getRobotX() {
        return robotX;
    }

    /**
     * Get the robot's Y position on the field (inches)
     * @return Y coordinate (positive = away from driver station)
     */
    public double getRobotY() {
        return robotY;
    }

    /**
     * Get the robot's heading on the field (degrees)
     * @return Heading in degrees (0 = facing away from driver station)
     */
    public double getRobotHeading() {
        return robotHeading;
    }

    /**
     * Get number of AprilTags currently detected
     * @return Number of detected tags
     */
    public int getDetectedTagCount() {
        return detectedTagCount;
    }

    /**
     * Get confidence in current position estimate (0.0 to 1.0)
     * Higher values indicate more reliable position data
     * @return Confidence score
     */
    public double getConfidence() {
        return detectionConfidence;
    }

    /**
     * Check if vision system has detected tags recently
     * @return true if tags detected in last second
     */
    public boolean hasRecentDetection() {
        return (System.currentTimeMillis() - lastDetectionTime) < 1000;
    }

    /**
     * Get all current AprilTag detections
     * @return List of detected AprilTags
     */
    public List<AprilTagDetection> getDetections() {
        if (aprilTagProcessor == null) return null;
        return aprilTagProcessor.getDetections();
    }

    /**
     * Get detailed information about a specific detection
     * @param detection The AprilTag detection
     * @return Formatted string with detection details
     */
    public String getDetectionInfo(AprilTagDetection detection) {
        if (detection == null) return "No detection";

        StringBuilder info = new StringBuilder();
        info.append(String.format("Tag ID: %d", detection.id));

        if (detection.metadata != null) {
            info.append(String.format(" (%s)", detection.metadata.name));

            if (detection.robotPose != null) {
                Position pos = detection.robotPose.getPosition();
                YawPitchRollAngles orient = detection.robotPose.getOrientation();

                info.append(String.format("\nPos: X=%.1f\" Y=%.1f\" Z=%.1f\"",
                    pos.x, pos.y, pos.z));
                info.append(String.format("\nYaw=%.1f° Pitch=%.1f° Roll=%.1f°",
                    orient.getYaw(AngleUnit.DEGREES),
                    orient.getPitch(AngleUnit.DEGREES),
                    orient.getRoll(AngleUnit.DEGREES)));
            }
        }

        return info.toString();
    }

    /**
     * Enable or disable the AprilTag processor
     * @param enabled true to enable, false to disable
     */
    public void setEnabled(boolean enabled) {
        if (visionPortal != null && aprilTagProcessor != null) {
            visionPortal.setProcessorEnabled(aprilTagProcessor, enabled);
        }
    }

    /**
     * Stop camera streaming to save CPU resources
     */
    public void stopStreaming() {
        if (visionPortal != null) {
            visionPortal.stopStreaming();
        }
    }

    /**
     * Resume camera streaming
     */
    public void resumeStreaming() {
        if (visionPortal != null) {
            visionPortal.resumeStreaming();
        }
    }

    /**
     * Check if vision system is initialized and ready
     * @return true if ready to use
     */
    public boolean isReady() {
        return isInitialized && visionPortal != null
            && visionPortal.getCameraState() == VisionPortal.CameraState.STREAMING;
    }

    /**
     * Get the camera state
     * @return Camera state string
     */
    public String getCameraState() {
        if (visionPortal == null) return "NOT_INITIALIZED";
        return visionPortal.getCameraState().toString();
    }

    /**
     * Close the vision portal and release resources
     * Call this when shutting down
     */
    public void close() {
        if (visionPortal != null) {
            visionPortal.close();
        }
    }

    /**
     * Set camera decimation (trade detection range for speed)
     * @param decimation 1 = best range/slow, 2 = balanced, 3 = fast/short range
     */
    public void setDecimation(int decimation) {
        if (aprilTagProcessor != null) {
            aprilTagProcessor.setDecimation(Math.max(1, Math.min(3, decimation)));
        }
    }
}

