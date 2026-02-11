package org.firstinspires.ftc.teamcode.Trowel.Drive;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import org.firstinspires.ftc.teamcode.Trowel.common.Odometry;
import org.firstinspires.ftc.teamcode.Trowel.Configs.TrowelHardware;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import java.util.List;
import java.util.ArrayList;

@Configurable
@TeleOp(name = "Trowel Teleop", group = "Trowel")
public class TrowelTeleOp extends OpMode {

    // ══════════════════════════════════════════════════════════════════════════
    // UNIFIED TURNING CONSTANTS - MATCHES PEDRO PATHING headingPIDFCoefficients
    // From Constants.java: P=1.5, I=0, D=0.06, F=0
    // These values work in autonomous without oscillation.
    // ══════════════════════════════════════════════════════════════════════════

    /** P gain for turning - MATCHES Pedro Pathing heading P (1.5 in PID, scaled for power output) */
    public static final double TURN_P = 0.025;  // Scaled from Pedro's 1.5 for motor power

    /** I gain for turning - Pedro Pathing uses 0 */
    public static final double TURN_I = 0.0;

    /** D gain for turning - MATCHES Pedro Pathing heading D (0.06 scaled) */
    public static final double TURN_D = 0.002;

    /** Maximum turn power for all aiming modes */
    public static final double TURN_MAX_POWER = 0.5;

    /** Minimum turn power to overcome static friction */
    public static final double TURN_MIN_POWER = 0.05;

    /** Deadzone in degrees - no correction if error smaller than this */
    public static final double TURN_DEADZONE_DEG = 2.0;

    /** Threshold to switch from coarse (odo) to fine (vision) aiming */
    public static final double COARSE_TO_FINE_THRESHOLD_DEG = 8.0;

    // ══════════════════════════════════════════════════════════════════════════
    // ABSOLUTE FIELD COORDINATE SYSTEM
    // (0,0) = bottom-left corner of field when viewed from above
    // X increases to the right (toward red alliance side)
    // Y increases upward (toward back wall from driver station)
    // All positions use this absolute coordinate system regardless of alliance
    // ══════════════════════════════════════════════════════════════════════════

    // ══════════════════════════════════════════════════════════════════════════
    // APRILTAG FIELD POSITIONS - ABSOLUTE COORDINATES
    // ══════════════════════════════════════════════════════════════════════════

    /** Blue AprilTag field position (in inches) - ABSOLUTE */
    public static final double BLUE_TAG_X = 16.079207920792072;
    public static final double BLUE_TAG_Y = 131.32673267326734;

    /** Red AprilTag field position (in inches) - ABSOLUTE */
    public static final double RED_TAG_X = 129.42574257425744;
    public static final double RED_TAG_Y = 131.326;

    // ══════════════════════════════════════════════════════════════════════════
    // GOAL COORDINATES - ABSOLUTE FIELD COORDINATES
    // ══════════════════════════════════════════════════════════════════════════

    /** Blue alliance goal coordinates (in inches) - ABSOLUTE */
    public static final double BLUE_GOAL_X = 10.73267326732673;
    public static final double BLUE_GOAL_Y = 136.13861386138615;

    /** Red alliance goal coordinates (in inches) - ABSOLUTE */
    public static final double RED_GOAL_X = 130.85148514851485;
    public static final double RED_GOAL_Y = 137.38613861386142;

    // ══════════════════════════════════════════════════════════════════════════
    // ALLIANCE SELECTION
    // ══════════════════════════════════════════════════════════════════════════

    public enum Alliance { NONE, RED, BLUE }
    private Alliance selectedAlliance = Alliance.NONE;

    // ══════════════════════════════════════════════════════════════════════════
    // BROWNOUT PROTECTION
    // ══════════════════════════════════════════════════════════════════════════

    public static final double VOLTAGE_BROWNOUT_THRESHOLD = 8.5;
    public static final double VOLTAGE_RECOVERY_THRESHOLD = 10.0;
    public static final double BROWNOUT_DRIVE_SCALE = 0.8;
    public static final double BROWNOUT_INTAKE_SCALE = 0.33;
    public static final int VOLTAGE_SAMPLE_COUNT = 5;

    // ══════════════════════════════════════════════════════════════════════════
    // FLYWHEEL CONFIGURATION - EXACT TICK SPEED CONTROL
    // The deposit always runs at EXACTLY the commanded tick speed.
    // Only recovery P gain varies; the target velocity never drifts.
    // ══════════════════════════════════════════════════════════════════════════

    public static final double FLYWHEEL_DEFAULT_TARGET = 525.0;
    public static final double FLYWHEEL_MIN = 300.0;
    public static final double FLYWHEEL_MAX = 800.0;
    public static final double FLYWHEEL_STEP = 5.0;
    public static final double SHOOT_BOOST = 40.0;

    /** Normal PIDF coefficients for steady-state velocity */
    public static final double PIDF_P = 240.0;
    public static final double PIDF_I = 0.0;
    public static final double PIDF_D = 0.0;
    public static final double PIDF_F = 23.4;

    /** Recovery P gain - higher for faster recovery after shooting */
    public static final double RECOVERY_P = 38.0;
    public static final double RECOVERY_THRESHOLD = 40.0;
    public static final double RECOVERY_EXIT = 15.0;

    // ══════════════════════════════════════════════════════════════════════════
    // DRIVE CONFIGURATION
    // ══════════════════════════════════════════════════════════════════════════

    public static final double SLOW_MODE_MULTIPLIER = 0.4;
    public static final double DRIVE_DEADZONE = 0.05;

    // ══════════════════════════════════════════════════════════════════════════
    // CAMERA MOUNT CONFIGURATION
    // ══════════════════════════════════════════════════════════════════════════

    public static final double CAMERA_X_OFFSET_INCHES = 5.5;
    public static final double CAMERA_Y_OFFSET_INCHES = 0.0;
    public static final double CAMERA_Z_OFFSET_INCHES = 10.5;
    public static final double CAMERA_PITCH_DEG = 0.28;
    public static final double CAMERA_YAW_DEG = 0.0;
    public static final double CAMERA_ROLL_DEG = 0.0;
    public static final String WEBCAM_NAME = "Webcam 1";

    /** Which AprilTag IDs to target (null = any) */
    public static final int[] TARGET_TAG_IDS = null;
    public static final boolean AIM_CLOSEST_TAG = true;

    // ══════════════════════════════════════════════════════════════════════════
    // ADAPTIVE POSE FUSION CONFIGURATION
    // Camera periodically corrects odometry; fusion weights adapt based on accuracy
    // ══════════════════════════════════════════════════════════════════════════

    public static final double POSE_CORRECTION_INTERVAL_SEC = 0.5;
    public static final double POSE_CORRECTION_MIN_RANGE = 6.0;
    public static final double POSE_CORRECTION_MAX_RANGE = 72.0;
    public static final boolean POSE_CORRECTION_ENABLED = true;

    /** Initial weight for camera pose (0-1), remainder is odometry weight */
    private double cameraFusionWeight = 0.7;

    /** How much to adjust fusion weight based on observed accuracy */
    private static final double FUSION_WEIGHT_ADJUST_RATE = 0.05;

    // ══════════════════════════════════════════════════════════════════════════
    // SERVO & INTAKE CONFIGURATION
    // ══════════════════════════════════════════════════════════════════════════

    public static final double SERVO_IDLE = 0.3;
    public static final double SERVO_SHOOT = 0.95;
    public static final double INTAKE_POWER = 1.0;
    public static final boolean DEPOSIT1_REVERSED = false;
    public static final boolean DEPOSIT2_REVERSED = false;

    // ══════════════════════════════════════════════════════════════════════════
    // STATE VARIABLES
    // ══════════════════════════════════════════════════════════════════════════

    private TrowelHardware robot;
    private Odometry odometry;
    private final ElapsedTime loopTimer = new ElapsedTime();

    private boolean odoEnabled = false;
    private boolean flywheelOn = false;
    private boolean inRecoveryMode = false;
    private double flywheelTarget = FLYWHEEL_DEFAULT_TARGET;

    private boolean prevFlywheelToggle = false;
    private boolean prevDpadUp = false;
    private boolean prevDpadDown = false;

    private double lastLoopTime = 0.0;
    private double lastP = -1;

    private double vel1 = 0, vel2 = 0, avgVel = 0, commandedTarget = 0;
    private double flPower = 0, frPower = 0, blPower = 0, brPower = 0;
    private double intake1Power = 0, intake2Power = 0;
    private double loopTimeMs = 0;
    private boolean isSlowMode = false;

    // Brownout state
    private VoltageSensor voltageSensor;
    private double[] voltageSamples;
    private int voltageSampleIndex = 0;
    private double smoothedVoltage = 12.0;
    private boolean inBrownout = false;

    // AprilTag auto-aim state
    private boolean aimLockActive = false;
    private double aimCorrectionPower = 0.0;
    private double aimErrorDeg = 0.0;
    private boolean tagDetected = false;
    private int detectedTagId = -1;
    private double detectedTagRange = 0.0;
    private double detectedTagBearing = 0.0;
    private double detectedTagYaw = 0.0;
    private int totalTagsVisible = 0;

    // Coarse aiming state (odometry-based)
    private boolean coarseAimActive = false;
    private double coarseAimErrorDeg = 0.0;
    private double coarseAimCorrectionPower = 0.0;
    private double targetHeadingRad = 0.0;

    // PID state for turning (shared across all aim modes)
    private double prevTurnErrorDeg = 0.0;
    private double turnErrorIntegral = 0.0;

    // Aiming phase state machine
    public enum AimPhase { IDLE, COARSE_SNAP, FINE_AIM }
    private AimPhase currentAimPhase = AimPhase.IDLE;

    // Initial pose state - camera establishes first pose before trusting odometry
    private boolean initialPoseEstablished = false;
    private int initialPoseAttempts = 0;
    private static final int MAX_INITIAL_POSE_ATTEMPTS = 100;

    // Pose correction state
    private double lastPoseCorrectionTime = 0.0;
    private int poseCorrectionCount = 0;

    // ══════════════════════════════════════════════════════════════════════════
    // ADAPTIVE AIMING SYSTEM - Self-improving through hypothesis testing
    // The system tracks aiming errors and adjusts parameters to improve accuracy
    // ══════════════════════════════════════════════════════════════════════════

    /** Current aim offset hypothesis being tested (degrees) */
    private double aimOffsetHypothesis = 0.0;

    /** Best known aim offset that produced lowest error */
    private double bestAimOffset = 0.0;

    /** Lowest average error achieved with best offset */
    private double bestAverageError = Double.MAX_VALUE;

    /** Rolling window of recent aiming errors for evaluation */
    private final ArrayList<Double> recentAimErrors = new ArrayList<>();
    private static final int AIM_ERROR_WINDOW_SIZE = 20;

    /** How much to vary the offset when testing hypotheses */
    private static final double HYPOTHESIS_STEP_DEG = 0.5;

    /** Number of samples needed before evaluating a hypothesis */
    private static final int HYPOTHESIS_EVAL_SAMPLES = 10;

    /** Counter for current hypothesis test */
    private int hypothesisSampleCount = 0;

    /** Direction of hypothesis exploration (-1, 0, 1) */
    private int hypothesisDirection = 0;

    // Vision-odometry fusion tracking
    private double lastCameraPoseX = 0, lastCameraPoseY = 0, lastCameraPoseHeading = 0;
    private double lastOdoPoseX = 0, lastOdoPoseY = 0, lastOdoPoseHeading = 0;
    private double cameraOdoDivergence = 0;

    // ══════════════════════════════════════════════════════════════
    // INITIALIZATION
    // ══════════════════════════════════════════════════════════════

    @Override
    public void init() {
        robot = new TrowelHardware(hardwareMap);
        loopTimer.reset();

        initVoltageSensor();
        initDriveMotors();
        initFlywheelMotors(PIDF_P);
        initOdometry();

        // Initialize vision - camera will establish initial pose before odometry is trusted
        boolean vOK = robot.initVision(WEBCAM_NAME,
                CAMERA_X_OFFSET_INCHES, CAMERA_Y_OFFSET_INCHES, CAMERA_Z_OFFSET_INCHES,
                CAMERA_YAW_DEG, CAMERA_PITCH_DEG, CAMERA_ROLL_DEG);

        // Reset initial pose state - camera must establish first pose
        initialPoseEstablished = false;
        initialPoseAttempts = 0;

        telemetry.addData("Vision Init", vOK ? "OK" : robot.getVisionInitError());
        telemetry.addData("Pose Status", "Waiting for camera to establish initial position");
        telemetry.update();

        if (robot.transferServo != null) {
            robot.transferServo.setPosition(SERVO_IDLE);
        }

        showControls();
    }

    @Override
    public void init_loop() {
        // Alliance selection using gamepad buttons
        if (gamepad1.x || gamepad2.x) {
            selectedAlliance = Alliance.BLUE;
        } else if (gamepad1.b || gamepad2.b) {
            selectedAlliance = Alliance.RED;
        }

        // Attempt to establish initial pose from camera if not yet done
        if (!initialPoseEstablished && robot.isVisionEnabled()) {
            attemptInitialPoseFromCamera();
        }

        // Show alliance selection
        telemetry.addLine("══════════════════════════════");
        telemetry.addLine("     ALLIANCE SELECTION");
        telemetry.addLine("══════════════════════════════");
        telemetry.addLine("");
        String allianceStr;
        switch (selectedAlliance) {
            case RED:   allianceStr = "🔴 RED ALLIANCE"; break;
            case BLUE:  allianceStr = "🔵 BLUE ALLIANCE"; break;
            default:    allianceStr = "⬜ NOT SELECTED"; break;
        }
        telemetry.addData("Current Selection", allianceStr);
        telemetry.addLine("");
        telemetry.addLine("Press X for BLUE  |  Press B for RED");
        telemetry.addLine("");

        // Initial pose status
        if (initialPoseEstablished) {
            telemetry.addData("Pose Status", "✓ Initial pose established from camera");
            if (odometry != null) {
                Odometry.Position p = odometry.getPosition();
                telemetry.addData("Odometry Reports", "X: %.1f in, Y: %.1f in, H: %.1f°",
                        p.xMm / 25.4, p.yMm / 25.4, Math.toDegrees(p.headingRad));
            }
            if (lastCameraPoseX != 0 || lastCameraPoseY != 0) {
                telemetry.addData("Camera Calculated", "X: %.1f in, Y: %.1f in, H: %.1f°",
                        lastCameraPoseX, lastCameraPoseY, lastCameraPoseHeading);
            }
        } else {
            telemetry.addData("Pose Status", "⏳ Waiting for AprilTag... (%d attempts)", initialPoseAttempts);
            telemetry.addLine("Point camera at AprilTag to establish position");
        }

        // Show AprilTag detections during init
        if (robot.isVisionEnabled()) {
            List<AprilTagDetection> detections = robot.getDetections();
            telemetry.addData("AprilTags Visible", detections.size());
            for (AprilTagDetection det : detections) {
                if (det.metadata != null) {
                    telemetry.addData("  Tag " + det.id, "%.1f\" away, bearing %.1f°",
                            det.ftcPose.range, det.ftcPose.bearing);
                } else {
                    telemetry.addData("  Tag " + det.id, "(unknown size — no metadata)");
                }
            }
        }
        telemetry.update();
    }

    /**
     * Attempts to establish the robot's initial pose using camera/AprilTag detection.
     * This MUST succeed before odometry-based aiming can work correctly.
     * 
     * Uses ABSOLUTE FIELD COORDINATES where (0,0) is bottom-left of field.
     * 
     * The AprilTag SDK's robotPose SHOULD give field coordinates, but sometimes
     * returns invalid values. In that case, we calculate from:
     * - The known absolute position of the AprilTag on the field
     * - The range and bearing from camera to tag (ftcPose)
     * - The robot's heading
     */
    private void attemptInitialPoseFromCamera() {
        initialPoseAttempts++;

        List<AprilTagDetection> detections = robot.getDetections();
        for (AprilTagDetection det : detections) {
            if (det.robotPose != null && det.ftcPose != null) {
                try {
                    // Get robot's field position from AprilTag SDK
                    org.firstinspires.ftc.robotcore.external.navigation.Position robotPos = det.robotPose.getPosition();
                    org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles robotYpr = det.robotPose.getOrientation();

                    double xInches = robotPos.toUnit(DistanceUnit.INCH).x;
                    double yInches = robotPos.toUnit(DistanceUnit.INCH).y;
                    double headingDeg = robotYpr.getYaw(AngleUnit.DEGREES);

                    // Check if SDK-provided coordinates are valid (on the field)
                    // Field is 144" x 144" (12ft x 12ft)
                    boolean needsRecalculation = (xInches < 0 || xInches > 144 || yInches < 0 || yInches > 144);

                    if (needsRecalculation) {
                        // SDK coordinates are invalid - calculate from ftcPose + known tag position
                        
                        // Get the tag's known ABSOLUTE field position based on tag ID
                        double tagFieldX, tagFieldY;
                        
                        // Into the Deep season tag IDs (adjust if different)
                        // Blue side tags: typically lower IDs
                        // Red side tags: typically higher IDs
                        if (det.id >= 11 && det.id <= 14) {
                            // Blue alliance wall tags
                            tagFieldX = BLUE_TAG_X;
                            tagFieldY = BLUE_TAG_Y;
                        } else if (det.id >= 15 && det.id <= 16) {
                            // Red alliance wall tags
                            tagFieldX = RED_TAG_X;
                            tagFieldY = RED_TAG_Y;
                        } else {
                            // Unknown tag ID - try to use tag metadata if available
                            if (det.metadata != null && det.metadata.fieldPosition != null) {
                                tagFieldX = det.metadata.fieldPosition.get(0) / 25.4; // mm to inches
                                tagFieldY = det.metadata.fieldPosition.get(1) / 25.4;
                            } else {
                                continue; // Can't determine tag position, skip
                            }
                        }
                        
                        // ftcPose gives us:
                        // - range: distance from camera to tag center (inches)
                        // - bearing: horizontal angle from camera center to tag (degrees, positive = right)
                        // - yaw: rotation of tag face relative to camera (degrees)
                        
                        double range = det.ftcPose.range;
                        double bearingRad = Math.toRadians(det.ftcPose.bearing);
                        double robotHeadingRad = Math.toRadians(headingDeg);
                        
                        // The direction FROM the robot TO the tag in field coordinates
                        // Robot heading is where the robot front points
                        // Bearing is the angle from robot front to the tag
                        // So the absolute angle from robot to tag = robotHeading + bearing
                        double angleRobotToTag = robotHeadingRad + bearingRad;
                        
                        // Robot position = Tag position - (vector from robot to tag)
                        // Vector from robot to tag has magnitude=range and angle=angleRobotToTag
                        xInches = tagFieldX - range * Math.cos(angleRobotToTag);
                        yInches = tagFieldY - range * Math.sin(angleRobotToTag);
                    }

                    // Final validation - coordinates should be on or near the 144"x144" field
                    if (xInches < -10 || xInches > 154 || yInches < -10 || yInches > 154) {
                        continue; // Invalid, try next tag
                    }

// Set this as the initial pose in Pinpoint (ABSOLUTE COORDINATES)
                    if (robot.pinpoint != null) {
                        // Create the pose with ABSOLUTE field coordinates
                        Pose2D newPose = new Pose2D(DistanceUnit.INCH, xInches, yInches,
                                                    AngleUnit.DEGREES, headingDeg);

                        // CRITICAL: Set the position in Pinpoint
                        // This should override any previous odometry tracking
                        robot.pinpoint.setPosition(newPose);

                        // Don't call robot.updatePinpoint() here - it might interfere
                        // Just let the odometry wrapper read the position we just set
                        if (odometry != null) {
                            odometry.update();
                        }

                        initialPoseEstablished = true;
                        poseCorrectionCount = 1;

                        // Store the calculated position for verification
                        lastCameraPoseX = xInches;
                        lastCameraPoseY = yInches;
                        lastCameraPoseHeading = headingDeg;
                    }
                    break; // Successfully set pose, exit loop
                } catch (Exception e) {
                    // Failed to set pose, will retry
                }
            }
        }

        // If we've tried too many times without success, set a default starting position
        if (!initialPoseEstablished && initialPoseAttempts > MAX_INITIAL_POSE_ATTEMPTS) {
            if (robot.pinpoint != null && selectedAlliance != Alliance.NONE) {
                double defaultX, defaultY, defaultHeading;
                if (selectedAlliance == Alliance.BLUE) {
                    // Blue alliance typical starting position (ABSOLUTE COORDINATES)
                    defaultX = 24.0;
                    defaultY = 120.0;
                    defaultHeading = 90.0; // Facing +Y (toward back wall)
                } else {
                    // Red alliance typical starting position (ABSOLUTE COORDINATES)
                    defaultX = 120.0;
                    defaultY = 120.0;
                    defaultHeading = 90.0; // Facing +Y (toward back wall)
                }

                Pose2D defaultPose = new Pose2D(DistanceUnit.INCH, defaultX, defaultY,
                                                AngleUnit.DEGREES, defaultHeading);
                robot.pinpoint.setPosition(defaultPose);

                if (odometry != null) {
                    robot.updatePinpoint();
                    odometry.update();
                }
            }
            initialPoseEstablished = true;
        }
    }

    private void initVoltageSensor() {
        voltageSensor = null;
        double maxVoltage = 0;
        for (VoltageSensor sensor : hardwareMap.voltageSensor) {
            double v = sensor.getVoltage();
            if (v > maxVoltage) {
                maxVoltage = v;
                voltageSensor = sensor;
            }
        }

        voltageSamples = new double[VOLTAGE_SAMPLE_COUNT];
        for (int i = 0; i < VOLTAGE_SAMPLE_COUNT; i++) {
            voltageSamples[i] = maxVoltage > 0 ? maxVoltage : 12.0;
        }
        smoothedVoltage = maxVoltage > 0 ? maxVoltage : 12.0;
    }

    private void showControls() {
        telemetry.addLine("══════════════════════════════");
        telemetry.addLine("    RANDY BUTTER KNUBS");
        telemetry.addLine("══════════════════════════════");
        telemetry.addLine("");
        telemetry.addLine("INIT - ALLIANCE SELECT:");
        telemetry.addLine("  X: Select BLUE  |  B: Select RED");
        telemetry.addLine("");
        telemetry.addLine("DRIVER 1 (Gamepad 1):");
        telemetry.addLine("  Left Stick: Move");
        telemetry.addLine("  Right Stick X: Turn");
        telemetry.addLine("  RB hold: Slow mode (40%)");
        telemetry.addLine("  LB hold: Auto-Aim (odo→camera)");
        telemetry.addLine("  LT: Shoot");
        telemetry.addLine("");
        telemetry.addLine("DRIVER 2 (Gamepad 2):");
        telemetry.addLine("  X: Toggle flywheel");
        telemetry.addLine("  Dpad Up/Down: Adjust flywheel speed");
        telemetry.addLine("  LT: Intake in  |  RT: Intake reverse");
        telemetry.addLine("  A: Intake2 in  |  B: Intake2 reverse");
        telemetry.addLine("");
        telemetry.addData("Battery", "%.2fV", smoothedVoltage);
        telemetry.addData("Vision", robot.getVisionStatusString());
        telemetry.addData("Camera Pitch", "%.1f° from horizontal", CAMERA_PITCH_DEG);
        telemetry.addData("Camera Offset", "X=%.1f Y=%.1f Z=%.1f inches",
                CAMERA_X_OFFSET_INCHES, CAMERA_Y_OFFSET_INCHES, CAMERA_Z_OFFSET_INCHES);
        telemetry.update();
    }

    private void initDriveMotors() {
        for (DcMotor m : new DcMotor[]{robot.frontLeft, robot.frontRight, robot.backLeft, robot.backRight}) {
            if (m != null) {
                m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                m.setPower(0);
            }
        }
    }

    private void initFlywheelMotors(double pValue) {
        try {
            for (DcMotor m : new DcMotor[]{robot.deposit1, robot.deposit2}) {
                if (m instanceof DcMotorEx) {
                    DcMotorEx motor = (DcMotorEx) m;
                    motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    motor.setVelocityPIDFCoefficients(pValue, PIDF_I, PIDF_D, PIDF_F);
                    motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                }
            }
            lastP = pValue;
        } catch (Exception ignored) {}
    }

    private void initOdometry() {
        try {
            robot.initPinpoint();
            if (robot.pinpoint != null) {
                odoEnabled = true;
                robot.updatePinpoint();
                odometry = new Odometry(robot, robot.pinpoint);
            } else {
                odometry = new Odometry(robot, null);
            }
        } catch (Exception e) {
            odometry = new Odometry(robot, null);
        }
    }

    // Pedro Pathing follower initialization removed to avoid multiple Pinpoint access
    // Coarse aiming uses simple P controller directly on drive motors

    // ══════════════════════════════════════════════════════════════
    // START / STOP
    // ══════════════════════════════════════════════════════════════

    @Override
    public void start() {
        loopTimer.reset();
        lastLoopTime = 0;
        flywheelTarget = FLYWHEEL_DEFAULT_TARGET;

        // Reset adaptive aiming state
        aimOffsetHypothesis = 0;
        bestAimOffset = 0;
        bestAverageError = Double.MAX_VALUE;
        recentAimErrors.clear();
        hypothesisSampleCount = 0;
        hypothesisDirection = 0;
    }

    @Override
    public void stop() {
        for (DcMotor m : new DcMotor[]{robot.frontLeft, robot.frontRight, robot.backLeft, robot.backRight}) {
            if (m != null) m.setPower(0);
        }
        if (robot.deposit1 != null) robot.deposit1.setPower(0);
        if (robot.deposit2 != null) robot.deposit2.setPower(0);
        robot.stop();

        // Clean up vision
        robot.closeVision();
    }

    // ══════════════════════════════════════════════════════════════
    // MAIN LOOP
    // ══════════════════════════════════════════════════════════════

    @Override
    public void loop() {
        double currentTime = loopTimer.seconds();
        double deltaTime = Math.min(currentTime - lastLoopTime, 0.1);
        loopTimeMs = deltaTime * 1000.0;
        lastLoopTime = currentTime;

        updateVoltage();

        // Track if we just set the position this cycle
        boolean justSetPosition = false;

        if (odoEnabled && robot.pinpoint != null) {
            robot.updatePinpoint();
            odometry.update();
        }

        updateAprilTagDetection();

        // If initial pose not established, keep trying from camera
        // This is critical - without initial pose, odometry is at (0,0) and aiming fails
        if (!initialPoseEstablished && robot.isVisionEnabled()) {
            attemptInitialPoseFromCamera();
            // If we just set the position, mark it
            if (initialPoseEstablished && poseCorrectionCount == 1) {
                justSetPosition = true;
            }
        }

        // Also update pose from camera periodically for drift correction
        // Skip if we just set position to avoid overwriting
        if (!justSetPosition) {
            updateAdaptivePoseFusion();
        }

        updateAimStateMachine();
        updateAdaptiveAiming();
        updateDrive();
        updateFlywheelTarget();
        updateFlywheel(deltaTime);
        updateIntake();
        updateServo();
        updateTelemetry();
    }

    // ══════════════════════════════════════════════════════════════
    // VOLTAGE MONITORING
    // ══════════════════════════════════════════════════════════════

    private void updateVoltage() {
        if (voltageSensor != null) {
            double rawVoltage = voltageSensor.getVoltage();
            voltageSamples[voltageSampleIndex] = rawVoltage;
            voltageSampleIndex = (voltageSampleIndex + 1) % VOLTAGE_SAMPLE_COUNT;

            double sum = 0;
            for (double sample : voltageSamples) sum += sample;
            smoothedVoltage = sum / VOLTAGE_SAMPLE_COUNT;
        }

        if (smoothedVoltage < VOLTAGE_BROWNOUT_THRESHOLD) {
            inBrownout = true;
        } else if (smoothedVoltage > VOLTAGE_RECOVERY_THRESHOLD) {
            inBrownout = false;
        }
    }

    // ══════════════════════════════════════════════════════════════
    // APRILTAG DETECTION & AUTO-AIM
    // ══════════════════════════════════════════════════════════════

    /**
     * Reads the latest AprilTag detections from the camera and selects
     * the best target tag for auto-aim.
     */
    private void updateAprilTagDetection() {
        tagDetected = false;
        detectedTagId = -1;
        detectedTagRange = 0;
        detectedTagBearing = 0;
        detectedTagYaw = 0;
        totalTagsVisible = 0;

        if (!robot.isVisionEnabled()) {
            return;
        }

        List<AprilTagDetection> detections = robot.getDetections();
        totalTagsVisible = detections.size();

        if (detections.isEmpty()) {
            return;
        }

        // Find the best target tag
        AprilTagDetection bestTag = null;
        double bestRange = Double.MAX_VALUE;

        for (AprilTagDetection detection : detections) {
            // Skip detections without pose data
            if (detection.ftcPose == null) {
                continue;
            }

            // Check if this tag ID is in our target list (if filtering is enabled)
            if (TARGET_TAG_IDS != null && TARGET_TAG_IDS.length > 0) {
                boolean isTarget = false;
                for (int targetId : TARGET_TAG_IDS) {
                    if (detection.id == targetId) {
                        isTarget = true;
                        break;
                    }
                }
                if (!isTarget) {
                    continue;
                }
            }

            // If aiming at closest, track by range; otherwise take first valid
            if (AIM_CLOSEST_TAG) {
                if (detection.ftcPose.range < bestRange) {
                    bestRange = detection.ftcPose.range;
                    bestTag = detection;
                }
            } else {
                bestTag = detection;
                break;
            }
        }

        if (bestTag != null && bestTag.ftcPose != null) {
            tagDetected = true;
            detectedTagId = bestTag.id;
            detectedTagRange = bestTag.ftcPose.range;
            detectedTagBearing = bestTag.ftcPose.bearing;
            detectedTagYaw = bestTag.ftcPose.yaw;
        }
    }

    /**
     * Computes the turn correction needed to center the robot on the detected
     * AprilTag. Uses the same P control as coarse aiming for consistency.
     *
     * The bearing from ftcPose is the horizontal angle from the camera's center
     * to the tag. Positive bearing = tag is to the right of center.
     *
     * SIGN CONVENTION (same as coarse aim):
     * - Positive bearing = tag is to the right = need to turn right (clockwise)
     * - Positive motor power = robot turns right
     * - So we use POSITIVE P * error (no negation needed for bearing)
     */
    private double computeAprilTagAimCorrection() {
        if (!tagDetected) {
            aimErrorDeg = 0;
            aimCorrectionPower = 0;
            return 0;
        }

        // The bearing is the angle from camera center to the tag in degrees
        double errorDeg = detectedTagBearing;
        aimErrorDeg = errorDeg;

        if (Math.abs(errorDeg) < TURN_DEADZONE_DEG) {
            aimCorrectionPower = 0;
            recordAimError(Math.abs(errorDeg));
            return 0;
        }

        // Simple P controller - positive bearing needs positive turn (turn right)
        double correction = TURN_P * errorDeg;

        // D term for damping
        if (loopTimeMs > 0 && TURN_D > 0) {
            double errorRate = (errorDeg - prevTurnErrorDeg) / (loopTimeMs / 1000.0);
            correction += TURN_D * errorRate;
        }

        // Clamp
        correction = Math.max(-TURN_MAX_POWER, Math.min(TURN_MAX_POWER, correction));

        // Apply minimum power
        if (Math.abs(correction) < TURN_MIN_POWER && Math.abs(errorDeg) > TURN_DEADZONE_DEG) {
            correction = Math.signum(correction) * TURN_MIN_POWER;
        }

        prevTurnErrorDeg = errorDeg;
        aimCorrectionPower = correction;
        recordAimError(Math.abs(errorDeg));

        return correction;
    }

    // ══════════════════════════════════════════════════════════════
    // AIMING STATE MACHINE (ODOMETRY → CAMERA)
    // Odometry gets robot close enough for camera to see tag,
    // then camera takes over for precise aiming.
    // ══════════════════════════════════════════════════════════════

    /**
     * Manages the aiming system:
     * 1. COARSE_SNAP: Use odometry to turn toward the goal (just to get camera in range)
     * 2. FINE_AIM: Camera/AprilTag takes over for precise aiming (PRIMARY)
     *
     * CAMERA IS PRIMARY - as soon as a tag is detected, camera takes over!
     * Odometry is only used to get the robot pointed roughly in the right direction.
     *
     * Activated when LB is held and alliance is selected.
     */
    private void updateAimStateMachine() {
        boolean aimButtonHeld = gamepad1.left_bumper;

        // Reset to idle if button released or no alliance selected
        if (!aimButtonHeld || selectedAlliance == Alliance.NONE) {
            currentAimPhase = AimPhase.IDLE;
            coarseAimActive = false;
            aimLockActive = false;
            prevTurnErrorDeg = 0;
            return;
        }

        // Calculate target heading based on alliance and current position (for coarse aim)
        computeTargetHeading();

        // CAMERA TAKES PRIORITY - if we see a tag, immediately switch to camera aiming!
        // This is the main aiming mode - odometry is just to get us close enough
        if (tagDetected && robot.isVisionEnabled()) {
            if (currentAimPhase != AimPhase.FINE_AIM) {
                currentAimPhase = AimPhase.FINE_AIM;
                coarseAimActive = false;
                aimLockActive = true;
            }
            return; // Camera is handling it, no need for state machine logic
        }

        // No tag visible - use odometry to turn toward the goal
        // This should get the camera pointed at the AprilTag
        switch (currentAimPhase) {
            case IDLE:
                // Start with coarse snap (odometry-based)
                currentAimPhase = AimPhase.COARSE_SNAP;
                coarseAimActive = true;
                aimLockActive = false;
                break;

            case COARSE_SNAP:
                // Keep coarse aiming until camera sees a tag
                // The state machine will switch to FINE_AIM automatically when tag is detected
                coarseAimActive = true;
                aimLockActive = false;
                break;

            case FINE_AIM:
                // We lost the tag - fall back to coarse aim
                currentAimPhase = AimPhase.COARSE_SNAP;
                coarseAimActive = true;
                aimLockActive = false;
                break;
        }
    }

    /**
     * Computes the target heading (in radians) to face the alliance-specific goal
     * based on current robot position from odometry.
     *
     * The robot should turn to FACE the goal, meaning the front of the robot
     * points toward the goal coordinates.
     *
     * Pinpoint/Pedro Pathing coordinate system:
     * - Heading 0 = facing +X (right side of field)
     * - Heading 90° = facing +Y (forward from driver station)
     * - Heading increases COUNTER-CLOCKWISE
     */
    private void computeTargetHeading() {
        if (selectedAlliance == Alliance.NONE || odometry == null) {
            targetHeadingRad = Math.toRadians(90.0);
            coarseAimErrorDeg = 0;
            return;
        }

        // Get current position in inches from odometry
        Odometry.Position pos = odometry.getPosition();
        double robotXInches = pos.xMm / 25.4;
        double robotYInches = pos.yMm / 25.4;
        double robotHeadingRad = pos.headingRad;

        // Get goal coordinates based on alliance
        double goalX, goalY;
        if (selectedAlliance == Alliance.BLUE) {
            goalX = BLUE_GOAL_X;
            goalY = BLUE_GOAL_Y;
        } else {
            goalX = RED_GOAL_X;
            goalY = RED_GOAL_Y;
        }

        // Calculate vector from robot to goal
        double dx = goalX - robotXInches;
        double dy = goalY - robotYInches;

        // atan2(dy, dx) gives the angle from +X axis to the vector
        // This is the heading we want the robot to face
        targetHeadingRad = Math.atan2(dy, dx);

        // Calculate heading error = target - current
        // Positive error means we need to turn counter-clockwise
        double errorRad = targetHeadingRad - robotHeadingRad;

        // Normalize error to -PI to +PI (shortest path)
        while (errorRad > Math.PI) errorRad -= 2.0 * Math.PI;
        while (errorRad < -Math.PI) errorRad += 2.0 * Math.PI;

        coarseAimErrorDeg = Math.toDegrees(errorRad);
    }

    /**
     * Computes the turn correction power for coarse aiming using odometry.
     * Uses simple P control to match Pedro Pathing behavior (no oscillation).
     *
     * SIGN CONVENTION:
     * - Positive error = need to turn counter-clockwise (left)
     * - Positive motor power = robot turns right (clockwise)
     * - So we NEGATE the error to get the correct turn direction
     */
    private double computeCoarseAimCorrection() {
        if (!coarseAimActive || selectedAlliance == Alliance.NONE) {
            coarseAimCorrectionPower = 0;
            prevTurnErrorDeg = 0;
            return 0;
        }

        double errorDeg = coarseAimErrorDeg;

        // If within deadzone, stop correcting
        if (Math.abs(errorDeg) < TURN_DEADZONE_DEG) {
            coarseAimCorrectionPower = 0;
            prevTurnErrorDeg = errorDeg;
            return 0;
        }

        // Simple P controller - matches Pedro Pathing behavior
        // NEGATE because positive error needs negative motor power (turn left)
        double correction = -TURN_P * errorDeg;

        // D term for damping (optional, helps reduce overshoot)
        if (loopTimeMs > 0 && TURN_D > 0) {
            double errorRate = (errorDeg - prevTurnErrorDeg) / (loopTimeMs / 1000.0);
            correction -= TURN_D * errorRate;
        }

        // Clamp to max power
        correction = Math.max(-TURN_MAX_POWER, Math.min(TURN_MAX_POWER, correction));

        // Apply minimum power to overcome static friction
        if (Math.abs(correction) < TURN_MIN_POWER && Math.abs(errorDeg) > TURN_DEADZONE_DEG) {
            correction = Math.signum(correction) * TURN_MIN_POWER;
        }

        prevTurnErrorDeg = errorDeg;
        coarseAimCorrectionPower = correction;
        return correction;
    }

    // ══════════════════════════════════════════════════════════════════════════
    // ADAPTIVE POSE FUSION (CAMERA ↔ ODOMETRY)
    // Evaluates different fusion methods and adopts the most accurate one.
    // ══════════════════════════════════════════════════════════════════════════

    /**
     * Adaptively fuses camera-based pose estimates with odometry data.
     * Uses the same coordinate transformation as attemptInitialPoseFromCamera.
     */
    private void updateAdaptivePoseFusion() {
        if (!POSE_CORRECTION_ENABLED || !odoEnabled || robot.pinpoint == null) {
            return;
        }

        double currentTime = loopTimer.seconds();

        // Only attempt correction at specified intervals
        if (currentTime - lastPoseCorrectionTime < POSE_CORRECTION_INTERVAL_SEC) {
            return;
        }

        // Need a valid tag detection with pose data
        if (!tagDetected || detectedTagRange < POSE_CORRECTION_MIN_RANGE ||
                detectedTagRange > POSE_CORRECTION_MAX_RANGE) {
            return;
        }

        List<AprilTagDetection> detections = robot.getDetections();
        AprilTagDetection bestDet = null;
        for (AprilTagDetection det : detections) {
            if (det.id == detectedTagId && det.ftcPose != null && det.robotPose != null && det.metadata != null) {
                bestDet = det;
                break;
            }
        }

        if (bestDet == null || bestDet.robotPose == null) {
            return;
        }

        try {
            // Get camera-based field pose
            org.firstinspires.ftc.robotcore.external.navigation.Position robotPos = bestDet.robotPose.getPosition();
            org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles robotYpr = bestDet.robotPose.getOrientation();

            double camX = robotPos.toUnit(DistanceUnit.INCH).x;
            double camY = robotPos.toUnit(DistanceUnit.INCH).y;
            double camHeading = robotYpr.getYaw(AngleUnit.DEGREES);

            // If camera coordinates are off the field, recalculate from ftcPose + tag position
            if (camX < 0 || camX > 144 || camY < 0 || camY > 144) {
                // Get tag's known field position
                double tagFieldX, tagFieldY;
                if (bestDet.id == 11 || bestDet.id == 12 || bestDet.id == 13 || bestDet.id == 14) {
                    tagFieldX = BLUE_TAG_X;
                    tagFieldY = BLUE_TAG_Y;
                } else if (bestDet.id == 15 || bestDet.id == 16) {
                    tagFieldX = RED_TAG_X;
                    tagFieldY = RED_TAG_Y;
                } else {
                    return; // Unknown tag
                }

                // Calculate from ftcPose
                double range = bestDet.ftcPose.range;
                double bearing = Math.toRadians(bestDet.ftcPose.bearing);
                double robotHeadingRad = Math.toRadians(camHeading);

                double angleToTag = robotHeadingRad + bearing;
                camX = tagFieldX - range * Math.cos(angleToTag);
                camY = tagFieldY - range * Math.sin(angleToTag);
            }

            // Validate
            if (camX < -20 || camX > 160 || camY < -20 || camY > 160) {
                return;
            }

            // Get current odometry pose
            Odometry.Position odoPos = odometry.getPosition();
            double odoX = odoPos.xMm / 25.4;
            double odoY = odoPos.yMm / 25.4;
            double odoHeading = Math.toDegrees(odoPos.headingRad);

            // Calculate divergence between camera and odometry
            double posDivergence = Math.sqrt(Math.pow(camX - odoX, 2) + Math.pow(camY - odoY, 2));
            double headingDivergence = Math.abs(camHeading - odoHeading);
            while (headingDivergence > 180) headingDivergence = 360 - headingDivergence;

            cameraOdoDivergence = posDivergence;

            // === ADAPTIVE FUSION ===
            // Adjust fusion weight based on observed divergence
            // High divergence when camera first sees tag = trust camera more
            // Low divergence = can trust odometry more
            if (posDivergence > 12.0 || headingDivergence > 15.0) {
                // Large divergence - increase camera trust
                cameraFusionWeight = Math.min(0.95, cameraFusionWeight + FUSION_WEIGHT_ADJUST_RATE);
            } else if (posDivergence < 3.0 && headingDivergence < 3.0) {
                // Small divergence - odometry is tracking well
                cameraFusionWeight = Math.max(0.3, cameraFusionWeight - FUSION_WEIGHT_ADJUST_RATE);
            }

            // Apply weighted fusion
            double fusedX = camX * cameraFusionWeight + odoX * (1.0 - cameraFusionWeight);
            double fusedY = camY * cameraFusionWeight + odoY * (1.0 - cameraFusionWeight);

            // For heading, use camera weight but blend carefully
            double headingDiff = camHeading - odoHeading;
            while (headingDiff > 180) headingDiff -= 360;
            while (headingDiff < -180) headingDiff += 360;
            double fusedHeading = odoHeading + headingDiff * cameraFusionWeight;

            // Set the fused pose to Pinpoint
            Pose2D newPose = new Pose2D(DistanceUnit.INCH, fusedX, fusedY,
                                        AngleUnit.DEGREES, fusedHeading);
            robot.pinpoint.setPosition(newPose);

            lastPoseCorrectionTime = currentTime;
            poseCorrectionCount++;

            // Store for tracking
            lastCameraPoseX = camX;
            lastCameraPoseY = camY;
            lastCameraPoseHeading = camHeading;
            lastOdoPoseX = odoX;
            lastOdoPoseY = odoY;
            lastOdoPoseHeading = odoHeading;

        } catch (Exception e) {
            // Fusion failed, continue without it
        }
    }

    // ══════════════════════════════════════════════════════════════════════════
    // ADAPTIVE AIMING SYSTEM - Self-improving through hypothesis testing
    // ══════════════════════════════════════════════════════════════════════════

    /**
     * Records an aiming error for the adaptive system to evaluate.
     */
    private void recordAimError(double errorMagnitude) {
        recentAimErrors.add(errorMagnitude);
        if (recentAimErrors.size() > AIM_ERROR_WINDOW_SIZE) {
            recentAimErrors.remove(0);
        }
        hypothesisSampleCount++;
    }

    /**
     * Updates the adaptive aiming system. This system:
     * 1. Generates hypotheses about aim offset corrections
     * 2. Tests each hypothesis by measuring resulting errors
     * 3. Adopts improvements that consistently reduce aiming error
     */
    private void updateAdaptiveAiming() {
        // Only evaluate when we have enough samples
        if (hypothesisSampleCount < HYPOTHESIS_EVAL_SAMPLES) {
            return;
        }

        // Calculate average error with current hypothesis
        double avgError = calculateAverageAimError();

        // If this hypothesis is better than best known, adopt it
        if (avgError < bestAverageError) {
            bestAverageError = avgError;
            bestAimOffset = aimOffsetHypothesis;

            // Continue exploring in same direction
            if (hypothesisDirection == 0) {
                // First improvement - pick a random direction
                hypothesisDirection = (Math.random() > 0.5) ? 1 : -1;
            }
        } else {
            // This hypothesis is worse - reverse direction or reduce step
            hypothesisDirection = -hypothesisDirection;

            // If we've oscillated, we're near optimal - reduce exploration
            if (Math.abs(aimOffsetHypothesis - bestAimOffset) < HYPOTHESIS_STEP_DEG) {
                // Near optimal, reset to best and stop exploring
                aimOffsetHypothesis = bestAimOffset;
                hypothesisDirection = 0;
            }
        }

        // Generate next hypothesis if still exploring
        if (hypothesisDirection != 0) {
            aimOffsetHypothesis = bestAimOffset + hypothesisDirection * HYPOTHESIS_STEP_DEG;
            // Clamp hypothesis to reasonable range
            aimOffsetHypothesis = Math.max(-10.0, Math.min(10.0, aimOffsetHypothesis));
        }

        // Reset for next evaluation period
        hypothesisSampleCount = 0;
        recentAimErrors.clear();
    }

    /**
     * Calculates the average aiming error from recent samples.
     */
    private double calculateAverageAimError() {
        if (recentAimErrors.isEmpty()) {
            return Double.MAX_VALUE;
        }
        double sum = 0;
        for (double error : recentAimErrors) {
            sum += error;
        }
        return sum / recentAimErrors.size();
    }

    // ══════════════════════════════════════════════════════════════
    // DRIVE
    // ══════════════════════════════════════════════════════════════

    private void updateDrive() {
        isSlowMode = gamepad1.right_bumper;

        // LB activates auto-aim (coarse → fine based on state machine)
        // aimLockActive is set by the state machine when in FINE_AIM phase
        // coarseAimActive is set by the state machine when in COARSE_SNAP phase

        double fwd = -gamepad1.left_stick_y;
        double str = gamepad1.left_stick_x;
        double rot;

        if (Math.abs(fwd) < DRIVE_DEADZONE) fwd = 0;
        if (Math.abs(str) < DRIVE_DEADZONE) str = 0;

        if (aimLockActive) {
            // Fine aim: use AprilTag camera for rotation correction
            rot = computeAprilTagAimCorrection();
        } else if (coarseAimActive) {
            // Coarse aim: use odometry-based heading correction
            rot = computeCoarseAimCorrection();
        } else {
            // Manual control
            rot = gamepad1.right_stick_x;
            if (Math.abs(rot) < DRIVE_DEADZONE) rot = 0;
            aimCorrectionPower = 0;
            aimErrorDeg = 0;
            coarseAimCorrectionPower = 0;
        }

        if (isSlowMode) {
            fwd *= SLOW_MODE_MULTIPLIER;
            str *= SLOW_MODE_MULTIPLIER;
            if (!aimLockActive && !coarseAimActive) {
                rot *= SLOW_MODE_MULTIPLIER;
            }
        }

        double fl = fwd + str + rot;
        double fr = fwd - str - rot;
        double bl = fwd - str + rot;
        double br = fwd + str - rot;

        double max = Math.max(Math.abs(fl), Math.max(Math.abs(fr),
                Math.max(Math.abs(bl), Math.abs(br))));
        if (max > 1.0) {
            fl /= max;
            fr /= max;
            bl /= max;
            br /= max;
        }

        if (inBrownout) {
            fl *= BROWNOUT_DRIVE_SCALE;
            fr *= BROWNOUT_DRIVE_SCALE;
            bl *= BROWNOUT_DRIVE_SCALE;
            br *= BROWNOUT_DRIVE_SCALE;
        }

        flPower = fl;
        frPower = fr;
        blPower = bl;
        brPower = br;

        if (robot.frontLeft != null) robot.frontLeft.setPower(fl);
        if (robot.frontRight != null) robot.frontRight.setPower(fr);
        if (robot.backLeft != null) robot.backLeft.setPower(bl);
        if (robot.backRight != null) robot.backRight.setPower(br);
    }

    // ══════════════════════════════════════════════════════════════
    // FLYWHEEL
    // ══════════════════════════════════════════════════════════════

    private void updateFlywheelTarget() {
        if (gamepad2.dpad_up && !prevDpadUp) {
            flywheelTarget = Math.min(flywheelTarget + FLYWHEEL_STEP, FLYWHEEL_MAX);
        }
        prevDpadUp = gamepad2.dpad_up;

        if (gamepad2.dpad_down && !prevDpadDown) {
            flywheelTarget = Math.max(flywheelTarget - FLYWHEEL_STEP, FLYWHEEL_MIN);
        }
        prevDpadDown = gamepad2.dpad_down;
    }

    private void updateFlywheel(double deltaTime) {
        if (gamepad2.x && !prevFlywheelToggle) {
            flywheelOn = !flywheelOn;
            if (!flywheelOn) stopFlywheel();
        }
        prevFlywheelToggle = gamepad2.x;

        updateFlywheelCommon(deltaTime, gamepad1.left_trigger > 0.5);
    }

    // ══════════════════════════════════════════════════════════════
    // INTAKE
    // ══════════════════════════════════════════════════════════════

    private void updateIntake() {
        double desired1;
        if (gamepad2.left_trigger > 0.3) {
            desired1 = INTAKE_POWER;
        } else if (gamepad2.right_trigger > 0.3) {
            desired1 = -INTAKE_POWER;
        } else {
            desired1 = 0;
        }

        double desired2;
        if (gamepad2.a) {
            desired2 = INTAKE_POWER;
        } else if (gamepad2.b) {
            desired2 = -INTAKE_POWER;
        } else {
            desired2 = 0;
        }

        if (inBrownout) {
            desired1 *= BROWNOUT_INTAKE_SCALE;
            desired2 *= BROWNOUT_INTAKE_SCALE;
        }

        intake1Power = desired1;
        intake2Power = desired2;

        if (robot.intake1 != null) robot.intake1.setPower(intake1Power);
        if (robot.intake2 != null) robot.intake2.setPower(intake2Power);
    }

    // ══════════════════════════════════════════════════════════════
    // SERVO — GP1 LT shoots
    // ══════════════════════════════════════════════════════════════

    private void updateServo() {
        if (robot.transferServo != null) {
            boolean shooting = gamepad1.left_trigger > 0.5;
            robot.transferServo.setPosition(shooting ? SERVO_SHOOT : SERVO_IDLE);
        }
    }

    // ══════════════════════════════════════════════════════════════════════════
    // FLYWHEEL VELOCITY CONTROL
    // The deposit mechanism runs at EXACTLY the tick speed the driver commands.
    // The commanded speed NEVER drifts or stabilizes at a different value.
    // Only the recovery speed (P gain) varies to return to target faster.
    // ══════════════════════════════════════════════════════════════════════════

    private void updateFlywheelCommon(double deltaTime, boolean shooting) {
        vel1 = (robot.deposit1 != null) ? Math.abs(robot.deposit1.getVelocity()) : 0;
        vel2 = (robot.deposit2 != null) ? Math.abs(robot.deposit2.getVelocity()) : 0;
        avgVel = (vel1 + vel2) / 2.0;

        if (!flywheelOn) {
            commandedTarget = 0;
            return;
        }

        // The EXACT target is always what the driver set + boost during shooting
        // NO ramping - the target is immediate and exact
        double exactTarget = flywheelTarget + (shooting ? SHOOT_BOOST : 0);
        commandedTarget = exactTarget;

        // Velocity error determines if we need recovery mode
        double velocityError = exactTarget - avgVel;

        // Recovery mode: use higher P gain when velocity drops (e.g., after shooting)
        // This ONLY affects how fast we return to target, NOT the target itself
        if (!inRecoveryMode && velocityError > RECOVERY_THRESHOLD) {
            inRecoveryMode = true;
            updatePIDF(RECOVERY_P);
        } else if (inRecoveryMode && velocityError < RECOVERY_EXIT) {
            inRecoveryMode = false;
            updatePIDF(PIDF_P);
        }

        // Command the EXACT target velocity - no deviation allowed
        setFlywheelVelocity(exactTarget);
    }

    private void setFlywheelVelocity(double velocity) {
        // Set the EXACT commanded velocity - motors will track this precisely
        if (robot.deposit1 != null) {
            robot.deposit1.setVelocity(DEPOSIT1_REVERSED ? -velocity : velocity);
        }
        if (robot.deposit2 != null) {
            robot.deposit2.setVelocity(DEPOSIT2_REVERSED ? -velocity : velocity);
        }
    }

    private void stopFlywheel() {
        setFlywheelVelocity(0);
        inRecoveryMode = false;
        updatePIDF(PIDF_P);
    }

    private void updatePIDF(double pValue) {
        if (Math.abs(pValue - lastP) < 0.01) return;
        try {
            if (robot.deposit1 != null) {
                robot.deposit1.setVelocityPIDFCoefficients(pValue, PIDF_I, PIDF_D, PIDF_F);
            }
            if (robot.deposit2 != null) {
                robot.deposit2.setVelocityPIDFCoefficients(pValue, PIDF_I, PIDF_D, PIDF_F);
            }
            lastP = pValue;
        } catch (Exception ignored) {}
    }

    // ══════════════════════════════════════════════════════════════
    // TELEMETRY
    // ══════════════════════════════════════════════════════════════

    private void updateTelemetry() {
        if (inBrownout) {
            telemetry.addData("Battery", "%.2fV ⚠ BROWNOUT — drive 80%% intakes 33%%", smoothedVoltage);
        } else {
            telemetry.addData("Battery", "%.2fV", smoothedVoltage);
        }

        // Alliance
        String allianceStr;
        switch (selectedAlliance) {
            case RED:   allianceStr = "🔴 RED"; break;
            case BLUE:  allianceStr = "🔵 BLUE"; break;
            default:    allianceStr = "⬜ NONE"; break;
        }
        telemetry.addData("Alliance", allianceStr);

        // Initial pose status
        telemetry.addData("Pose Init", initialPoseEstablished ? "✓" : "⏳");

        telemetry.addLine("");

        // Aiming Status - Camera is primary, odometry is just to get close
        telemetry.addLine("── AIMING ──");
        String phaseStr;
        switch (currentAimPhase) {
            case COARSE_SNAP: phaseStr = "ODO (finding tag...)"; break;
            case FINE_AIM:    phaseStr = "CAMERA ✓ (primary)"; break;
            default:          phaseStr = "IDLE"; break;
        }
        telemetry.addData("Mode", phaseStr);

        if (coarseAimActive && odometry != null) {
            Odometry.Position pos = odometry.getPosition();
            double robotX = pos.xMm / 25.4;
            double robotY = pos.yMm / 25.4;
            double robotH = Math.toDegrees(pos.headingRad);

            double goalX = selectedAlliance == Alliance.BLUE ? BLUE_GOAL_X : RED_GOAL_X;
            double goalY = selectedAlliance == Alliance.BLUE ? BLUE_GOAL_Y : RED_GOAL_Y;

            telemetry.addData("Robot", "X=%.1f Y=%.1f H=%.1f°", robotX, robotY, robotH);
            telemetry.addData("Goal", "X=%.1f Y=%.1f", goalX, goalY);
            telemetry.addData("Turning to", "%.1f° (err: %.1f°)",
                    Math.toDegrees(targetHeadingRad), coarseAimErrorDeg);
            telemetry.addData("Power", "%.3f", coarseAimCorrectionPower);
            telemetry.addLine("(Looking for AprilTag...)");
        }

        if (aimLockActive && tagDetected) {
            String lockStatus = Math.abs(aimErrorDeg) < TURN_DEADZONE_DEG ? "🎯 LOCKED" : "AIMING";
            telemetry.addData("Camera Aim", "%s (err: %.1f°)", lockStatus, aimErrorDeg);
            telemetry.addData("Power", "%.3f", aimCorrectionPower);
        }

        // Adaptive aiming status
        if (bestAverageError < Double.MAX_VALUE) {
            telemetry.addData("Aim Offset", "%.2f° (best: %.2f° err: %.2f°)",
                    aimOffsetHypothesis, bestAimOffset, bestAverageError);
        }

        telemetry.addLine("");

        // AprilTag Detection
        telemetry.addLine("── APRILTAG ──");
        telemetry.addData("Vision", robot.getVisionStatusString());
        telemetry.addData("Tags Visible", totalTagsVisible);

        if (tagDetected) {
            telemetry.addData("Target Tag", "#%d at %.1f\" | bearing %.1f°",
                    detectedTagId, detectedTagRange, detectedTagBearing);

            // Show raw vs calculated camera pose for debugging
            List<AprilTagDetection> dets = robot.getDetections();
            for (AprilTagDetection d : dets) {
                if (d.id == detectedTagId && d.robotPose != null && d.ftcPose != null) {
                    org.firstinspires.ftc.robotcore.external.navigation.Position rp = d.robotPose.getPosition();
                    org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles ypr = d.robotPose.getOrientation();
                    double rawX = rp.toUnit(DistanceUnit.INCH).x;
                    double rawY = rp.toUnit(DistanceUnit.INCH).y;
                    double rawH = ypr.getYaw(AngleUnit.DEGREES);
                    telemetry.addData("SDK Pose (raw)", "X=%.1f Y=%.1f H=%.1f°", rawX, rawY, rawH);

                    // Show our calculated position
                    double tagX = (d.id >= 11 && d.id <= 14) ? BLUE_TAG_X : RED_TAG_X;
                    double tagY = (d.id >= 11 && d.id <= 14) ? BLUE_TAG_Y : RED_TAG_Y;
                    double range = d.ftcPose.range;
                    double bearingRad = Math.toRadians(d.ftcPose.bearing);
                    double headingRad = Math.toRadians(rawH);
                    double angleToTag = headingRad + bearingRad;
                    double calcX = tagX - range * Math.cos(angleToTag);
                    double calcY = tagY - range * Math.sin(angleToTag);
                    telemetry.addData("Calc Pose", "X=%.1f Y=%.1f (tag at %.1f,%.1f)", calcX, calcY, tagX, tagY);
                    break;
                }
            }
        }

        // Pose Fusion status
        telemetry.addLine("");
        telemetry.addLine("── POSE FUSION ──");
        telemetry.addData("Corrections", "%d (weight: %.0f%% cam)",
                poseCorrectionCount, cameraFusionWeight * 100);
        if (cameraOdoDivergence > 0) {
            telemetry.addData("Divergence", "%.1f in", cameraOdoDivergence);
        }

        telemetry.addLine("");

        // Flywheel - exact velocity control
        telemetry.addLine("── FLYWHEEL (Exact Speed) ──");
        if (flywheelOn) {
            double error = commandedTarget - avgVel;
            String status = inRecoveryMode ? "RECOVERING" :
                    (Math.abs(error) < 20) ? "READY ✓" : "STABILIZING";

            telemetry.addData("Status", status);
            telemetry.addData("Target", "%.0f ticks/s [Dpad ±%.0f]", flywheelTarget, FLYWHEEL_STEP);
            telemetry.addData("Actual", "%.0f / %.0f (err: %.0f)", avgVel, commandedTarget, error);
            telemetry.addData("Motors", "%.0f | %.0f", vel1, vel2);
        } else {
            telemetry.addLine("OFF (GP2.X to start)");
            telemetry.addData("Target", "%.0f", flywheelTarget);
        }

        telemetry.addLine("");

        // Drive
        telemetry.addLine("── DRIVE ──");
        String modeStr;
        if (aimLockActive && isSlowMode) {
            modeStr = "FINE AIM + SLOW";
        } else if (coarseAimActive && isSlowMode) {
            modeStr = "COARSE AIM + SLOW";
        } else if (aimLockActive) {
            modeStr = "FINE AIM";
        } else if (coarseAimActive) {
            modeStr = "COARSE AIM";
        } else if (isSlowMode) {
            modeStr = "SLOW (40%)";
        } else {
            modeStr = "NORMAL";
        }
        telemetry.addData("Mode", modeStr);
        telemetry.addData("FL|FR", "%+.2f | %+.2f", flPower, frPower);
        telemetry.addData("BL|BR", "%+.2f | %+.2f", blPower, brPower);

        telemetry.addLine("");

        // Intakes
        String i1 = intake1Power > 0.1 ? "IN" : intake1Power < -0.1 ? "OUT" : "OFF";
        String i2 = intake2Power > 0.1 ? "IN" : intake2Power < -0.1 ? "OUT" : "OFF";
        telemetry.addData("Intakes", "%s | %s", i1, i2);

        // Position (Odometry)
        telemetry.addLine("");
        telemetry.addLine("── ODOMETRY ──");
        if (odoEnabled && odometry != null) {
            Odometry.Position p = odometry.getPosition();
            telemetry.addData("Position", "X: %.1f in, Y: %.1f in",
                    p.xMm / 25.4, p.yMm / 25.4);
            telemetry.addData("Heading", "%.1f°", Math.toDegrees(p.headingRad));
            if (selectedAlliance != Alliance.NONE) {
                double goalX = selectedAlliance == Alliance.BLUE ? BLUE_GOAL_X : RED_GOAL_X;
                double goalY = selectedAlliance == Alliance.BLUE ? BLUE_GOAL_Y : RED_GOAL_Y;
                double dx = goalX - (p.xMm / 25.4);
                double dy = goalY - (p.yMm / 25.4);
                double distToGoal = Math.sqrt(dx * dx + dy * dy);
                telemetry.addData("Dist to Goal", "%.1f in", distToGoal);
            }
        } else {
            telemetry.addLine("Odometry not available");
        }

        telemetry.addLine("");
        double hz = loopTimeMs > 0 ? 1000.0 / loopTimeMs : 0;
        telemetry.addData("Loop", "%.1fms (%.0fHz)", loopTimeMs, hz);

        telemetry.update();
    }
}