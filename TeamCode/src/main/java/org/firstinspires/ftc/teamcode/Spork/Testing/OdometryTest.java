package org.firstinspires.ftc.teamcode.Testing;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.Locale;

/**
 * Odometry Test - Comprehensive diagnostic for Pinpoint odometry system
 * This OpMode tests:
 * - Pinpoint device initialization
 * - Encoder connectivity and direction
 * - Heading calculation (degrees and radians)
 * - Position tracking accuracy
 * - Update frequency
 * Controls:
 * - A: Reset odometry position to (0, 0, 0°)
 * - B: Set position to field start (120.316, 128.707, 38°)
 * - X: Toggle raw encoder display
 * - Y: Toggle device status display
 * - DPAD UP: Increase heading offset by 1°
 * - DPAD DOWN: Decrease heading offset by 1°
 * - DPAD LEFT: Decrease heading offset by 10°
 * - DPAD RIGHT: Increase heading offset by 10°
 */
@TeleOp(name = "Odometry Test", group = "Testing")
@Configurable
public class OdometryTest extends OpMode {

    private TelemetryManager panelsTelemetry;
    private Follower follower;
    private GoBildaPinpointDriver pinpoint;

    private ElapsedTime runtime;
    private ElapsedTime updateTimer;

    // Test state
    private boolean showRawEncoders = false;
    private boolean showDeviceStatus = true;
    private double headingOffsetDeg = 0.0;

    // Button state tracking
    private boolean lastA = false;
    private boolean lastB = false;
    private boolean lastX = false;
    private boolean lastY = false;
    private boolean lastDpadUp = false;
    private boolean lastDpadDown = false;
    private boolean lastDpadLeft = false;
    private boolean lastDpadRight = false;

    // Tracking data
    private double prevX = 0;
    private double prevY = 0;
    private double prevHeading = 0;
    private int updateCount = 0;
    private double updateRate = 0;

    // Error tracking
    private boolean initializationError = false;
    private String errorMessage = "";

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        runtime = new ElapsedTime();
        updateTimer = new ElapsedTime();

        telemetry.addLine("Initializing Odometry Test...");
        telemetry.update();

        try {
            // Initialize Pedro Pathing follower (which uses Pinpoint)
            follower = Constants.createFollower(hardwareMap);
            follower.setStartingPose(new Pose(0, 0, 0));

            // Try to get direct access to Pinpoint device
            try {
                pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "odo");
                pinpoint.setOffsets(
                    Constants.localizerConstants.strafePodX,
                    Constants.localizerConstants.forwardPodY,
                    DistanceUnit.INCH
                );
                pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
                pinpoint.setEncoderDirections(
                    Constants.localizerConstants.strafeEncoderDirection,
                    Constants.localizerConstants.forwardEncoderDirection
                );
                pinpoint.resetPosAndIMU();
                telemetry.addLine("✓ Pinpoint device initialized");
            } catch (Exception e) {
                telemetry.addLine("⚠ Warning: Could not access Pinpoint directly");
                telemetry.addLine("  Reason: " + e.getMessage());
                pinpoint = null;
            }

            telemetry.addLine("✓ Pedro Pathing follower initialized");
            telemetry.addLine("");
            telemetry.addLine("Ready! Press START to begin testing.");

        } catch (Exception e) {
            initializationError = true;
            errorMessage = e.getMessage();
            telemetry.addLine("❌ INITIALIZATION FAILED!");
            telemetry.addLine("Error: " + errorMessage);
            telemetry.addLine("");
            telemetry.addLine("Check:");
            telemetry.addLine("1. Is Pinpoint device named 'odo' in config?");
            telemetry.addLine("2. Is Pinpoint properly connected?");
            telemetry.addLine("3. Are encoder cables connected?");
        }

        telemetry.update();
    }

    @Override
    public void start() {
        runtime.reset();
        updateTimer.reset();
    }

    @Override
    public void loop() {
        if (initializationError) {
            displayError();
            return;
        }

        // Update follower (this updates odometry)
        follower.update();

        // Track update rate
        updateCount++;
        if (updateTimer.seconds() >= 1.0) {
            updateRate = updateCount / updateTimer.seconds();
            updateCount = 0;
            updateTimer.reset();
        }

        // Handle button inputs
        handleInputs();

        // Display telemetry
        displayTelemetry();
    }

    private void handleInputs() {
        // A - Reset position
        if (gamepad1.a && !lastA) {
            follower.setStartingPose(new Pose(0, 0, 0));
            if (pinpoint != null) {
                pinpoint.resetPosAndIMU();
            }
            telemetry.speak("Position reset");
        }
        lastA = gamepad1.a;

        // B - Set to field start position
        if (gamepad1.b && !lastB) {
            double startHeadingRad = Math.toRadians(38);
            follower.setStartingPose(new Pose(120.316, 128.707, startHeadingRad));
            if (pinpoint != null) {
                pinpoint.setPosition(new Pose2D(
                    DistanceUnit.INCH,
                    120.316,
                    128.707,
                    AngleUnit.RADIANS,
                    startHeadingRad
                ));
            }
            telemetry.speak("Start position set");
        }
        lastB = gamepad1.b;

        // X - Toggle raw encoder display
        if (gamepad1.x && !lastX) {
            showRawEncoders = !showRawEncoders;
        }
        lastX = gamepad1.x;

        // Y - Toggle device status display
        if (gamepad1.y && !lastY) {
            showDeviceStatus = !showDeviceStatus;
        }
        lastY = gamepad1.y;

        // DPAD - Adjust heading offset
        if (gamepad1.dpad_up && !lastDpadUp) {
            headingOffsetDeg += 1.0;
        }
        lastDpadUp = gamepad1.dpad_up;

        if (gamepad1.dpad_down && !lastDpadDown) {
            headingOffsetDeg -= 1.0;
        }
        lastDpadDown = gamepad1.dpad_down;

        if (gamepad1.dpad_right && !lastDpadRight) {
            headingOffsetDeg += 10.0;
        }
        lastDpadRight = gamepad1.dpad_right;

        if (gamepad1.dpad_left && !lastDpadLeft) {
            headingOffsetDeg -= 10.0;
        }
        lastDpadLeft = gamepad1.dpad_left;
    }

    private void displayTelemetry() {
        // Get current pose
        Pose currentPose = follower.getPose();
        double x = currentPose.getX();
        double y = currentPose.getY();
        double headingRad = currentPose.getHeading();
        double headingDeg = Math.toDegrees(headingRad);

        // Normalize heading to 0-360
        while (headingDeg < 0) headingDeg += 360;
        while (headingDeg >= 360) headingDeg -= 360;

        // Apply offset if testing
        double adjustedHeadingDeg = headingDeg + headingOffsetDeg;
        while (adjustedHeadingDeg < 0) adjustedHeadingDeg += 360;
        while (adjustedHeadingDeg >= 360) adjustedHeadingDeg -= 360;

        // Calculate deltas
        double deltaX = x - prevX;
        double deltaY = y - prevY;
        double deltaHeading = headingDeg - prevHeading;

        // Store for next iteration
        prevX = x;
        prevY = y;
        prevHeading = headingDeg;

        // === MAIN POSITION DATA ===
        panelsTelemetry.debug("=== POSITION ===", "");
        panelsTelemetry.debug("X (inches)", String.format(Locale.US, "%.2f", x));
        panelsTelemetry.debug("Y (inches)", String.format(Locale.US, "%.2f", y));
        panelsTelemetry.debug("Heading (deg)", String.format(Locale.US, "%.1f°", headingDeg));
        panelsTelemetry.debug("Heading (rad)", String.format(Locale.US, "%.3f", headingRad));

        if (Math.abs(headingOffsetDeg) > 0.1) {
            panelsTelemetry.debug("Adjusted Hdg", String.format(Locale.US, "%.1f° (offset: %.1f°)", adjustedHeadingDeg, headingOffsetDeg));
        }

        // === VELOCITY/DELTA DATA ===
        panelsTelemetry.debug("=== MOVEMENT ===", "");
        panelsTelemetry.debug("ΔX", String.format(Locale.US, "%.3f", deltaX));
        panelsTelemetry.debug("ΔY", String.format(Locale.US, "%.3f", deltaY));
        panelsTelemetry.debug("ΔHeading", String.format(Locale.US, "%.2f°", deltaHeading));

        // === SYSTEM STATUS ===
        panelsTelemetry.debug("=== STATUS ===", "");
        panelsTelemetry.debug("Update Rate", String.format(Locale.US, "%.1f Hz", updateRate));
        panelsTelemetry.debug("Runtime", String.format(Locale.US, "%.1f sec", runtime.seconds()));
        panelsTelemetry.debug("Follower Busy", follower.isBusy());

        // === PINPOINT DEVICE STATUS ===
        if (showDeviceStatus && pinpoint != null) {
            panelsTelemetry.debug("=== PINPOINT ===", "");

            try {
                pinpoint.update();
                Pose2D pose2D = pinpoint.getPosition();
                GoBildaPinpointDriver.DeviceStatus status = pinpoint.getDeviceStatus();

                panelsTelemetry.debug("Device Status", status.toString());
                panelsTelemetry.debug("Pinpoint X", String.format(Locale.US, "%.2f", pose2D.getX(DistanceUnit.INCH)));
                panelsTelemetry.debug("Pinpoint Y", String.format(Locale.US, "%.2f", pose2D.getY(DistanceUnit.INCH)));
                panelsTelemetry.debug("Pinpoint Hdg", String.format(Locale.US, "%.1f°", pose2D.getHeading(AngleUnit.DEGREES)));

                // Check for discrepancies
                double xDiff = Math.abs(x - pose2D.getX(DistanceUnit.INCH));
                double yDiff = Math.abs(y - pose2D.getY(DistanceUnit.INCH));
                double hDiff = Math.abs(headingDeg - pose2D.getHeading(AngleUnit.DEGREES));

                if (xDiff > 0.5 || yDiff > 0.5 || hDiff > 2.0) {
                    panelsTelemetry.debug("⚠ WARNING", "Follower/Pinpoint mismatch!");
                }

            } catch (Exception e) {
                panelsTelemetry.debug("Pinpoint Error", e.getMessage());
            }
        }

        // === RAW ENCODER DATA ===
        if (showRawEncoders && pinpoint != null) {
            panelsTelemetry.debug("=== ENCODERS ===", "");

            try {
                pinpoint.update();
                int encX = pinpoint.getEncoderX();
                int encY = pinpoint.getEncoderY();

                panelsTelemetry.debug("Forward Enc", String.format(Locale.US, "%d ticks", encY));
                panelsTelemetry.debug("Strafe Enc", String.format(Locale.US, "%d ticks", encX));

                // Check if encoders are moving
                if (encX == 0 && encY == 0) {
                    panelsTelemetry.debug("⚠ WARNING", "Encoders show zero!");
                }

            } catch (Exception e) {
                panelsTelemetry.debug("Encoder Error", e.getMessage());
            }
        }

        // === DIAGNOSTIC INFO ===
        panelsTelemetry.debug("=== DIAGNOSTICS ===", "");

        // Check for stuck heading (0-1 range issue)
        if (Math.abs(headingRad) < 0.02 && runtime.seconds() > 2.0) {
            panelsTelemetry.debug("⚠ Heading Check", "Near zero - move robot to test");
        } else if (headingRad > 0.0 && headingRad < 1.0 && headingDeg < 10.0) {
            panelsTelemetry.debug("⚠ POSSIBLE ISSUE", "Heading may be normalized 0-1!");
        } else if (headingRad >= 0.0 && headingRad <= 6.3) {
            panelsTelemetry.debug("✓ Heading Range", "Normal (radians)");
        }

        // Movement detection
        if (Math.abs(deltaX) > 0.1 || Math.abs(deltaY) > 0.1) {
            panelsTelemetry.debug("✓ Movement", "Detected");
        } else if (runtime.seconds() > 5.0) {
            panelsTelemetry.debug("ℹ Movement", "None - push robot to test");
        }

        // === CONTROLS ===
        panelsTelemetry.debug("=== CONTROLS ===", "");
        panelsTelemetry.debug("A", "Reset to (0,0,0)");
        panelsTelemetry.debug("B", "Set field start");
        panelsTelemetry.debug("X", showRawEncoders ? "Hide encoders" : "Show encoders");
        panelsTelemetry.debug("Y", showDeviceStatus ? "Hide device" : "Show device");
        panelsTelemetry.debug("DPAD", "Adjust heading offset");

        panelsTelemetry.update(telemetry);
    }

    private void displayError() {
        telemetry.addLine("❌ ODOMETRY TEST FAILED");
        telemetry.addLine("");
        telemetry.addLine("Error: " + errorMessage);
        telemetry.addLine("");
        telemetry.addLine("Troubleshooting:");
        telemetry.addLine("1. Check robot configuration");
        telemetry.addLine("2. Verify Pinpoint named 'odo'");
        telemetry.addLine("3. Check encoder connections");
        telemetry.addLine("4. Restart robot controller");
        telemetry.addLine("");
        telemetry.addLine("Configuration should have:");
        telemetry.addLine("- GoBildaPinpointDriver: 'odo'");
        telemetry.addLine("- Forward encoder (Y-axis)");
        telemetry.addLine("- Strafe encoder (X-axis)");
        telemetry.update();
    }

    @Override
    public void stop() {
        if (follower != null) {
            // Stop any movement
            telemetry.addLine("Odometry test stopped");
            telemetry.update();
        }
    }
}

