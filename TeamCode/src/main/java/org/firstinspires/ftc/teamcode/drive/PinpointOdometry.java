package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import org.firstinspires.ftc.teamcode.config.RobotConstants;

/**
 * Minimal custom odometry built directly on the stock FTC SDK
 * {@link GoBildaPinpointDriver}. No Pedro Pathing involved.
 *
 * The two odometry pods plug straight into the Pinpoint, which does all of the
 * math (including sensor-fused heading) on board. This class only sends the
 * mounting info it needs and then reads the pose once per loop.
 *
 * Coordinate convention (matches the old Pedro pose data):
 *  - X is forward from where the pose was zeroed, in inches.
 *  - Y is strafe (left positive) from where the pose was zeroed, in inches.
 *  - Heading is CCW-positive, in radians, normalized to +/-180 degrees.
 */
public class PinpointOdometry {
    /** The Pinpoint boots in NOT_READY; wait this long for it to leave that state. */
    private static final long READY_TIMEOUT_MS = 3000;
    /** Allow the on-board IMU zero calibration some time to finish (device samples for ~0.25 s). */
    private static final long IMU_CALIBRATION_WAIT_MS = 300;

    private final GoBildaPinpointDriver pinpoint;
    private final boolean available;

    public PinpointOdometry(HardwareMap hardwareMap) {
        GoBildaPinpointDriver device = null;
        if (hardwareMap != null) {
            try {
                device = hardwareMap.tryGet(GoBildaPinpointDriver.class, RobotConstants.PINPOINT);
            } catch (Exception ignored) {
                device = null;
            }
        }
        this.pinpoint = device;
        this.available = device != null;

        if (device == null) {
            return;
        }

        // Writes to the device are ignored while it powers up, so do not configure until it is ready.
        long deadline = System.currentTimeMillis() + READY_TIMEOUT_MS;
        do {
            update();
            if (getDeviceStatus() != GoBildaPinpointDriver.DeviceStatus.NOT_READY) {
                break;
            }
            sleep(10);
        } while (System.currentTimeMillis() < deadline);

        device.setOffsets(
                RobotConstants.PINPOINT_X_POD_OFFSET_MM,
                RobotConstants.PINPOINT_Y_POD_OFFSET_MM,
                DistanceUnit.MM);
        device.setEncoderResolution(RobotConstants.PINPOINT_POD_TYPE);
        device.setEncoderDirections(
                RobotConstants.PINPOINT_X_POD_DIRECTION,
                RobotConstants.PINPOINT_Y_POD_DIRECTION);

        // Zero the pose and recalibrate the built-in IMU. Robot must be stationary here.
        device.resetPosAndIMU();
        sleep(IMU_CALIBRATION_WAIT_MS);
        update();
    }

    /** Call once per loop to pull the newest pose out of the Pinpoint. */
    public void update() {
        if (pinpoint == null) {
            return;
        }
        pinpoint.update();
    }

    public boolean isAvailable() {
        return available;
    }

    /** X position in inches, measured from the pose that was zeroed most recently. */
    public double getX() {
        return pinpoint == null ? 0.0 : pinpoint.getPosX(DistanceUnit.INCH);
    }

    /** Y position in inches (left positive), measured from the pose that was zeroed most recently. */
    public double getY() {
        return pinpoint == null ? 0.0 : pinpoint.getPosY(DistanceUnit.INCH);
    }

    /** Heading in radians, CCW positive, normalized to +/-180 degrees. */
    public double getHeadingRadians() {
        return pinpoint == null ? 0.0 : pinpoint.getHeading(AngleUnit.RADIANS);
    }

    public GoBildaPinpointDriver.DeviceStatus getDeviceStatus() {
        return pinpoint == null ? GoBildaPinpointDriver.DeviceStatus.NOT_READY : pinpoint.getDeviceStatus();
    }

    /** Approximate how fast the Pinpoint's own odometry loop is running, in Hz. */
    public double getFrequency() {
        return pinpoint == null ? 0.0 : pinpoint.getFrequency();
    }

    /** Raw tick counts from each pod; handy when checking pod wiring or direction. */
    public int getEncoderX() {
        return pinpoint == null ? 0 : pinpoint.getEncoderX();
    }

    public int getEncoderY() {
        return pinpoint == null ? 0 : pinpoint.getEncoderY();
    }

    /** Zero the pose and recalibrate the IMU. Robot must be stationary. */
    public void resetPose() {
        if (pinpoint == null) {
            return;
        }
        pinpoint.resetPosAndIMU();
        sleep(IMU_CALIBRATION_WAIT_MS);
        update();
    }

    /** Overwrite what the Pinpoint is tracking from, e.g. to relocalize against a known spot. */
    public void setPose(double x, double y, double headingRadians) {
        if (pinpoint == null) {
            return;
        }
        pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, x, y, AngleUnit.RADIANS, headingRadians));
    }

    public String getStatusText() {
        if (pinpoint == null) {
            return "not connected";
        }
        GoBildaPinpointDriver.DeviceStatus status = pinpoint.getDeviceStatus();
        return status.name() + String.format(" (%.0f Hz, enc: %d/%d)",
                pinpoint.getFrequency(), pinpoint.getEncoderX(), pinpoint.getEncoderY());
    }

    private static void sleep(long milliseconds) {
        try {
            Thread.sleep(milliseconds);
        } catch (InterruptedException ignored) {
            Thread.currentThread().interrupt();
        }
    }
}
