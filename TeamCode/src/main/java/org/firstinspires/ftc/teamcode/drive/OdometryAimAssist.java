package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.config.RobotConstants;

/**
 * Aims by rotating the entire chassis toward a target point using Pinpoint odometry.
 * This robot does not use a separate turret; the chassis itself turns to face the target.
 */
public class OdometryAimAssist {
    private final PinpointOdometry odometry;
    private final boolean enabled;

    private double headingError;
    private double targetX = 8.356902356902351;
    private double targetY = 9.111672278338956;
    private boolean pointTargetEnabled = true;

    public OdometryAimAssist(HardwareMap hardwareMap) {
        odometry = new PinpointOdometry(hardwareMap);
        enabled = odometry.isAvailable();
    }

    public void setTargetPoint(double x, double y) {
        this.targetX = x;
        this.targetY = y;
        pointTargetEnabled = true;
    }

    public void setPointTargetEnabled(boolean enabled) {
        this.pointTargetEnabled = enabled;
    }

    public void updatePose() {
        if (!enabled) {
            return;
        }

        odometry.update();
    }

    public double getTurnCorrection(Alliance alliance) {
        if (!enabled) {
            return 0.0;
        }

        double poseX = odometry.getX();
        double poseY = odometry.getY();
        double poseHeading = odometry.getHeadingRadians();

        double targetXToUse = targetX;
        double targetYToUse = targetY;

        if (pointTargetEnabled) {
            double targetHeading = Math.atan2(targetYToUse - poseY, targetXToUse - poseX);
            headingError = normalizeAngle(targetHeading - poseHeading);
        } else {
            double[] targetXs = alliance == Alliance.RED ? RobotConstants.RED_AIM_XS : RobotConstants.BLUE_AIM_XS;
            double[] targetYs = alliance == Alliance.RED ? RobotConstants.RED_AIM_YS : RobotConstants.BLUE_AIM_YS;

            int closestIndex = 0;
            double closestDistanceSquared = Double.POSITIVE_INFINITY;
            for (int i = 0; i < targetXs.length; i++) {
                double dx = targetXs[i] - poseX;
                double dy = targetYs[i] - poseY;
                double distanceSquared = dx * dx + dy * dy;
                if (distanceSquared < closestDistanceSquared) {
                    closestDistanceSquared = distanceSquared;
                    closestIndex = i;
                }
            }

            targetXToUse = targetXs[closestIndex];
            targetYToUse = targetYs[closestIndex];
            double targetHeading = Math.atan2(targetYToUse - poseY, targetXToUse - poseX);
            headingError = normalizeAngle(targetHeading - poseHeading);
        }

        if (Math.abs(headingError) < RobotConstants.AIM_DEADBAND_RADIANS) {
            return 0.0;
        }
        return Range.clip(
                headingError * RobotConstants.AIM_TURN_GAIN,
                -RobotConstants.AIM_MAX_TURN,
                RobotConstants.AIM_MAX_TURN);
    }

    public void setPose(double x, double y, double headingRadians) {
        odometry.setPose(x, y, headingRadians);
    }

    /** Zero the pose and recalibrate the Pinpoint IMU. Robot must be stationary. */
    public void resetPose() {
        odometry.resetPose();
    }

    public boolean isEnabled() {
        return enabled;
    }

    public double getX() {
        return odometry.getX();
    }

    public double getY() {
        return odometry.getY();
    }

    /** Heading in degrees, CCW positive, normalized to +/-180. */
    public double getHeadingDegrees() {
        return Math.toDegrees(odometry.getHeadingRadians());
    }

    public double getHeadingError() {
        return headingError;
    }

    public double getHeadingErrorDegrees() {
        return Math.toDegrees(getHeadingError());
    }

    public String getOdometryStatusText() {
        return odometry.getStatusText();
    }

    /** Text summary of the pose for telemetry, e.g. "(12.3 in, -4.5 in, 91.2 deg)". */
    public String getPoseText() {
        return String.format("(%.1f in, %.1f in, %.1f deg)", odometry.getX(), odometry.getY(), getHeadingDegrees());
    }

    private static double normalizeAngle(double angle) {
        while (angle > Math.PI) {
            angle -= 2.0 * Math.PI;
        }
        while (angle < -Math.PI) {
            angle += 2.0 * Math.PI;
        }
        return angle;
    }
}
