package org.firstinspires.ftc.teamcode.auxiliary;

import org.firstinspires.ftc.teamcode.config.RobotConstants;
import org.firstinspires.ftc.teamcode.drive.Alliance;

/**
 * Chassis auto-aim toward the nearest goal aim point for the selected
 * alliance, using the Pinpoint pose. Purely math: it computes the turn power;
 * the caller (TeleOp/Drivetrain) decides when it may replace the driver's
 * turn stick.
 * <p>
 * All tunables are read from RobotConstants every update/compute so FTC
 * Dashboard edits take effect live.
 * <p>
 * Sign convention: AIM_TURN_SIGN is +1 because the Drivetrain's mecanum mix
 * gives the left wheels +turn and the right wheels -turn, so a POSITIVE turn
 * input spins the chassis CCW. The Pinpoint heading is CCW-positive, so a
 * positive heading error ("target is CCW of me") needs a positive turn. If
 * the robot ends up turning away from the goal, set AIM_TURN_SIGN to -1 in
 * FTC Dashboard.
 */
public class AutoAimController {
    private final PinpointLocalizer localizer;

    private Alliance alliance = null;

    // Target selection state (recomputed by update()).
    private boolean targetKnown = false;
    private int targetIndex = 0;
    private double targetX = 0.0;
    private double targetY = 0.0;
    private double distanceInches = 0.0;

    // Heading state (recomputed by computeTurn()).
    private double headingError = 0.0;

    public AutoAimController(PinpointLocalizer localizer) {
        this.localizer = localizer;
        validateArrays();
    }

    /** Selecting a null alliance disables aiming entirely. */
    public void setAlliance(Alliance alliance) {
        this.alliance = alliance;
    }

    public Alliance getAlliance() {
        return alliance;
    }

    /**
     * Recomputes the target goal point from the current pose. Call once per
     * loop; keeps the current target index with AIM_SWITCH_HYSTERESIS_IN of
     * hysteresis so a standing robot equidistant from both goals does not
     * flip-flop.
     */
    public void update() {
        if (alliance == null) {
            targetKnown = false;
            distanceInches = 0.0;
            headingError = 0.0;
            return;
        }

        validateArrays(); // cheap: catches a bad dashboard edit mid-match
        double[] aimXs = alliance == Alliance.RED ? RobotConstants.RED_AIM_XS : RobotConstants.BLUE_AIM_XS;
        double[] aimYs = alliance == Alliance.RED ? RobotConstants.RED_AIM_YS : RobotConstants.BLUE_AIM_YS;

        double robotX = localizer.getXInches();
        double robotY = localizer.getYInches();
        int nearest = FieldGeometry.nearestIndex(aimXs, aimYs, robotX, robotY);

        if (!targetKnown) {
            targetIndex = nearest;
            targetKnown = true;
        } else {
            // Only switch away from the current target when the other point is
            // closer by more than the hysteresis band.
            double currentDistance = FieldGeometry.distance(robotX, robotY, aimXs[targetIndex], aimYs[targetIndex]);
            double remainingDistance = FieldGeometry.distance(robotX, robotY, aimXs[nearest], aimYs[nearest]);
            if (nearest != targetIndex && remainingDistance + RobotConstants.AIM_SWITCH_HYSTERESIS_IN < currentDistance) {
                targetIndex = nearest;
            }
        }

        targetX = aimXs[targetIndex];
        targetY = aimYs[targetIndex];
        distanceInches = FieldGeometry.distance(robotX, robotY, targetX, targetY);
    }

    /**
     * Turn power toward the target, added like the driver's turn input in the
     * mecanum mix (+ turns CCW). Returns 0 when aiming is disabled or the pose
     * is not valid.
     */
    public double computeTurn() {
        if (alliance == null) {
            return 0.0;
        }

        if (RobotConstants.AIM_REQUIRES_VALID_POSE && !localizer.isPoseValid()) {
            headingError = 0.0;
            return 0.0;
        }

        double robotX = localizer.getXInches();
        double robotY = localizer.getYInches();
        double robotHeading = localizer.getHeadingRadians();

        // NaN safety: skip the first loop (constants not pulled from config yet)
        // and parking exactly on the target point.
        if (!targetKnown
                || !Double.isFinite(targetX) || !Double.isFinite(targetY)
                || !Double.isFinite(robotHeading)) {
            headingError = 0.0;
            return 0.0;
        }
        if (targetX == robotX && targetY == robotY) {
            headingError = 0.0;
            return 0.0;
        }

        double desiredHeading = FieldGeometry.headingTo(robotX, robotY, targetX, targetY)
                + Math.toRadians(RobotConstants.AIM_HEADING_OFFSET_DEG);
        headingError = FieldGeometry.angleWrap(desiredHeading - robotHeading);

        if (Math.abs(headingError) < RobotConstants.AIM_DEADBAND_RADIANS) {
            return 0.0;
        }

        double turn = Math.max(-RobotConstants.AIM_MAX_TURN,
                Math.min(RobotConstants.AIM_MAX_TURN, RobotConstants.AIM_TURN_GAIN * headingError));
        return turn * RobotConstants.AIM_TURN_SIGN;
    }

    private static void validateArrays() {
        FieldGeometry.requireSameLength("RED_AIM_XS/YS", RobotConstants.RED_AIM_XS, RobotConstants.RED_AIM_YS);
        FieldGeometry.requireSameLength("BLUE_AIM_XS/YS", RobotConstants.BLUE_AIM_XS, RobotConstants.BLUE_AIM_YS);
    }

    public int getTargetIndex() {
        return targetIndex;
    }

    public double getTargetXInches() {
        return targetX;
    }

    public double getTargetYInches() {
        return targetY;
    }

    public double getDistanceInches() {
        return distanceInches;
    }

    public double getHeadingErrorDegrees() {
        return Math.toDegrees(headingError);
    }

    /** True when the controller has a target and the robot is inside the deadband. */
    public boolean isAimed() {
        return alliance != null && targetKnown && Math.abs(headingError) < RobotConstants.AIM_DEADBAND_RADIANS;
    }
}