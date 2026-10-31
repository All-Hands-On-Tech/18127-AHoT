package org.firstinspires.ftc.teamcode.auxiliary;

/**
 * Pure geometry helpers shared by the auto-aim and relocalization code.
 * Field frame: X forward, Y left, inches; headings CCW-positive like the
 * Pinpoint. No FTC imports so this class stays trivially testable.
 */
public final class FieldGeometry {
    private FieldGeometry() {
    }

    /**
     * Index of the point in (xs[i], ys[i]) closest to (x, y).
     * Throws if the arrays are missing, empty, or different lengths.
     */
    public static int nearestIndex(double[] xs, double[] ys, double x, double y) {
        if (xs == null || ys == null) {
            throw new IllegalArgumentException("nearestIndex: arrays must not be null");
        }
        if (xs.length == 0 || ys.length == 0) {
            throw new IllegalArgumentException("nearestIndex: arrays must not be empty");
        }
        if (xs.length != ys.length) {
            throw new IllegalArgumentException(
                    "nearestIndex: arrays different lengths (" + xs.length + " vs " + ys.length + ")");
        }

        int bestIndex = 0;
        double bestDistanceSquared = Double.POSITIVE_INFINITY;
        for (int i = 0; i < xs.length; i++) {
            double dx = xs[i] - x;
            double dy = ys[i] - y;
            double distanceSquared = dx * dx + dy * dy;
            if (distanceSquared < bestDistanceSquared) {
                bestDistanceSquared = distanceSquared;
                bestIndex = i;
            }
        }
        return bestIndex;
    }

    /** Straight-line distance between two field points, in whatever unit was passed in. */
    public static double distance(double x1, double y1, double x2, double y2) {
        double dx = x2 - x1;
        double dy = y2 - y1;
        return Math.hypot(dx, dy);
    }

    /**
     * Field-frame heading from (fromX, fromY) toward (toX, toY): atan2(dy, dx),
     * CCW from the +X axis. atan2 returns NaN only for NaN inputs, which the
     * callers separately guard against.
     */
    public static double headingTo(double fromX, double fromY, double toX, double toY) {
        return Math.atan2(toY - fromY, toX - fromX);
    }

    /**
     * Wraps an angle into -pi..pi. Non-finite input (0/NaN heading from a
     * dead sensor, for instance) maps to 0 rather than looping forever or
     * poisoning downstream math.
     */
    public static double angleWrap(double radians) {
        if (!Double.isFinite(radians)) {
            return 0.0;
        }
        while (radians > Math.PI) {
            radians -= 2.0 * Math.PI;
        }
        while (radians < -Math.PI) {
            radians += 2.0 * Math.PI;
        }
        return radians;
    }

    /**
     * Defense against FTC Dashboard edits that make a pair of arrays
     * inconsistent (XS and YS must describe the same points). Called at init
     * and every update so a bad dashboard edit fails loudly instead of making
     * the aim silently chase the wrong point.
     */
    public static void requireSameLength(String label, double[] xs, double[] ys) {
        if (xs == null || ys == null) {
            throw new IllegalStateException(label + ": arrays must not be null");
        }
        if (xs.length == 0 || ys.length == 0) {
            throw new IllegalStateException(label + ": arrays must not be empty");
        }
        if (xs.length != ys.length) {
            throw new IllegalStateException(
                    label + ": length mismatch (" + xs.length + " vs " + ys.length + ") - fix RobotConstants");
        }
    }
}
