package org.firstinspires.ftc.teamcode.auxiliary;

/**
 * Static pose handoff between OpModes within one robot boot. An Autonomous
 * OpMode should save its final pose in its stop() path; TeleOp init() then
 * loads it so auto-aim and relocalization know roughly where the robot is
 * without a manual corner relocalize.
 * <p>
 * Not saved from anywhere yet - there are no Autonomous OpModes in the repo
 * at the moment - but AutoAim/Relocalize already honor {@link #valid} when
 * loading, so wiring a future Auton only needs one save() call.
 */
public final class PoseStorage {
    public static double xInches = 0.0;
    public static double yInches = 0.0;
    public static double headingRadians = 0.0;
    public static boolean valid = false;

    private PoseStorage() {
    }

    public static void save(double xInches, double yInches, double headingRadians) {
        PoseStorage.xInches = xInches;
        PoseStorage.yInches = yInches;
        PoseStorage.headingRadians = headingRadians;
        PoseStorage.valid = true;
    }

    public static void clear() {
        valid = false;
    }
}
