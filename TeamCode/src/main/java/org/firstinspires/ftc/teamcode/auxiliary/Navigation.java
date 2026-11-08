package org.firstinspires.ftc.teamcode.auxiliary;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import org.firstinspires.ftc.teamcode.config.RobotConstants;

public final class Navigation {
    private Navigation() {
    }

    // =========================================================================
    // Alliance
    // =========================================================================
    public enum Alliance {
        RED("Red"),
        BLUE("Blue");

        private final String displayName;

        Alliance(String displayName) {
            this.displayName = displayName;
        }

        public String getDisplayName() {
            return displayName;
        }
    }

    // =========================================================================
    // AllianceSelector: init-time selection that persists across OpModes
    // =========================================================================
    /**
     * Alliance selection during init: X button = BLUE, B button = RED, pressed
     * on either driver's gamepad. The selection lives in a static field so it
     * persists from an Autonomous OpMode's selection into the TeleOp (and
     * across OpMode restarts) within the same robot boot.
     */
    public static final class AllianceSelector {

        private static Alliance selected = null;

        private AllianceSelector() {
        }

        /**
         * Poll for X/B presses. Call every init_loop iteration (callers running
         * a LinearOpMode should loop this until isStarted()).
         */
        public static void update(Gamepad gamepad1, Gamepad gamepad2) {
            if ((gamepad1 != null && gamepad1.x) || (gamepad2 != null && gamepad2.x)) {
                selected = Alliance.BLUE;
            }
            if ((gamepad1 != null && gamepad1.b) || (gamepad2 != null && gamepad2.b)) {
                selected = Alliance.RED;
            }
        }

        /** The currently selected alliance, or null if none was ever selected. */
        public static Alliance get() {
            return selected;
        }

        /** Shows the selection (or a loud warning) plus the selection instructions. */
        public static void addTelemetry(Telemetry telemetry) {
            if (selected == null) {
                telemetry.addData("Alliance", "NO ALLIANCE SELECTED - aim and relocalize are disabled");
            } else {
                telemetry.addData("Alliance", selected.getDisplayName());
            }
            telemetry.addData("Alliance select", "X = blue, B = red (either gamepad)");
        }
    }

    // =========================================================================
    // PoseStorage
    // =========================================================================
    /**
     * Static pose handoff between OpModes within one robot boot. An Autonomous
     * OpMode should save its final pose in its stop() path; TeleOp init() then
     * loads it so auto-aim and relocalization know roughly where the robot is
     * without a manual corner relocalize.
     */
    public static final class PoseStorage {
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

    // =========================================================================
    // Pinpoint: the pose source (device wrapper + pose validity)
    // =========================================================================
    /**
     * Minimal custom odometry built directly on the stock FTC SDK
     * {@link GoBildaPinpointDriver}. The two odometry pods plug straight into
     * the Pinpoint, which does all of the math (including sensor-fused
     * heading) on board; this class only sends the mounting info it needs and
     * then reads the pose once per loop. Adds pose-validity tracking, which
     * the auto-aim and relocalization code requires.
     *
     * update() must be called exactly once per OpMode loop, before anything
     * else reads the pose.
     */
    public static final class Pinpoint {

        /** The Pinpoint boots in NOT_READY; wait this long for it to leave that state. */
        private static final long READY_TIMEOUT_MS = 3000;
        /** Allow the on-board IMU zero calibration some time to finish (device samples for ~0.25 s). */
        private static final long IMU_CALIBRATION_WAIT_MS = 300;

        private final GoBildaPinpointDriver pinpoint;
        private final boolean available;

        /** True once the pose has been relocalized or loaded from PoseStorage. */
        private boolean poseValid = false;

        public Pinpoint(HardwareMap hardwareMap) {
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
                pinpoint.update();
                if (device.getDeviceStatus() != GoBildaPinpointDriver.DeviceStatus.NOT_READY) {
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
            if (pinpoint != null) {
                pinpoint.update();
            }
        }

        public boolean isAvailable() {
            return available;
        }

        /** X position in inches, measured from the pose that was zeroed most recently. */
        public double getXInches() {
            return pinpoint == null ? 0.0 : pinpoint.getPosX(DistanceUnit.INCH);
        }

        /** Y position in inches (left positive), measured from the pose that was zeroed most recently. */
        public double getYInches() {
            return pinpoint == null ? 0.0 : pinpoint.getPosY(DistanceUnit.INCH);
        }

        /**
         * Heading in radians, CCW positive, wrapped to -pi..pi. The Pinpoint
         * reports within +/-180, but the wrap here guarantees the contract.
         */
        public double getHeadingRadians() {
            return pinpoint == null ? 0.0 : angleWrap(pinpoint.getHeading(AngleUnit.RADIANS));
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

        /** Overwrite what the Pinpoint is tracking from, e.g. from a corner relocalize. */
        public void setPose(double x, double y, double headingRadians) {
            if (pinpoint != null) {
                pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, x, y, AngleUnit.RADIANS, headingRadians));
            }
            poseValid = true;
        }

        /**
         * False at startup (the un-relocalized pose is not trustworthy for
         * aiming), true after setPose() (a corner relocalize) or a PoseStorage
         * load from init().
         */
        public boolean isPoseValid() {
            return poseValid;
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

    // =========================================================================
    // AutoAim: turn power toward the nearest goal aim point
    // =========================================================================
    /**
     * Chassis auto-aim toward the nearest goal aim point for the selected
     * alliance, using the Pinpoint pose. Purely math: it computes the turn
     * power; the caller (TeleOp/Drivetrain) decides when it may replace the
     * driver's turn stick.
     * <p>
     * All tunables are read from RobotConstants every update/compute so FTC
     * Dashboard edits take effect live.
     * <p>
     * Sign convention: AIM_TURN_SIGN is +1 because the Drivetrain's mecanum mix
     * gives the left wheels +turn and the right wheels -turn, so a POSITIVE
     * turn input spins the chassis CCW. The Pinpoint heading is CCW-positive,
     * so a positive heading error ("target is CCW of me") needs a positive
     * turn. If the robot ends up turning away from the goal, set
     * AIM_TURN_SIGN to -1 in FTC Dashboard.
     */
    public static final class AutoAim {
        private final Pinpoint localizer;

        private Alliance alliance = null;

        // Target selection state (recomputed by update()).
        private boolean targetKnown = false;
        private int targetIndex = 0;
        private double targetX = 0.0;
        private double targetY = 0.0;
        private double distanceInches = 0.0;

        // Heading state (recomputed by computeTurn()).
        private double headingError = 0.0;

        public AutoAim(Pinpoint localizer) {
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
         * Recomputes the target goal point from the current pose. Call once
         * per loop; keeps the current target index with
         * AIM_SWITCH_HYSTERESIS_IN of hysteresis so a standing robot
         * equidistant from both goals does not flip-flop.
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
            int nearest = nearestIndex(aimXs, aimYs, robotX, robotY);

            if (!targetKnown) {
                targetIndex = nearest;
                targetKnown = true;
            } else {
                // Only switch away from the current target when the other point is
                // closer by more than the hysteresis band.
                double currentDistance = distance(robotX, robotY, aimXs[targetIndex], aimYs[targetIndex]);
                double remainingDistance = distance(robotX, robotY, aimXs[nearest], aimYs[nearest]);
                if (nearest != targetIndex && remainingDistance + RobotConstants.AIM_SWITCH_HYSTERESIS_IN < currentDistance) {
                    targetIndex = nearest;
                }
            }

            targetX = aimXs[targetIndex];
            targetY = aimYs[targetIndex];
            distanceInches = distance(robotX, robotY, targetX, targetY);
        }

        /**
         * Turn power toward the target, added like the driver's turn input in
         * the mecanum mix (+ turns CCW). Returns 0 when aiming is disabled or
         * the pose is not valid.
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

            double desiredHeading = Math.atan2(targetY - robotY, targetX - robotX)
                    + Math.toRadians(RobotConstants.AIM_HEADING_OFFSET_DEG);
            headingError = angleWrap(desiredHeading - robotHeading);

            if (Math.abs(headingError) < RobotConstants.AIM_DEADBAND_RADIANS) {
                return 0.0;
            }

            double turn = Math.max(-RobotConstants.AIM_MAX_TURN,
                    Math.min(RobotConstants.AIM_MAX_TURN, RobotConstants.AIM_TURN_GAIN * headingError));
            return turn * RobotConstants.AIM_TURN_SIGN;
        }

        private static void validateArrays() {
            requireSameLength("RED_AIM_XS/YS", RobotConstants.RED_AIM_XS, RobotConstants.RED_AIM_YS);
            requireSameLength("BLUE_AIM_XS/YS", RobotConstants.BLUE_AIM_XS, RobotConstants.BLUE_AIM_YS);
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

    // =========================================================================
    // Relocalizer: snap the pose to the nearest corner point
    // =========================================================================
    /**
     * Corner relocalization: snaps the Pinpoint pose to the nearest known
     * corner point for the selected alliance. Heading per corner comes from
     * the RELOCALIZE_*_HEADINGS_DEG constants; a NaN entry means "trust the
     * Pinpoint's own heading and only fix X/Y".
     */
    public static final class Relocalizer {
        private final Pinpoint localizer;

        // Telemetry state for the most recent relocalize.
        private int lastIndex = -1;
        private boolean lastHeadingKept = false;

        public Relocalizer(Pinpoint localizer) {
            this.localizer = localizer;
            validateArrays();
        }

        /**
         * Sets the pose to the corner point nearest the current (possibly
         * bad) pose. Returns the chosen corner index, or -1 if there is no
         * alliance and nothing was done.
         */
        public int relocalize(Alliance alliance) {
            if (alliance == null) {
                return -1;
            }

            validateArrays();
            double[] cornerXs;
            double[] cornerYs;
            double[] cornerHeadingsDeg;
            if (alliance == Alliance.RED) {
                cornerXs = RobotConstants.RELOCALIZE_RED_XS;
                cornerYs = RobotConstants.RELOCALIZE_RED_YS;
                cornerHeadingsDeg = RobotConstants.RELOCALIZE_RED_HEADINGS_DEG;
            } else {
                cornerXs = RobotConstants.RELOCALIZE_BLUE_XS;
                cornerYs = RobotConstants.RELOCALIZE_BLUE_YS;
                cornerHeadingsDeg = RobotConstants.RELOCALIZE_BLUE_HEADINGS_DEG;
            }

            int index = nearestIndex(cornerXs, cornerYs,
                    localizer.getXInches(), localizer.getYInches());

            // NaN means "keep the current Pinpoint heading" for this corner.
            double heading = cornerHeadingsDeg[index];
            boolean keepHeading = Double.isNaN(heading);
            double newHeading = keepHeading
                    ? localizer.getHeadingRadians()
                    : Math.toRadians(heading);

            localizer.setPose(cornerXs[index], cornerYs[index], newHeading);

            lastIndex = index;
            lastHeadingKept = keepHeading;
            return index;
        }

        private static void validateArrays() {
            requireSameLength("RELOCALIZE_RED_XS/YS", RobotConstants.RELOCALIZE_RED_XS, RobotConstants.RELOCALIZE_RED_YS);
            requireSameLength("RELOCALIZE_RED_XS/Y/HEADINGS",
                    RobotConstants.RELOCALIZE_RED_XS, RobotConstants.RELOCALIZE_RED_HEADINGS_DEG);
            requireSameLength("RELOCALIZE_BLUE_XS/YS", RobotConstants.RELOCALIZE_BLUE_XS, RobotConstants.RELOCALIZE_BLUE_YS);
            requireSameLength("RELOCALIZE_BLUE_XS/Y/HEADINGS",
                    RobotConstants.RELOCALIZE_BLUE_XS, RobotConstants.RELOCALIZE_BLUE_HEADINGS_DEG);
        }

        /** Corner index used by the most recent relocalize, or -1 if none yet. */
        public int getLastIndex() {
            return lastIndex;
        }

        /** True when the last relocalize kept the Pinpoint heading (NaN constant). */
        public boolean isLastHeadingKept() {
            return lastHeadingKept;
        }
    }

    // =========================================================================
    // Field-geometry helpers (private: nothing outside Navigation needs them)
    // =========================================================================

    /**
     * Index of the point in (xs[i], ys[i]) closest to (x, y).
     * Throws if the arrays are missing, empty, or different lengths.
     */
    private static int nearestIndex(double[] xs, double[] ys, double x, double y) {
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
    private static double distance(double x1, double y1, double x2, double y2) {
        return Math.hypot(x2 - x1, y2 - y1);
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
    private static void requireSameLength(String label, double[] xs, double[] ys) {
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
