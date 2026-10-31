package org.firstinspires.ftc.teamcode.auxiliary;

import org.firstinspires.ftc.teamcode.config.RobotConstants;
import org.firstinspires.ftc.teamcode.drive.Alliance;

/**
 * Corner relocalization: snaps the Pinpoint pose to the nearest known corner
 * point for the selected alliance. Heading per corner comes from the
 * RELOCALIZE_*_HEADINGS_DEG constants; a NaN entry means "trust the Pinpoint's
 * own heading and only fix X/Y".
 */
public class Relocalizer {
    private final PinpointLocalizer localizer;

    // Telemetry state for the most recent relocalize.
    private int lastIndex = -1;
    private boolean lastHeadingKept = false;

    public Relocalizer(PinpointLocalizer localizer) {
        this.localizer = localizer;
        validateArrays();
    }

    /**
     * Sets the pose to the corner point nearest the current (possibly bad)
     * pose. Returns the chosen corner index, or -1 if there is no alliance and
     * nothing was done.
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

        int index = FieldGeometry.nearestIndex(cornerXs, cornerYs,
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
        FieldGeometry.requireSameLength("RELOCALIZE_RED_XS/YS", RobotConstants.RELOCALIZE_RED_XS, RobotConstants.RELOCALIZE_RED_YS);
        FieldGeometry.requireSameLength("RELOCALIZE_RED_XS/Y/HEADINGS",
                RobotConstants.RELOCALIZE_RED_XS, RobotConstants.RELOCALIZE_RED_HEADINGS_DEG);
        FieldGeometry.requireSameLength("RELOCALIZE_BLUE_XS/YS", RobotConstants.RELOCALIZE_BLUE_XS, RobotConstants.RELOCALIZE_BLUE_YS);
        FieldGeometry.requireSameLength("RELOCALIZE_BLUE_XS/Y/HEADINGS",
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