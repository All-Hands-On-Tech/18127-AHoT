package org.firstinspires.ftc.teamcode.auxiliary;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.drive.Alliance;
import org.firstinspires.ftc.teamcode.drive.PinpointOdometry;

/**
 * TeleOp pose source on top of the shared GoBilda Pinpoint. Extends the
 * existing {@link PinpointOdometry} wrapper (which owns the Pinpoint device
 * and its configuration) instead of duplicating it; adds the pose-validity
 * tracking the auto-aim and relocalization code needs.
 * <p>
 * update() must be called exactly once per OpMode loop, before anything else
 * reads the pose.
 */
public class PinpointLocalizer extends PinpointOdometry {

    /** True once the pose has been relocalized or loaded from PoseStorage. */
    private boolean poseValid = false;

    public PinpointLocalizer(HardwareMap hardwareMap) {
        super(hardwareMap);
    }

    public double getXInches() {
        return getX();
    }

    public double getYInches() {
        return getY();
    }

    /**
     * Heading in radians, CCW positive, wrapped to -pi..pi. The Pinpoint
     * reports within +/-180, but the wrap here guarantees the contract in case
     * a firmware change removes it.
     */
    @Override
    public double getHeadingRadians() {
        return FieldGeometry.angleWrap(super.getHeadingRadians());
    }

    /** Overwrite the Pinpoint pose, e.g. from a corner relocalize or PoseStorage. */
    @Override
    public void setPose(double x, double y, double headingRadians) {
        super.setPose(x, y, headingRadians);
        poseValid = true;
    }

    /**
     * False at startup (the un-relocalized pose is not trustworthy for aiming),
     * true after setPose() (a corner relocalize) or a PoseStorage load from
     * init().
     */
    public boolean isPoseValid() {
        return poseValid;
    }
}
