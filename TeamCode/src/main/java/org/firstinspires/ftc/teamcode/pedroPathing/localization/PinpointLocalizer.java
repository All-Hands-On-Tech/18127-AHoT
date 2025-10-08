package org.firstinspires.ftc.teamcode.pedroPathing.localization;

import org.firstinspires.ftc.teamcode.pedroPathing.geometry.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.math.Vector;
import org.firstinspires.ftc.teamcode.common.RobotHardware;
import org.firstinspires.ftc.teamcode.common.Odometry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/**
 * PinpointLocalizer - Adapter for Pedro Pathing to use GoBilda Pinpoint odometry
 */
public class PinpointLocalizer implements Localizer {
    private final RobotHardware hw;
    private Pose startPose = new Pose(0, 0, 0);
    private Pose currentPose = new Pose(0, 0, 0);

    public PinpointLocalizer(RobotHardware hw) {
        this.hw = hw;
    }

    @Override
    public Pose getPose() {
        if (hw.pinpoint != null) {
            double x = hw.pinpoint.getPosX(DistanceUnit.INCH);
            double y = hw.pinpoint.getPosY(DistanceUnit.INCH);
            double heading = hw.pinpoint.getHeading(AngleUnit.RADIANS);
            currentPose = new Pose(x, y, heading);
        }
        return currentPose;
    }

    @Override
    public Pose getVelocity() {
        if (hw.pinpoint != null) {
            double vx = hw.pinpoint.getVelX(DistanceUnit.INCH);
            double vy = hw.pinpoint.getVelY(DistanceUnit.INCH);
            // Use getHeadingVelocity with AngleUnit (not UnnormalizedAngleUnit)
            double vHeading = 0; // Pinpoint doesn't provide heading velocity directly
            return new Pose(vx, vy, vHeading);
        }
        return new Pose(0, 0, 0);
    }

    @Override
    public Vector getVelocityVector() {
        Pose vel = getVelocity();
        return new Vector(vel);
    }

    @Override
    public void setStartPose(Pose setStart) {
        this.startPose = setStart;
        if (hw.pinpoint != null) {
            hw.pinpoint.setPosition(new org.firstinspires.ftc.robotcore.external.navigation.Pose2D(
                DistanceUnit.INCH,
                setStart.getX(),
                setStart.getY(),
                AngleUnit.RADIANS,
                setStart.getHeading()
            ));
        }
    }

    @Override
    public void setPose(Pose setPose) {
        this.currentPose = setPose;
        if (hw.pinpoint != null) {
            hw.pinpoint.setPosition(new org.firstinspires.ftc.robotcore.external.navigation.Pose2D(
                DistanceUnit.INCH,
                setPose.getX(),
                setPose.getY(),
                AngleUnit.RADIANS,
                setPose.getHeading()
            ));
        }
    }

    @Override
    public void update() {
        hw.updatePinpoint();
    }

    @Override
    public double getTotalHeading() {
        if (hw.pinpoint != null) {
            return hw.pinpoint.getHeading(AngleUnit.RADIANS);
        }
        return 0;
    }

    @Override
    public void resetIMU() {
        if (hw.pinpoint != null) {
            hw.pinpoint.recalibrateIMU();
        }
    }

    @Override
    public double getForwardMultiplier() {
        return 1.0; // Pinpoint handles calibration internally
    }

    @Override
    public double getLateralMultiplier() {
        return 1.0; // Pinpoint handles calibration internally
    }

    @Override
    public double getTurningMultiplier() {
        return 1.0; // Pinpoint handles calibration internally
    }

    @Override
    public boolean isNAN() {
        if (hw.pinpoint == null) return true;
        Pose pose = getPose();
        return Double.isNaN(pose.getX()) || Double.isNaN(pose.getY()) || Double.isNaN(pose.getHeading());
    }

    @Override
    public double getIMUHeading() {
        if (hw.pinpoint != null) {
            return hw.pinpoint.getHeading(AngleUnit.RADIANS);
        }
        return 0;
    }
}
