package org.firstinspires.ftc.teamcode.drive;

import com.pedropathing.math.Pose;
import com.pedropathing.revhub.localizers.TwoWheelLocalizer;
import com.pedropathing.utils.Angle;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.config.RobotConstants;

public class OdometryAimAssist {
    private final TwoWheelLocalizer localizer;
    private Pose pose = Pose.zero();
    private double headingError;

    public OdometryAimAssist(HardwareMap hardwareMap) {
        localizer = new TwoWheelLocalizer(hardwareMap, RobotConstants.createOdometryConfig());
        localizer.setPose(Pose.zero());
    }

    public double getTurnCorrection(Alliance alliance) {
        localizer.update();
        pose = localizer.pose();

        double targetX = alliance == Alliance.RED ? RobotConstants.RED_AIM_X : RobotConstants.BLUE_AIM_X;
        double targetY = alliance == Alliance.RED ? RobotConstants.RED_AIM_Y : RobotConstants.BLUE_AIM_Y;
        double targetHeading = Math.atan2(targetY - pose.y(), targetX - pose.x());
        headingError = Angle.normalizeSigned(targetHeading - pose.heading());

        if (Math.abs(headingError) < RobotConstants.AIM_DEADBAND_RADIANS) {
            return 0.0;
        }
        return Range.clip(
                headingError * RobotConstants.AIM_TURN_GAIN,
                -RobotConstants.AIM_MAX_TURN,
                RobotConstants.AIM_MAX_TURN);
    }

    public Pose getPose() {
        return pose;
    }

    public double getHeadingError() {
        return headingError;
    }
}
