package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.pedroPathing.control.PIDFCoefficients;
import org.firstinspires.ftc.teamcode.pedroPathing.control.FilteredPIDFCoefficients;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.FollowerConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.paths.PathConstraints;
import org.firstinspires.ftc.teamcode.pedroPathing.Drivetrain;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.PinpointLocalizer;
import org.firstinspires.ftc.teamcode.common.RobotHardware;

public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(8.2)
            .forwardZeroPowerAcceleration(-25.9346931313679598)
            .lateralZeroPowerAcceleration(-67.342491844080064)
            .translationalPIDFCoefficients(new PIDFCoefficients(
                    0.03,
                    0,
                    0,
                    0.015
            ))
            .translationalPIDFSwitch(4)
            .secondaryTranslationalPIDFCoefficients(new PIDFCoefficients(
                    0.4,
                    0,
                    0.005,
                    0.0006
            ))
            .headingPIDFCoefficients(new PIDFCoefficients(
                    0.8,
                    0,
                    0,
                    0.01
            ))
            .secondaryHeadingPIDFCoefficients(new PIDFCoefficients(
                    2.5,
                    0,
                    0.1,
                    0.0005
            ))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(
                    0.1,
                    0,
                    0.00035,
                    0.6,
                    0.015
            ))
            .secondaryDrivePIDFCoefficients(new FilteredPIDFCoefficients(
                    0.02,
                    0,
                    0.000005,
                    0.6,
                    0.01
            ))
            .drivePIDFSwitch(15)
            .centripetalScaling(0.0005);

    /**
    These are the PathConstraints in order:
    tValueConstraint, velocityConstraint, translationalConstraint, headingConstraint, timeoutConstraint,
    brakingStrength, BEZIER_CURVE_SEARCH_LIMIT, brakingStart

    The BEZIER_CURVE_SEARCH_LIMIT should typically be left at 10 and shouldn't be changed.

    NOTE: All coordinates and velocities are in INCHES for this configuration.
    */
    public static PathConstraints pathConstraints = new PathConstraints(
            0.995,
            0.1,
            0.1,
            0.009,
            50,
            1.25,
            10,
            1
    );

    //Add custom localizers or drivetrains here
    public static Follower createFollower(HardwareMap hardwareMap) {
        RobotHardware hw = new RobotHardware(hardwareMap);
        hw.initPinpoint();

        PinpointLocalizer localizer = new PinpointLocalizer(hw);

        MecanumDrivetrain drivetrain = new MecanumDrivetrain(hw);

        return new Follower(followerConstants, localizer, drivetrain, pathConstraints);
    }
}
