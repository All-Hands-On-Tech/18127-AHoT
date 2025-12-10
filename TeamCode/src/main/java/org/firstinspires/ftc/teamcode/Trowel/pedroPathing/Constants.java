package org.firstinspires.ftc.teamcode.Trowel.pedroPathing;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(0)  // TODO: Tune manually - Robot mass in kilograms
            .forwardZeroPowerAcceleration(0)  // TODO: Tune manually
            .lateralZeroPowerAcceleration(0)  // TODO: Tune manually
            .translationalPIDFCoefficients(new PIDFCoefficients(
                    0,  // TODO: Tune manually
                    0,
                    0,
                    0
            ))
            .translationalPIDFSwitch(0)  // TODO: Tune manually
            .headingPIDFCoefficients(new PIDFCoefficients(
                    0,  // TODO: Tune manually
                    0,
                    0,
                    0
            ))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(
                    0,  // TODO: Tune manually
                    0,
                    0,
                    0,
                    0
            ))
            .drivePIDFSwitch(0)  // TODO: Tune manually
            .centripetalScaling(0);  // TODO: Tune manually

    // Motor correction factors
    public static final double LEFT_FRONT_POWER = 1.0;
    public static final double LEFT_REAR_POWER = 1.0;
    public static final double RIGHT_FRONT_POWER = 1.0;
    public static final double RIGHT_REAR_POWER = 1.0;

    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1.0)  // Full speed
            .rightFrontMotorName("frontRight")
            .rightRearMotorName("backRight")
            .leftRearMotorName("backLeft")
            .leftFrontMotorName("frontLeft")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .xVelocity(0)  // TODO: Tune manually
            .yVelocity(0);  // TODO: Tune manually

    public static final GoBildaPinpointDriver.EncoderDirection PINPOINT_FORWARD_DIR = GoBildaPinpointDriver.EncoderDirection.REVERSED;
    public static final GoBildaPinpointDriver.EncoderDirection PINPOINT_STRAFE_DIR = GoBildaPinpointDriver.EncoderDirection.REVERSED;

    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(0)  // TODO: Tune manually - Offset of forward pod from center of rotation (in inches)
            .strafePodX(0)   // TODO: Tune manually - Offset of strafe pod from center of rotation (in inches)
            .hardwareMapName("odo")  // Hardware device name in configuration
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(PINPOINT_FORWARD_DIR)  // TODO: Verify direction on robot
            .strafeEncoderDirection(PINPOINT_STRAFE_DIR);   // TODO: Verify direction on robot

    public static PathConstraints pathConstraints = new PathConstraints(
            0,      // TODO: Tune manually - tValueConstraint
            100,    // timeoutConstraint
            0,      // TODO: Tune manually - brakingStrength
            0       // TODO: Tune manually - brakingStart
    );


    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .pinpointLocalizer(localizerConstants)
                .build();
    }
}
