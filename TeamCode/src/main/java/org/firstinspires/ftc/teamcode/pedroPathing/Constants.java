package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.pedropathing.control.PIDFCoefficients;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(8.2)  // Robot mass in kilograms
            .forwardZeroPowerAcceleration(-42.983767)
            .lateralZeroPowerAcceleration(-56.82688)
            .translationalPIDFCoefficients(new PIDFCoefficients(
                    0.12,
                    0.0,
                    0.025,
                    0.01
            ))
            .translationalPIDFSwitch(4)
            .headingPIDFCoefficients(new PIDFCoefficients(
                    1.567,
                    0,
                    0.034,
                    0.0
            ))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(
                    0.014,
                    0,
                    0.00025,
                    0.6,
                    0.0
            ))
            .drivePIDFSwitch(15)
            .centripetalScaling(0.0005);

    // Motor correction factors
    public static final double LEFT_FRONT_POWER = 1.0;
    public static final double LEFT_REAR_POWER = 1.0;
    public static final double RIGHT_FRONT_POWER = 1.0;
    public static final double RIGHT_REAR_POWER = 1.0;

    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .rightFrontMotorName("frontRight") //backLeft
            .rightRearMotorName("backRight") //frontLeft
            .leftRearMotorName("backLeft") //frontRight
            .leftFrontMotorName("frontLeft")  //backRight/
            // Match RobotHardware motor directions:
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .xVelocity(59.864243829)
            .yVelocity(47.518);

    public static final GoBildaPinpointDriver.EncoderDirection PINPOINT_FORWARD_DIR = GoBildaPinpointDriver.EncoderDirection.REVERSED;
    public static final GoBildaPinpointDriver.EncoderDirection PINPOINT_STRAFE_DIR = GoBildaPinpointDriver.EncoderDirection.REVERSED;

    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(4)  // Offset of forward pod from center of rotation (in inches)
            .strafePodX(1.6)   // Offset of strafe pod from center of rotation (in inches)
            .hardwareMapName("odo")  // Hardware device name in configuration
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(PINPOINT_FORWARD_DIR)
            .strafeEncoderDirection(PINPOINT_STRAFE_DIR);

    public static PathConstraints pathConstraints = new PathConstraints(
            0.99,   // tValueConstraint
            100,    // timeoutConstraint
            0.7,      // brakingStrength
            1       // brakingStart
    );

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .pinpointLocalizer(localizerConstants)
                .build();
    }
}
