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
            .useSecondaryTranslationalPIDF(true)
            .forwardZeroPowerAcceleration(-42.983767)
            .lateralZeroPowerAcceleration(-56.82688)
            .translationalPIDFCoefficients(new PIDFCoefficients(
                    0.07,
                    0.0,
                    0.0,
                    0.03
            ))
            .translationalPIDFSwitch(4)
            .secondaryTranslationalPIDFCoefficients(new PIDFCoefficients(
                    0.08,
                    0.0,
                    0.01,
                    0.03
            ))
            .headingPIDFCoefficients(new PIDFCoefficients(
                    0.08, // Slightly increased P for more responsiveness
                    0,    // I remains 0
                    0.35, // Slightly increased D for more damping
                    0.025 // Slightly decreased F for less minimum output
            ))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(
                    0.1,
                    0,
                    0.00035,
                    0.6,
                    0.015
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
            .rightFrontMotorName("frontRight")
            .rightRearMotorName("backRight")
            .leftRearMotorName("backLeft")
            .leftFrontMotorName("frontLeft")
            // Match RobotHardware motor directions:
            .leftFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .leftRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .xVelocity(59.96108)
            .yVelocity(47.518);

    public static final GoBildaPinpointDriver.EncoderDirection PINPOINT_FORWARD_DIR = GoBildaPinpointDriver.EncoderDirection.REVERSED;
    public static final GoBildaPinpointDriver.EncoderDirection PINPOINT_STRAFE_DIR = GoBildaPinpointDriver.EncoderDirection.REVERSED;

    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(0.75)  // Offset of forward pod from center of rotation (in inches)
            .strafePodX(4.25)   // Offset of strafe pod from center of rotation (in inches)
            .hardwareMapName("odo")  // Hardware device name in configuration
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(PINPOINT_FORWARD_DIR)
            .strafeEncoderDirection(PINPOINT_STRAFE_DIR);

    public static PathConstraints pathConstraints = new PathConstraints(
            0.99,   // maxVelocity
            100,    // maxAcceleration
            1,      // maxAngularVelocity
            1       // maxAngularAcceleration
    );

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .pinpointLocalizer(localizerConstants)
                .build();
    }
}
