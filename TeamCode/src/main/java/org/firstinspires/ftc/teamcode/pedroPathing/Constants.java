package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * IMPORTANT MIGRATION NOTE:
 * This project uses Pedro Pathing 2.0.3 which has a COMPLETELY DIFFERENT API than older versions.
 *
 * The old constants file used methods that NO LONGER EXIST in 2.0.3:
 * - forwardZeroPowerAcceleration(-25.9346931313679598)
 * - lateralZeroPowerAcceleration(-67.342491844080064)
 * - translationalPIDFCoefficients, secondaryTranslationalPIDFCoefficients
 * - headingPIDFCoefficients, secondaryHeadingPIDFCoefficients
 * - drivePIDFCoefficients, secondaryDrivePIDFCoefficients
 * - translationalPIDFSwitch, drivePIDFSwitch
 * - centripetalScaling
 *
 * Pedro Pathing 2.0.3 uses a different tuning system. You will need to:
 * 1. Re-tune your robot using the 2.0.3 tuning OpModes
 * 2. Check the Pedro Pathing 2.0.3 documentation for the new API
 * 3. Or downgrade to an older version that supports these methods
 */
public class Constants {

    // Follower Constants - Robot mass (PIDF tuning methods removed - incompatible with 2.0.3)
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(8.2);  // Robot mass in kilograms

    /**
     * Path Constraints parameters (from old config):
     * - tValueConstraint: 0.995
     * - velocityConstraint: 0.1
     * - translationalConstraint: 0.1
     * - headingConstraint: 0.009
     * - timeoutConstraint: 50
     * - brakingStrength: 1.25
     * - BEZIER_CURVE_SEARCH_LIMIT: 10
     * - brakingStart: 1
     *
     * NOTE: Pedro Pathing 2.0.3 uses a different PathConstraints constructor.
     * The new constructor takes: (maxVelocity, maxAcceleration, maxAngularVelocity, maxAngularAcceleration)
     * You will need to convert your old values or re-tune for 2.0.3.
     */
    public static PathConstraints pathConstraints = new PathConstraints(
            0.99,   // maxVelocity (placeholder - needs tuning for 2.0.3)
            100,    // maxAcceleration (placeholder - needs tuning for 2.0.3)
            1,      // maxAngularVelocity (placeholder - needs tuning for 2.0.3)
            1       // maxAngularAcceleration (placeholder - needs tuning for 2.0.3)
    );

    // Mecanum Drive Constants - motor configuration
    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .rightFrontMotorName("frontRight")
            .rightRearMotorName("backRight")
            .leftRearMotorName("backLeft")
            .leftFrontMotorName("frontLeft")
            // Match RobotHardware motor directions:
            // RobotHardware sets frontLeft=FORWARD, frontRight=REVERSE, backLeft=FORWARD, backRight=REVERSE
            .leftFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .leftRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightRearMotorDirection(DcMotorSimple.Direction.REVERSE);

    // Pinpoint Localizer Constants
    // Offsets are where the odometry pods are relative to the robot's center of rotation
    // forwardPodY: offset of the forward (parallel) pod in the Y direction
    // strafePodX: offset of the strafe (perpendicular) pod in the X direction
    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(0.75)  // Offset of forward pod from center of rotation (in inches)
            .strafePodX(4.25)   // Offset of strafe pod from center of rotation (in inches)
            .distanceUnit(DistanceUnit.INCH)  // Using inches for measurements
            .hardwareMapName("odo")  // Hardware device name in configuration
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            // NOTE: Use FORWARD here if your odometry pods are wired/installed such that
            // forward motion increases the forward (X) encoder and left motion increases the strafe (Y) encoder.
            // PinpointDiagnostic (TeleOp) sets both to FORWARD by default; matching that avoids inverted axes in the localizer.
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD);

    public static Follower createFollower(HardwareMap hardwareMap) {
        // Create follower using FollowerBuilder with Pinpoint localizer
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .pinpointLocalizer(localizerConstants)
                .build();
    }
}
