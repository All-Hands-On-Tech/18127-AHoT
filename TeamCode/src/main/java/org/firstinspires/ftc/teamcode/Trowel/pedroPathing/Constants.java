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

import com.pedropathing.paths.PathChain;

public class Constants {

    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(10.7955)  // TODO: Tune manually - Robot mass in kilograms
            .forwardZeroPowerAcceleration(-14.438)  // TODO: Tune manually
            .lateralZeroPowerAcceleration(-66.447)  // TODO: Tune manually
            .translationalPIDFCoefficients(new PIDFCoefficients(
                    0.062,  // TODO: Tune manually
                    0,
                    0.021,
                    0.02
            ))// TODO: Tune manually
            .headingPIDFCoefficients(new PIDFCoefficients(
                    1.5,  // Increased P for stronger auto aiming
                    0,
                    0.06,
                    0
            ))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(
                    0.01,
                    0,
                    0.000,
                    0.6,
                    0.0
            ))// TODO: Tune manually
            .centripetalScaling(0.0005);  // TODO: Tune manually

    // Motor correction factors (multiplicative). Normally set to 1.0.
    // If a motor needs to be inverted in software instead of changing its hardware direction,
    // set its multiplier to -1.0 (for example LEFT_FRONT_POWER = -1.0).
    // These are non-final so you can change them at runtime or via a config panel if desired.
    public static double LEFT_FRONT_POWER = 1.0;
    public static double LEFT_REAR_POWER = 1.0;
    public static double RIGHT_FRONT_POWER = 1.0;
    public static double RIGHT_REAR_POWER = 1.0;

    // ========== Hardware-spec constants (no manual tuning required) ==========
    // Wheel and motor/encoder hardware specifications
    public static final double WHEEL_RADIUS_IN = 2.0;      // inches
    public static final double TICKS_PER_REV = 537.6;      // encoder ticks per motor revolution
    public static final double GEAR_RATIO = 1.0;           // output (wheel) revs per motor rev
    public static final double MAX_RPM = 435.0;            // motor free speed (RPM)


    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1.0)  // Full speed
            .rightFrontMotorName("frontRight")
            .rightRearMotorName("backRight")
            .leftRearMotorName("backLeft")
            .leftFrontMotorName("frontLeft")
            .leftFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .leftRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            // Match the Spork config: right motors forward, left motors reversed.
            // Change these directions if your physical wiring/gearbox requires flipping.
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            // Use hardware-derived theoretical velocity so no manual tuning required here
            .xVelocity(72.772)
            .yVelocity(68.001);

    // Pinpoint/localizer encoder directions — flip if odometry reports reversed axes.
    public static final GoBildaPinpointDriver.EncoderDirection PINPOINT_FORWARD_DIR = GoBildaPinpointDriver.EncoderDirection.REVERSED;
    public static final GoBildaPinpointDriver.EncoderDirection PINPOINT_STRAFE_DIR = GoBildaPinpointDriver.EncoderDirection.REVERSED;

    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(0.1)  // TODO: Tune manually - Offset of forward pod from center of rotation (in inches)
            .strafePodX(8)   // TODO: Tune manually - Offset of strafe pod from center of rotation (in inches)
            .hardwareMapName("odo")  // Hardware device name in configuration
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(PINPOINT_FORWARD_DIR)  // TODO: Verify direction on robot
            .strafeEncoderDirection(PINPOINT_STRAFE_DIR);   // TODO: Verify direction on robot

    public static PathConstraints pathConstraints = new PathConstraints(
            0.99,      // TODO: Tune manually - tValueConstraint
            100,    // timeoutConstraint
            1,      // TODO: Tune manually - brakingStrength
            2       // TODO: Tune manually - brakingStart
    );


    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .pinpointLocalizer(localizerConstants)
                .build();
    }

    public static void setMotorPowerMultiplier(String motor, double multiplier) {
        switch (motor) {
            case "LEFT_FRONT":
                LEFT_FRONT_POWER = multiplier;
                break;
            case "LEFT_REAR":
                LEFT_REAR_POWER = multiplier;
                break;
            case "RIGHT_FRONT":
                RIGHT_FRONT_POWER = multiplier;
                break;
            case "RIGHT_REAR":
                RIGHT_REAR_POWER = multiplier;
                break;
            default:
                throw new IllegalArgumentException("Invalid motor name: " + motor);
        }
    }

    public static PathChain mirror(PathChain chain) {
        if (chain == null) return null;
        try {
            // Pedro Pathing PathChain exposes mirror(); use it when available
            java.lang.reflect.Method m = chain.getClass().getMethod("mirror");
            Object mirrored = m.invoke(chain);
            if (mirrored instanceof PathChain) return (PathChain) mirrored;
        } catch (Exception ignored) {
        }
        // Fallback: return original chain if mirror unavailable
        return chain;
    }

}
