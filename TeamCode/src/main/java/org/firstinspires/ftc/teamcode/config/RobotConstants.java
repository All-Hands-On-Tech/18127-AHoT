package org.firstinspires.ftc.teamcode.config;

import com.pedropathing.revhub.localizers.Encoder;
import com.pedropathing.revhub.localizers.RevHubIMU;
import com.pedropathing.revhub.localizers.TwoWheelConfig;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public final class RobotConstants {
    private RobotConstants() {
    }

    // Hardware names
    public static final String FRONT_LEFT_DRIVE = "front_left_drive";
    public static final String BACK_LEFT_DRIVE = "back_left_drive";
    public static final String FRONT_RIGHT_DRIVE = "front_right_drive";
    public static final String BACK_RIGHT_DRIVE = "back_right_drive";
    public static final String INTAKE_MOTOR = "intake";
    public static final String LEFT_FLYWHEEL_MOTOR = "left_flywheel";
    public static final String RIGHT_FLYWHEEL_MOTOR = "right_flywheel";
    public static final String FORWARD_ODOMETRY_POD = "lf";
    public static final String STRAFE_ODOMETRY_POD = "rr";
    public static final String IMU = "imu";

    // Odometry calibration. Replace these with TwoWheelTuner output.
    public static final double FORWARD_TICKS_TO_INCHES = 1.0;
    public static final double STRAFE_TICKS_TO_INCHES = 1.0;
    public static final double FORWARD_POD_OFFSET = 0.0;
    public static final double STRAFE_POD_OFFSET = 0.0;
    public static final double FORWARD_ENCODER_DIRECTION = Encoder.FORWARD;
    public static final double STRAFE_ENCODER_DIRECTION = Encoder.FORWARD;
    public static final RevHubOrientationOnRobot.LogoFacingDirection IMU_LOGO_DIRECTION =
            RevHubOrientationOnRobot.LogoFacingDirection.UP;
    public static final RevHubOrientationOnRobot.UsbFacingDirection IMU_USB_DIRECTION =
            RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD;

    // Driver controls
    public static final double DRIVE_SPEED_SCALE = 0.80;
    public static final double STRAFE_SPEED_SCALE = 0.80;
    public static final double TURN_SPEED_SCALE = 0.70;
    public static final double INTAKE_POWER = 1.0;
    public static final double FLYWHEEL_TICKS_PER_SECOND = 2500.0;

    // Aiming positions and controller
    public static final double RED_AIM_X = 144.0;
    public static final double RED_AIM_Y = -36.0;
    public static final double BLUE_AIM_X = 144.0;
    public static final double BLUE_AIM_Y = 36.0;
    public static final double AIM_TURN_GAIN = 1.0;
    public static final double AIM_MAX_TURN = 0.40;
    public static final double AIM_DEADBAND_RADIANS = Math.toRadians(1.0);

    public static Mecanum createDrive(HardwareMap hardwareMap) {
        return new Mecanum(hardwareMap, new MecanumConfig(config -> {
            config.frontLeftName.set(FRONT_LEFT_DRIVE);
            config.backLeftName.set(BACK_LEFT_DRIVE);
            config.frontRightName.set(FRONT_RIGHT_DRIVE);
            config.backRightName.set(BACK_RIGHT_DRIVE);
            config.frontLeftDirection.set(DcMotorSimple.Direction.REVERSE);
            config.backLeftDirection.set(DcMotorSimple.Direction.REVERSE);
            config.frontRightDirection.set(DcMotorSimple.Direction.FORWARD);
            config.backRightDirection.set(DcMotorSimple.Direction.FORWARD);
            config.manualBrakeMode.set(true);
            config.powerThreshold.set(0.0);
        }));
    }

    public static TwoWheelConfig createOdometryConfig() {
        return new TwoWheelConfig(config -> {
            config.xPodName.set(FORWARD_ODOMETRY_POD);
            config.yPodName.set(STRAFE_ODOMETRY_POD);
            config.imuName.set(IMU);
            config.xPodOffset.set(FORWARD_POD_OFFSET);
            config.yPodOffset.set(STRAFE_POD_OFFSET);
            config.forwardTicksToInches.set(FORWARD_TICKS_TO_INCHES);
            config.strafeTicksToInches.set(STRAFE_TICKS_TO_INCHES);
            config.xPodDirection.set(FORWARD_ENCODER_DIRECTION);
            config.yPodDirection.set(STRAFE_ENCODER_DIRECTION);
            config.imu.set(new RevHubIMU(new RevHubOrientationOnRobot(
                    IMU_LOGO_DIRECTION, IMU_USB_DIRECTION)));
        });
    }
}
