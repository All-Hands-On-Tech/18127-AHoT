package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public final class Constants {
    private Constants() {
    }

    public static final String FRONT_LEFT_DRIVE = "front_left_drive";
    public static final String BACK_LEFT_DRIVE = "back_left_drive";
    public static final String FRONT_RIGHT_DRIVE = "front_right_drive";
    public static final String BACK_RIGHT_DRIVE = "back_right_drive";

    public static final String INTAKE_MOTOR = "intake";
    public static final String LEFT_FLYWHEEL_MOTOR = "left_flywheel";
    public static final String RIGHT_FLYWHEEL_MOTOR = "right_flywheel";
    public static final String LIMELIGHT_NAME = "limelight";
    public static final int LIMELIGHT_PIPELINE = 0;

    public static final double DRIVE_SPEED_SCALE = 0.80;
    public static final double STRAFE_SPEED_SCALE = 0.80;
    public static final double TURN_SPEED_SCALE = 0.70;
    public static final double INTAKE_POWER = 1.0;
    public static final double FLYWHEEL_TICKS_PER_SECOND = 2500.0;

    public static final double AIM_TURN_GAIN = 0.020;
    public static final double AIM_MAX_TURN = 0.40;
    public static final double AIM_DEADBAND_DEGREES = 1.0;

    public static final int[] RED_TARGET_IDS = {1, 2, 3};
    public static final int[] BLUE_TARGET_IDS = {4, 5, 6};

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

}