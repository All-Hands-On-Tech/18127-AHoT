package org.firstinspires.ftc.teamcode.config;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Config("RobotConstants")
public final class RobotConstants {
    private RobotConstants() {
    }

    // Hardware names
    public static String FRONT_LEFT_DRIVE = "front_left_drive";
    public static String BACK_LEFT_DRIVE = "back_left_drive";
    public static String FRONT_RIGHT_DRIVE = "front_right_drive";
    public static String BACK_RIGHT_DRIVE = "back_right_drive";
    public static String INTAKE_MOTOR = "intake";
    public static String LEFT_FLYWHEEL_MOTOR = "left_flywheel";
    public static String RIGHT_FLYWHEEL_MOTOR = "right_flywheel";
    public static String CONTINUOUS_SERVO_LEFT = "continuous_servo_left";
    public static String CONTINUOUS_SERVO_RIGHT = "continuous_servo_right";
    public static String FLOWER_LEFT_SERVO = "flower_left_servo";
    public static String FLOWER_RIGHT_SERVO = "flower_right_servo";
    public static String BLOCKER_SERVO = "blocker_servo";
    public static String AXON_BLOCKER_SERVO = "axon_blocker_servo";
    public static String FLOWER_SERVO = "flower_servo";
    public static String PINPOINT = "pinpoint";

    //  - X pod = the pod that must count up when the robot drives forward.
    //  - Y pod = the pod that must count up when the robot strafes left.
    //  - X pod offset (mm): how far LEFT of the robot's center the X pod sits
    //    (positive = left). Y pod offset (mm): how far FORWARD of center the
    //    Y pod sits (positive = forward).
    public static GoBildaPinpointDriver.GoBildaOdometryPods PINPOINT_POD_TYPE =
            GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD;
    public static double PINPOINT_X_POD_OFFSET_MM = 0.0;
    public static double PINPOINT_Y_POD_OFFSET_MM = 0.0;
    public static GoBildaPinpointDriver.EncoderDirection PINPOINT_X_POD_DIRECTION =
            GoBildaPinpointDriver.EncoderDirection.FORWARD;
    public static GoBildaPinpointDriver.EncoderDirection PINPOINT_Y_POD_DIRECTION =
            GoBildaPinpointDriver.EncoderDirection.FORWARD;

    // Driver controls
    public static double DRIVE_SPEED_SCALE = 1;
    public static double STRAFE_SPEED_SCALE = 0.8;
    public static double TURN_SPEED_SCALE = 0.8;
    public static double INTAKE_POWER = 1.0;
    public static double FLYWHEEL_TICKS_PER_SECOND = 1480.0;
    public static double FLYWHEEL_KP = 0.045;
    public static double FLYWHEEL_KI = 0.003;
    public static double FLYWHEEL_KD = 0.002;
    public static double FLYWHEEL_KF = 0.00045;

    // Aiming positions in field coordinates, inches. The robot should aim at the
    // nearest target for its alliance.
    public static double[] RED_AIM_XS = {56.77, 57.41};
    public static double[] RED_AIM_YS = {58.15, 84.83};
    public static double[] BLUE_AIM_XS = {83.93, 84.25};
    public static double[] BLUE_AIM_YS = {56.88, 81.02};
    public static double AIM_TURN_GAIN = 1.0;
    public static double AIM_MAX_TURN = 0.40;
    public static double AIM_DEADBAND_RADIANS = Math.toRadians(1.0);
    public static double FLOWER_DEFAULT_POSITION = 0.18;
    public static double FLOWER_ACTIVE_POSITION = 0.82;
    public static double FLOWER_MIN_POSITION = 0.05;
    public static double FLOWER_MAX_POSITION = 1.0;

    public static boolean hasMotor(HardwareMap hardwareMap, String deviceName) {
        return hardwareMap != null && hardwareMap.tryGet(DcMotorEx.class, deviceName) != null;
    }

    public static boolean hasCRServo(HardwareMap hardwareMap, String deviceName) {
        return hardwareMap != null && hardwareMap.tryGet(CRServo.class, deviceName) != null;
    }
}
