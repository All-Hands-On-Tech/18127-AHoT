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
    public static String CONTINUOUS_SERVO_LEFT = "flower_right_servo";
    public static String CONTINUOUS_SERVO_RIGHT = "flower_left_servo";
    public static String BLOCKER_SERVO = "blocker_servo";
    public static String AXON_BLOCKER_SERVO = "axon_blocker_servo";
    public static String PINPOINT = "pinpoint";

    //  - X pod = the pod that must count up when the robot drives forward.
    //  - Y pod = the pod that must count up when the robot strafes left.
    //  - X pod offset (mm): how far LEFT of the robot's center the X pod sits
    //    (positive = left). Y pod offset (mm): how far FORWARD of center the
    //    Y pod sits (positive = forward).
    public static GoBildaPinpointDriver.GoBildaOdometryPods PINPOINT_POD_TYPE =
            GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD;
    public static double PINPOINT_X_POD_OFFSET_MM = -64.4059753417969;  // -2.5356683205431843 in * 25.4
    public static double PINPOINT_Y_POD_OFFSET_MM = 224.4959411621094;  //  8.838422880398008 in * 25.4
    public static GoBildaPinpointDriver.EncoderDirection PINPOINT_X_POD_DIRECTION =
            GoBildaPinpointDriver.EncoderDirection.REVERSED;
    public static GoBildaPinpointDriver.EncoderDirection PINPOINT_Y_POD_DIRECTION =
            GoBildaPinpointDriver.EncoderDirection.REVERSED;

    // Driver controls
    public static double DRIVE_SPEED_SCALE = 1;
    public static double STRAFE_SPEED_SCALE = 0.8;
    public static double TURN_SPEED_SCALE = 0.8;
    public static double INTAKE_POWER = 1.0;
    public static double FLYWHEEL_TICKS_PER_SECOND = 1550.0;



    public static double FLYWHEEL_KS = 0.055097;
    public static double FLYWHEEL_KV = 0.000385428;
    public static double FLYWHEEL_KP = 0.001532730;
    public static double FLYWHEEL_KI = 0.003737268;
    public static double FLYWHEEL_KD = 0.0;       // tuner leaves this at 0 (velocity is too noisy)

    // Control-law parameters the tuner used while producing the constants above.
    public static double FLYWHEEL_NOMINAL_VOLTAGE = 12.0;  // voltage the constants were tuned at
    public static double FLYWHEEL_BOOST_FRACTION = 0.15;   // full power when error > this fraction of target (coast when below -this)
    public static double FLYWHEEL_INTEGRAL_LIMIT = 0.25;   // max power contribution of the integral term (anti-windup)
    public static double FLYWHEEL_READY_BAND = 0.025;      // within +-2.5% of target counts as "at speed"

    // Aiming positions in field coordinates, inches. The robot should aim at the
    // nearest target for its alliance.
    public static double[] RED_AIM_XS = {56.77, 57.41};
    public static double[] RED_AIM_YS = {58.15, 84.83};
    public static double[] BLUE_AIM_XS = {83.93, 84.25};
    public static double[] BLUE_AIM_YS = {56.88, 81.02};
    public static double AIM_TURN_GAIN = 1.0;
    public static double AIM_MAX_TURN = 0.40;
    public static double AIM_DEADBAND_RADIANS = Math.toRadians(1.0);

    // Auto-aim / corner relocalization (inches, CCW-positive headings).
    public static double[] RELOCALIZE_RED_XS = {8.4, 8.4};
    public static double[] RELOCALIZE_RED_YS = {9.1, 133.1};
    public static double[] RELOCALIZE_BLUE_XS = {133.4, 132.5};
    public static double[] RELOCALIZE_BLUE_YS = {133.1, 9.1};
    public static double[] RELOCALIZE_RED_HEADINGS_DEG = {Double.NaN, Double.NaN};   // NaN = keep current heading
    public static double[] RELOCALIZE_BLUE_HEADINGS_DEG = {Double.NaN, Double.NaN};
    public static double AIM_SWITCH_HYSTERESIS_IN = 3.0;
    public static double AIM_HEADING_OFFSET_DEG = 0.0;   // angle of the shooter relative to robot front
    public static double AIM_TURN_SIGN = 1.0;            // flip to -1 if the robot turns away from the goal
    public static boolean AIM_REQUIRES_VALID_POSE = true;

    // Basic time-based autonomous (no Pedro Pathing).
    public static double AUTO_DRIVE_DISTANCE_IN = 10.0;  // forward distance before shooting
    public static double AUTO_START_HEADING_DEG = 90.0;  // heading the pose starts at (robot's facing at init)
    public static double AUTO_DRIVE_POWER = 0.5;
    public static double AUTO_DRIVE_TIMEOUT_S = 8.0;      // safety net if the Pinpoint is unavailable
    public static double AUTO_HEADING_HOLD_GAIN = 0.5;    // turn power per radian of heading error
    public static double AUTO_HEADING_MAX_TURN = 0.3;     // clamp on the straight-line heading correction
    public static double AUTO_SPIN_UP_SECONDS = 3.0;      // flywheel spin-up before opening the gate
    public static double AUTO_GATE_WAIT_SECONDS = 1.0;    // gate open before feeding starts
    public static double AUTO_INTAKE1_SECONDS = 4.0;      // intake forward run
    public static double AUTO_REVERSE_SECONDS = 0.35 ;      // intake reverse kick (unjam)
    public static double AUTO_INTAKE2_SECONDS = 3.0;      // final intake forward run


    public static boolean hasMotor(HardwareMap hardwareMap, String deviceName) {
        return hardwareMap != null && hardwareMap.tryGet(DcMotorEx.class, deviceName) != null;
    }

    public static boolean hasCRServo(HardwareMap hardwareMap, String deviceName) {
        return hardwareMap != null && hardwareMap.tryGet(CRServo.class, deviceName) != null;
    }
}