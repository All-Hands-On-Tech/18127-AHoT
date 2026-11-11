package org.firstinspires.ftc.teamcode.config;

import com.acmerobotics.dashboard.config.Config;

@Config("RobotConstants")
public final class RobotConstants {
    private RobotConstants() {
    }

    // Hardware names (mechanisms; drivetrain/localizer names live in pedro/Constants.java)
    public static String INTAKE_MOTOR = "intake";
    public static String SHOOTER_MOTOR = "shooter";
    public static String CONTINUOUS_SERVO_LEFT = "flower_right_servo";
    public static String CONTINUOUS_SERVO_RIGHT = "flower_left_servo";
    public static String BLOCKER_SERVO = "blocker_servo";
    public static String AXON_BLOCKER_SERVO = "axon_blocker_servo";

    // Driver controls
    public static double DRIVE_SPEED_SCALE = 1;
    public static double STRAFE_SPEED_SCALE = 0.8;
    public static double TURN_SPEED_SCALE = 0.8;
    public static double INTAKE_POWER = 1.0;

    // Flywheel (built-in motor velocity PIDF, tune live from Dashboard)
    public static double FLYWHEEL_TICKS_PER_SECOND = 1257.0;
    public static double FLYWHEEL_P = 67.0;
    public static double FLYWHEEL_I = 0.0;
    public static double FLYWHEEL_D = 0.0;
    public static double FLYWHEEL_F = 7.0;

    // Heading aim
    public static double AIM_TURN_GAIN = 1.0;
    public static double AIM_MAX_TURN = 0.40;
    public static double AIM_DEADBAND_RADIANS = Math.toRadians(1.0);
    public static double AIM_HEADING_OFFSET_DEG = 0.0;   // shooter angle relative to robot front
    public static double AIM_TURN_SIGN = 1.0;            // flip to -1 if it turns away from the goal
}