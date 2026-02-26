package org.firstinspires.ftc.teamcode.Trowel.Configs;

import com.bylazar.configurables.annotations.Configurable;

/**
 * Configurables for the Trowel TeleOp.
 * All values can be tuned in real-time via Panels Dashboard.
 */
@Configurable
public class TrowelConfigs {

    // ══════════════════════════════════════════════════════════════════════════
    // APRILTAG FIELD POSITIONS — ABSOLUTE COORDINATES  (odometry correction)
    // ══════════════════════════════════════════════════════════════════════════

    public static double BLUE_TAG_X = 16.079207920792072;
    public static double BLUE_TAG_Y = 131.32673267326734;
    public static double RED_TAG_X  = 129.42574257425744;
    public static double RED_TAG_Y  = 131.326;

    // ══════════════════════════════════════════════════════════════════════════
    // GOAL COORDINATES — used only for telemetry distance readout
    // ══════════════════════════════════════════════════════════════════════════

    public static double BLUE_GOAL_X = 9.491675915649276;
    public static double BLUE_GOAL_Y = 138.8945615982242;
    public static double RED_GOAL_X  = 132.0;
    public static double RED_GOAL_Y  = 138.8945615982242;

    // ══════════════════════════════════════════════════════════════════════════
    // ALLIANCE TAG ID RANGES
    // ══════════════════════════════════════════════════════════════════════════

    public static int BLUE_TAG_ID_MIN = 11;
    public static int BLUE_TAG_ID_MAX = 14;
    public static int RED_TAG_ID_MIN  = 15;
    public static int RED_TAG_ID_MAX  = 16;

    // ══════════════════════════════════════════════════════════════════════════
    // AUTO-AIM TURN PID — Using Pedro Pathing constants
    // ══════════════════════════════════════════════════════════════════════════

    public static double HEADING_P = 1.5;  // From Pedro Pathing Constants
    public static double HEADING_I = 0.0;
    public static double HEADING_D = 0.06;  // From Pedro Pathing Constants
    public static double HEADING_F = 0.0;

    public static double MAX_TURN_POWER = 0.6;
    public static double MIN_TURN_POWER = 0.08;

    public static double INTEGRAL_ACTIVE_THRESHOLD_DEG = 45.0;
    public static double INTEGRAL_MAX_ACCUMULATION      = 0.2;
    public static double HEADING_F_SCALE                = 0.01;

    /** Bearing error (degrees) at or below which telemetry shows "LOCKED" */
    public static double AIM_LOCK_TOLERANCE_DEG = 3.0;

    // ══════════════════════════════════════════════════════════════════════════
    // ROBOT PHYSICAL PROPERTIES  (kept for reference / future use)
    // ══════════════════════════════════════════════════════════════════════════

    public static double ROBOT_MASS_KG            = 10.7955;
    public static double FORWARD_ZERO_POWER_ACCEL = -14.438;
    public static double LATERAL_ZERO_POWER_ACCEL = -66.447;

    // ══════════════════════════════════════════════════════════════════════════
    // BROWNOUT PROTECTION
    // ══════════════════════════════════════════════════════════════════════════

    public static double VOLTAGE_BROWNOUT_THRESHOLD  = 8.5;
    public static double VOLTAGE_RECOVERY_THRESHOLD  = 10.0;
    public static double BROWNOUT_DRIVE_SCALE        = 0.8;
    public static double BROWNOUT_INTAKE_SCALE       = 0.33;
    public static int    VOLTAGE_SAMPLE_COUNT        = 5;

    // ══════════════════════════════════════════════════════════════════════════
    // FLYWHEEL CONFIGURATION
    // ══════════════════════════════════════════════════════════════════════════

    public static double FLYWHEEL_DEFAULT_TARGET = 532.5;
    public static double FLYWHEEL_MIN            = 300.0;
    public static double FLYWHEEL_MAX            = 2000.0;
    public static double FLYWHEEL_STEP           = 5.0;
    public static double SHOOT_BOOST             = 40.0;

    public static double PIDF_P = 200.0;
    public static double PIDF_I = 0.0;
    public static double PIDF_D = 0.0;
    public static double PIDF_F = 25.2;

    public static double RECOVERY_P         = 38.0;
    public static double RECOVERY_THRESHOLD  = 40.0;
    public static double RECOVERY_EXIT       = 15.0;

    // ══════════════════════════════════════════════════════════════════════════
    // DRIVE CONFIGURATION
    // ══════════════════════════════════════════════════════════════════════════

    public static double SLOW_MODE_MULTIPLIER = 0.4;
    public static double DRIVE_DEADZONE       = 0.1;

    // ══════════════════════════════════════════════════════════════════════════
    // CAMERA MOUNT CONFIGURATION
    // ══════════════════════════════════════════════════════════════════════════

    public static double CAMERA_X_OFFSET_INCHES = 5.5;
    public static double CAMERA_Y_OFFSET_INCHES = 0.0;
    public static double CAMERA_Z_OFFSET_INCHES = 10.5;
    public static double CAMERA_PITCH_DEG       = 0.28;
    public static double CAMERA_YAW_DEG         = 0.0;
    public static double CAMERA_ROLL_DEG        = 0.0;
    public static String WEBCAM_NAME            = "Webcam 1";

    // ══════════════════════════════════════════════════════════════════════════
    // SERVO & INTAKE CONFIGURATION
    // ══════════════════════════════════════════════════════════════════════════

    public static double  SERVO_IDLE          = 0.3;
    public static double  SERVO_SHOOT         = 0.85;
    public static double  INTAKE_POWER        = 1.0;
    public static boolean DEPOSIT1_REVERSED   = false;
    public static boolean DEPOSIT2_REVERSED   = false;
}

