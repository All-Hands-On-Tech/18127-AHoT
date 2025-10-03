package org.firstinspires.ftc.teamcode.common;

public class RobotConfig {
    // Drive motor names
    public String frontLeftName = "frontLeft";
    public String frontRightName = "frontRight";
    public String backLeftName = "backLeft";
    public String backRightName = "backRight";


    // Metadata only: logical identifiers for pods stored in Pinpoint firmware/config docs
    public String forwardPodLogicalName = "forwardPod"; // forward (X) pod label
    public String strafePodLogicalName = "strafePod";   // strafe (Y) pod label

    // Device names
    public String intakeName = "intake";
    public String pinpointName = "odo"; // Pinpoint device name
    public String imuName = "imu"; // re-added for RobotHardware
    public String depositMotorLName = "DepositMotorL";
    public String depositMotorRName = "DepositMotorR";
    public String camServoName = "Cam";
    public String transferServoName = "transferServo";

    // Legacy odometry geometry (kept for compatibility)
    public double ticksPerRev = 8192;
    public double wheelDiameterMM = 35.0;
    public double odoPerpendicularOffsetMM = 0.0;
    public double odoParallelOffsetMM = 0.0;
}
