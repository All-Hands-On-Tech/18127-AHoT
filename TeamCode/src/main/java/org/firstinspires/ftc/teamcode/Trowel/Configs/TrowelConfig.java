package org.firstinspires.ftc.teamcode.Trowel.Configs;

import org.firstinspires.ftc.teamcode.GoBildaPinpointDriver;

public class TrowelConfig {
    // Drive motor names
    public String frontLeftName = "frontLeft";
    public String frontRightName = "frontRight";
    public String backLeftName = "backLeft";
    public String backRightName = "backRight";

    // Intake motors
    public String intake1Name = "intake1";
    public String intake2Name = "intake2";

    // Deposit motors
    public String deposit1Name = "deposit1";
    public String deposit2Name = "deposit2";

    // Transfer servos
    public String transfer1Name = "transfer1";

    // Device names
    // Standardize on a single IMU name. If your robot config uses a different name change this value.
    public String imuName = "imu";
    public String pinpointName = "odo";

    // Standardized encoder ticks per revolution (default requested: 750)
    public int encoderTicksPerRev = 750;

    // Pinpoint odometry pod offsets in millimeters
    // X offset is for the forward/backward encoder (parallel to forward motion)
    // Y offset is for the left/right encoder (perpendicular to forward motion)
    public double odoPerpendicularOffsetMM = -127.05;
    public double odoParallelOffsetMM = -165.95;

    // Pinpoint encoder directions
    public GoBildaPinpointDriver.EncoderDirection pinpointForwardEncoderDirection = GoBildaPinpointDriver.EncoderDirection.REVERSED;
    public GoBildaPinpointDriver.EncoderDirection pinpointStrafeEncoderDirection = GoBildaPinpointDriver.EncoderDirection.REVERSED;
}
