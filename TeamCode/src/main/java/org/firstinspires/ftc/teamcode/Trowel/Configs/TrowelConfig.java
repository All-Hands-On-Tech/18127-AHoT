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
    public String transfer2Name = "transfer2";

    // Device names
    public String pinpointName = "odo";
    public String imuName = "imu";

    // Pinpoint odometry pod offsets in millimeters
    // These are the physical distances from the center of rotation to each encoder wheel
    // X offset is for the forward/backward encoder (parallel to forward motion)
    // Y offset is for the left/right encoder (perpendicular to forward motion)
    public double odoPerpendicularOffsetMM = 19.05;  // 0.75 inches = 19.05 mm (forward pod Y offset)
    public double odoParallelOffsetMM = 107.95;      // 4.25 inches = 107.95 mm (strafe pod X offset)

    // Pinpoint encoder directions
    public GoBildaPinpointDriver.EncoderDirection pinpointForwardEncoderDirection = GoBildaPinpointDriver.EncoderDirection.FORWARD;
    public GoBildaPinpointDriver.EncoderDirection pinpointStrafeEncoderDirection = GoBildaPinpointDriver.EncoderDirection.FORWARD;
}

