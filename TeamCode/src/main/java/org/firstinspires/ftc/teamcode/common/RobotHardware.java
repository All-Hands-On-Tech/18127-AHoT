package org.firstinspires.ftc.teamcode.common;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.teamcode.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.GoBildaPinpointDriver.DeviceStatus;
import org.firstinspires.ftc.teamcode.GoBildaPinpointDriver.EncoderDirection;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class RobotHardware {
    // Drive motors
    public DcMotor frontLeft, frontRight, backLeft, backRight, intakeMotor;

    // IMU (hub) fallback
    public IMU imu;

    // Pinpoint
    public GoBildaPinpointDriver pinpoint;
    public int pinpointBadReadCount = 0;
    public boolean pinpointBusDowngraded = false;
    public String pinpointRecoveryAction = "";
    private boolean pinpointInitialized = false;

    private final RobotConfig config;
    private HardwareMap hwMap;

    public RobotHardware(HardwareMap hardwareMap) {
        config = new RobotConfig();
        hwMap = hardwareMap;
        initializeHardware(hardwareMap);
    }

    private void initializeHardware(HardwareMap hardwareMap) {
        try {
            // Drive motors
            frontLeft = hardwareMap.get(DcMotor.class, config.frontLeftName);
            frontRight = hardwareMap.get(DcMotor.class, config.frontRightName);
            backLeft = hardwareMap.get(DcMotor.class, config.backLeftName);
            backRight = hardwareMap.get(DcMotor.class, config.backRightName);
            try { intakeMotor = hardwareMap.get(DcMotor.class, config.intakeName); } catch (Exception ignored) {}

            if (frontLeft != null) frontLeft.setDirection(DcMotor.Direction.FORWARD);
            if (frontRight != null) frontRight.setDirection(DcMotor.Direction.REVERSE);
            if (backLeft != null) backLeft.setDirection(DcMotor.Direction.FORWARD);
            if (backRight != null) backRight.setDirection(DcMotor.Direction.REVERSE);
            if (intakeMotor != null) intakeMotor.setDirection(DcMotor.Direction.FORWARD);

            if (frontLeft != null) frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            if (frontRight != null) frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            if (backLeft != null) backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            if (backRight != null) backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            if (intakeMotor != null) intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

            // IMU
            try {
                imu = hardwareMap.get(IMU.class, config.imuName);
                if (imu != null) {
                    RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(
                            RevHubOrientationOnRobot.LogoFacingDirection.UP,
                            RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
                    );
                    IMU.Parameters params = new IMU.Parameters(orientationOnRobot);
                    imu.initialize(params);
                    imu.resetYaw();
                }
            } catch (Exception ignored) {}

        } catch (Exception e) {
            System.err.println("RobotHardware init error: " + e.getMessage());
        }
    }

    public void initPinpoint() {
        if (pinpointInitialized) return;
        try {
            pinpoint = hwMap.get(GoBildaPinpointDriver.class, config.pinpointName);
        } catch (Exception e) {
            pinpoint = null;
            pinpointInitialized = true;
            return;
        }
        pinpoint.initialize();
        int attempts = 0;
        while (attempts < 50 && pinpoint.getDeviceStatus() != DeviceStatus.READY) {
            try { Thread.sleep(40); } catch (InterruptedException ignored) {}
            pinpoint.update();
            attempts++;
        }
        if (pinpoint != null) {
            pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
            pinpoint.setEncoderDirections(EncoderDirection.FORWARD, EncoderDirection.FORWARD);
            pinpoint.setOffsets(config.odoPerpendicularOffsetMM, config.odoParallelOffsetMM, DistanceUnit.MM);
            pinpoint.resetPosAndIMU();
        }
        pinpointInitialized = true;
    }

    public void updatePinpoint() {
        if (pinpoint == null) return;
        pinpoint.update();
        DeviceStatus st = pinpoint.getDeviceStatus();
        if (st == DeviceStatus.FAULT_BAD_READ) {
            pinpointBadReadCount++;
            if (pinpointBadReadCount == 15) { pinpoint.resetPosAndIMU(); pinpointRecoveryAction = "Soft reset"; }
            else if (pinpointBadReadCount == 30 && !pinpointBusDowngraded) { pinpoint.setStandardBusSpeed(); pinpointBusDowngraded = true; pinpointRecoveryAction = "Bus -> 100k"; }
            else if (pinpointBadReadCount == 60) { pinpoint.setFastBusSpeed(); pinpoint.initialize(); pinpointRecoveryAction = "Re-init bus"; pinpointBadReadCount = 0; pinpointBusDowngraded = false; }
        } else {
            pinpointBadReadCount = 0;
        }
    }

    public void setDrivePowers(double fl, double fr, double bl, double br) {
        if (frontLeft != null) frontLeft.setPower(fl);
        if (frontRight != null) frontRight.setPower(fr);
        if (backLeft != null) backLeft.setPower(bl);
        if (backRight != null) backRight.setPower(br);
    }
}
