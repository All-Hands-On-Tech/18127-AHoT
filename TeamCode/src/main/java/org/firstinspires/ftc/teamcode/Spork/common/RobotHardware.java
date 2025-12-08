package org.firstinspires.ftc.teamcode.common;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import org.firstinspires.ftc.teamcode.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.GoBildaPinpointDriver.DeviceStatus;
import org.firstinspires.ftc.teamcode.GoBildaPinpointDriver.EncoderDirection;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class RobotHardware {
    // Drive motors
    public DcMotor frontLeft, frontRight, backLeft, backRight, intake1, intake2;

    // IMU (hub) fallback
    public IMU imu;

    // Pinpoint
    public GoBildaPinpointDriver pinpoint;
    public int pinpointBadReadCount = 0;
    public boolean pinpointBusDowngraded = false;
    public String pinpointRecoveryAction = "";
    private boolean pinpointInitialized = false;

    // Deposit motors - changed to DcMotor for velocity control
    public DcMotorEx depositMotorL, depositMotorR;
    // Transfer motors
    public DcMotorEx transferMotorL, transferMotorR;
    // Servos
    public com.qualcomm.robotcore.hardware.Servo cam;
    public com.qualcomm.robotcore.hardware.Servo transferServo; // Changed from CRServo to Servo

    private final RobotConfig config;
    private HardwareMap hwMap;

    // Custom PIDF coefficients for deposit motors - tuned for stable velocity control
    public static final PIDFCoefficients DEPOSIT_PIDF = new PIDFCoefficients(80.0, 0.4, 6.0, 12.0);

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
            try { intake1 = hardwareMap.get(DcMotor.class, config.intake1Name); } catch (Exception ignored) {}
            try { intake2 = hardwareMap.get(DcMotor.class, config.intake2Name); } catch (Exception ignored) {}

            if (frontLeft != null) frontLeft.setDirection(DcMotor.Direction.FORWARD);
            if (frontRight != null) frontRight.setDirection(DcMotor.Direction.REVERSE);
            if (backLeft != null) backLeft.setDirection(DcMotor.Direction.FORWARD);
            if (backRight != null) backRight.setDirection(DcMotor.Direction.REVERSE);
            if (intake1 != null) intake1.setDirection(DcMotor.Direction.FORWARD);
            if (intake2 != null) intake2.setDirection(DcMotor.Direction.FORWARD);

            if (frontLeft != null) frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            if (frontRight != null) frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            if (backLeft != null) backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            if (backRight != null) backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            if (intake1 != null) intake1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            if (intake2 != null) intake2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

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

            // Deposit motors
            try { depositMotorL = hardwareMap.get(DcMotorEx.class, config.depositMotorLName); } catch (Exception ignored) {}
            try { depositMotorR = hardwareMap.get(DcMotorEx.class, config.depositMotorRName); } catch (Exception ignored) {}
            if (depositMotorL != null) {
                depositMotorL.setDirection(DcMotor.Direction.REVERSE);
                depositMotorL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                depositMotorL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                depositMotorL.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, DEPOSIT_PIDF);
            }
            if (depositMotorR != null) {
                depositMotorR.setDirection(DcMotor.Direction.FORWARD);
                depositMotorR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                depositMotorR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                depositMotorR.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, DEPOSIT_PIDF);
            }
            // Transfer motors
            try { transferMotorL = hardwareMap.get(DcMotorEx.class, config.transferMotorLName); } catch (Exception ignored) {}
            try { transferMotorR = hardwareMap.get(DcMotorEx.class, config.transferMotorRName); } catch (Exception ignored) {}
            if (transferMotorL != null) {
                transferMotorL.setDirection(DcMotor.Direction.FORWARD);
                transferMotorL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                transferMotorL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
            if (transferMotorR != null) {
                transferMotorR.setDirection(DcMotor.Direction.FORWARD);
                transferMotorR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                transferMotorR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
            // Servos
            try { cam = hardwareMap.get(com.qualcomm.robotcore.hardware.Servo.class, config.camServoName); } catch (Exception ignored) {}
            try { transferServo = hardwareMap.get(com.qualcomm.robotcore.hardware.Servo.class, config.transferServoName); } catch (Exception ignored) {}

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
            pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.REVERSED);
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
        // Apply motor power correction factors from Constants to compensate for speed differences
        if (frontLeft != null) frontLeft.setPower(fl * org.firstinspires.ftc.teamcode.pedroPathing.Constants.LEFT_FRONT_POWER);
        if (frontRight != null) frontRight.setPower(fr * org.firstinspires.ftc.teamcode.pedroPathing.Constants.RIGHT_FRONT_POWER);
        if (backLeft != null) backLeft.setPower(bl * org.firstinspires.ftc.teamcode.pedroPathing.Constants.LEFT_REAR_POWER);
        if (backRight != null) backRight.setPower(br * org.firstinspires.ftc.teamcode.pedroPathing.Constants.RIGHT_REAR_POWER);
    }

    // Intake control
    public void startIntake1() {
        if (intake1 != null) intake1.setPower(1.0);
    }
    public void stopIntake1() {
        if (intake1 != null) intake1.setPower(0.0);
    }
    public void startIntake2() {
        if (intake2 != null) intake2.setPower(1.0);
    }
    public void stopIntake2() {
        if (intake2 != null) intake2.setPower(0.0);
    }

    // Deposit control (simple example: run both motors forward for deposit)
    public void runDeposit() {
        if (depositMotorL != null) depositMotorL.setPower(1.0); // Adjust power/direction as needed
        if (depositMotorR != null) depositMotorR.setPower(1.0);
    }
    public void stopDeposit() {
        if (depositMotorL != null) depositMotorL.setPower(0.0);
        if (depositMotorR != null) depositMotorR.setPower(0.0);
    }

    // Transfer servo control for deposit
    public void runTransferServo() {
        if (transferServo != null) transferServo.setPosition(1.0); // Move to active position
    }
    public void stopTransferServo() {
        if (transferServo != null) transferServo.setPosition(0.5); // Return to neutral
    }

    // Deposit control (runs deposit motors and transfer servo)
    public void runDepositFull() {
        runDeposit();
        runTransferServo();
    }
    public void stopDepositFull() {
        stopDeposit();
        stopTransferServo();
    }
}
