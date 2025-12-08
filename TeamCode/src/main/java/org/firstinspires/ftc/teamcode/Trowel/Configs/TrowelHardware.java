package org.firstinspires.ftc.teamcode.Trowel.Configs;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.GoBildaPinpointDriver.DeviceStatus;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * TrowelHardware - Robot hardware abstraction for Trowel
 *
 * This class handles initialization of all robot hardware components including:
 * - 4-wheel mecanum drive motors
 * - Intake motors (intake1 and intake2)
 * - Deposit motors (deposit1 and deposit2)
 * - Transfer servos
 * - IMU and Pinpoint odometry
 */
public class TrowelHardware {
    // Drive motors
    public DcMotor frontLeft, frontRight, backLeft, backRight;

    // Intake motors
    public DcMotor intake1, intake2;

    // Deposit motors with encoder support
    public DcMotorEx deposit1, deposit2;

    // Transfer servos
    public Servo transfer1, transfer2;

    // Sensors
    public IMU imu;
    public GoBildaPinpointDriver pinpoint;

    // Configuration
    private final TrowelConfig config;
    private HardwareMap hwMap;

    // Pinpoint initialization tracking
    private boolean pinpointInitialized = false;
    public int pinpointBadReadCount = 0;
    public boolean pinpointBusDowngraded = false;
    public String pinpointRecoveryAction = "";

    // Custom PIDF coefficients for deposit motors - tuned for stable velocity control
    public static final PIDFCoefficients DEPOSIT_PIDF = new PIDFCoefficients(80.0, 0.4, 6.0, 13.5);

    /**
     * Constructor - Initialize hardware with HardwareMap
     *
     * @param hardwareMap The robot's hardware map from OpMode
     */
    public TrowelHardware(HardwareMap hardwareMap) {
        config = new TrowelConfig();
        hwMap = hardwareMap;
        initializeHardware(hardwareMap);
    }

    /**
     * Initialize all hardware components
     */
    private void initializeHardware(HardwareMap hardwareMap) {
        try {
            // Drive motors
            frontLeft = hardwareMap.get(DcMotor.class, config.frontLeftName);
            frontRight = hardwareMap.get(DcMotor.class, config.frontRightName);
            backLeft = hardwareMap.get(DcMotor.class, config.backLeftName);
            backRight = hardwareMap.get(DcMotor.class, config.backRightName);

            // Intake motors
            try {
                intake1 = hardwareMap.get(DcMotor.class, config.intake1Name);
            } catch (Exception ignored) {
            }
            try {
                intake2 = hardwareMap.get(DcMotor.class, config.intake2Name);
            } catch (Exception ignored) {
            }

            // Transfer servos
            try {
                transfer1 = hardwareMap.get(Servo.class, config.transfer1Name);
            } catch (Exception ignored) {
            }
            try {
                transfer2 = hardwareMap.get(Servo.class, config.transfer2Name);
            } catch (Exception ignored) {
            }

            // Configure drive motors
            if (frontLeft != null) {
                frontLeft.setDirection(DcMotor.Direction.FORWARD);
                frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }
            if (frontRight != null) {
                frontRight.setDirection(DcMotor.Direction.FORWARD);
                frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }
            if (backLeft != null) {
                backLeft.setDirection(DcMotor.Direction.REVERSE);
                backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }
            if (backRight != null) {
                backRight.setDirection(DcMotor.Direction.FORWARD);
                backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }

            // Intake motors
            if (intake1 != null) {
                intake1.setDirection(DcMotor.Direction.FORWARD);
                intake1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                intake1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }
            if (intake2 != null) {
                intake2.setDirection(DcMotor.Direction.FORWARD);
                intake2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                intake2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }

            // Deposit motors
            try {
                deposit1 = hardwareMap.get(DcMotorEx.class, config.deposit1Name);
            } catch (Exception ignored) {
            }
            try {
                deposit2 = hardwareMap.get(DcMotorEx.class, config.deposit2Name);
            } catch (Exception ignored) {
            }

            if (deposit1 != null) {
                deposit1.setDirection(DcMotor.Direction.FORWARD);
                deposit1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                deposit1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                deposit1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, DEPOSIT_PIDF);
            }
            if (deposit2 != null) {
                deposit2.setDirection(DcMotor.Direction.REVERSE);
                deposit2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                deposit2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                deposit2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, DEPOSIT_PIDF);
            }

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
            } catch (Exception ignored) {
            }

            // Configure transfer servos - set to starting positions
            // transfer2 is reversed (1.0 to 0.0) so both servos move together
            if (transfer1 != null) {
                transfer1.scaleRange(0.0, 1.0);
                transfer1.setPosition(0.5);
            }
            if (transfer2 != null) {
                transfer2.setDirection(Servo.Direction.REVERSE);
                transfer2.scaleRange(0.0, 1.0);
                transfer2.setPosition(0.5);
            }

        } catch (Exception e) {
            System.err.println("TrowelHardware init error: " + e.getMessage());
        }
    }

    /**
     * Initialize transfer servo to idle position
     */
    public void initTransferServos() {
        if (transfer1 != null) transfer1.setPosition(0.5);
        if (transfer2 != null) transfer2.setPosition(0.5);
    }

    /**
     * Initialize Pinpoint odometry system
     */
    public void initPinpoint() {
        if (pinpointInitialized) return;
        try {
            pinpoint = hwMap.get(GoBildaPinpointDriver.class, config.pinpointName);
        } catch (Exception e) {
            pinpoint = null;
            pinpointInitialized = true;
            return;
        }

        try {
            pinpoint.initialize();
            int attempts = 0;
            while (attempts < 50 && pinpoint.getDeviceStatus() != DeviceStatus.READY) {
                try {
                    Thread.sleep(40);
                } catch (InterruptedException ignored) {
                }
                pinpoint.update();
                attempts++;
            }

            if (pinpoint != null && pinpoint.getDeviceStatus() == DeviceStatus.READY) {
                pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
                pinpoint.setEncoderDirections(config.pinpointForwardEncoderDirection, config.pinpointStrafeEncoderDirection);
                pinpoint.setOffsets(config.odoPerpendicularOffsetMM, config.odoParallelOffsetMM, DistanceUnit.MM);
                pinpoint.resetPosAndIMU();
            }
        } catch (Exception e) {
            System.err.println("Pinpoint init error: " + e.getMessage());
            pinpoint = null;
        }

        pinpointInitialized = true;
    }

    /**
     * Update Pinpoint position reading
     */
    public void updatePinpoint() {
        if (pinpoint == null) return;
        try {
            pinpoint.update();
        } catch (Exception e) {
            System.err.println("Pinpoint update error: " + e.getMessage());
        }
    }

    /**
     * Reset all motors to stop
     */
    public void stop() {
        if (frontLeft != null) frontLeft.setPower(0);
        if (frontRight != null) frontRight.setPower(0);
        if (backLeft != null) backLeft.setPower(0);
        if (backRight != null) backRight.setPower(0);
        if (intake1 != null) intake1.setPower(0);
        if (intake2 != null) intake2.setPower(0);
        if (deposit1 != null) deposit1.setPower(0);
        if (deposit2 != null) deposit2.setPower(0);
    }

    /**
     * Run deposit motors at specified velocity (ticks per second)
     * Uses setVelocity with RUN_USING_ENCODER mode for smooth control
     */
    public void setDepositVelocity(double velocity) {
        if (deposit1 != null) {
            deposit1.setVelocity(velocity);
        }
        if (deposit2 != null) {
            deposit2.setVelocity(velocity);
        }
    }

    /**
     * Stop deposit motors
     */
    public void stopDeposit() {
        if (deposit1 != null) deposit1.setPower(0.0);
        if (deposit2 != null) deposit2.setPower(0.0);
    }

    /**
     * Reset deposit motor encoders and set to RUN_USING_ENCODER mode
     */
    public void resetDepositEncoders() {
        if (deposit1 != null) {
            deposit1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            deposit1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        if (deposit2 != null) {
            deposit2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            deposit2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }


    /**
     * Get hardware initialization status as a formatted string for telemetry
     */
    public String getInitializationStatus() {
        StringBuilder status = new StringBuilder();
        status.append("=== DRIVE MOTORS ===\n");
        status.append("FL: ").append(frontLeft != null ? "OK" : "MISSING").append("\n");
        status.append("FR: ").append(frontRight != null ? "OK" : "MISSING").append("\n");
        status.append("BL: ").append(backLeft != null ? "OK" : "MISSING").append("\n");
        status.append("BR: ").append(backRight != null ? "OK" : "MISSING").append("\n");

        status.append("=== INTAKE MOTORS ===\n");
        status.append("Intake1: ").append(intake1 != null ? "OK" : "MISSING").append("\n");
        status.append("Intake2: ").append(intake2 != null ? "OK" : "MISSING").append("\n");

        status.append("=== DEPOSIT MOTORS ===\n");
        status.append("Deposit1: ").append(deposit1 != null ? "OK" : "MISSING").append("\n");
        status.append("Deposit2: ").append(deposit2 != null ? "OK" : "MISSING").append("\n");

        status.append("=== TRANSFER SERVOS ===\n");
        status.append("Transfer1: ").append(transfer1 != null ? "OK" : "MISSING").append("\n");
        status.append("Transfer2: ").append(transfer2 != null ? "OK" : "MISSING").append("\n");

        status.append("=== SENSORS ===\n");
        status.append("IMU: ").append(imu != null ? "OK" : "MISSING").append("\n");
        status.append("Pinpoint: ").append(pinpoint != null ? "OK" : "MISSING").append("\n");

        return status.toString();
    }

    /**
     * Get motor directions and configurations as a formatted string
     */
    public String getMotorConfigurations() {
        StringBuilder config = new StringBuilder();

        config.append("=== DRIVE MOTOR DIRECTIONS ===\n");
        if (frontLeft != null) config.append("FL Direction: ").append(frontLeft.getDirection()).append("\n");
        if (frontRight != null) config.append("FR Direction: ").append(frontRight.getDirection()).append("\n");
        if (backLeft != null) config.append("BL Direction: ").append(backLeft.getDirection()).append("\n");
        if (backRight != null) config.append("BR Direction: ").append(backRight.getDirection()).append("\n");

        config.append("=== DEPOSIT MOTOR MODES ===\n");
        if (deposit1 != null) config.append("Deposit1 Mode: ").append(deposit1.getMode()).append("\n");
        if (deposit2 != null) config.append("Deposit2 Mode: ").append(deposit2.getMode()).append("\n");

        config.append("=== TRANSFER SERVO POSITIONS ===\n");
        if (transfer1 != null) config.append("Transfer1 Pos: ").append(String.format("%.2f", transfer1.getPosition())).append("\n");
        if (transfer2 != null) config.append("Transfer2 Pos: ").append(String.format("%.2f", transfer2.getPosition())).append("\n");

        return config.toString();
    }

    /**
     * Get current motor powers for telemetry
     */
    public String getMotorPowers() {
        StringBuilder powers = new StringBuilder();

        powers.append("=== DRIVE MOTOR POWERS ===\n");
        if (frontLeft != null) powers.append("FL Power: ").append(String.format("%.2f", frontLeft.getPower())).append("\n");
        if (frontRight != null) powers.append("FR Power: ").append(String.format("%.2f", frontRight.getPower())).append("\n");
        if (backLeft != null) powers.append("BL Power: ").append(String.format("%.2f", backLeft.getPower())).append("\n");
        if (backRight != null) powers.append("BR Power: ").append(String.format("%.2f", backRight.getPower())).append("\n");

        powers.append("=== INTAKE MOTOR POWERS ===\n");
        if (intake1 != null) powers.append("Intake1 Power: ").append(String.format("%.2f", intake1.getPower())).append("\n");
        if (intake2 != null) powers.append("Intake2 Power: ").append(String.format("%.2f", intake2.getPower())).append("\n");

        powers.append("=== DEPOSIT MOTOR POWERS ===\n");
        if (deposit1 != null) powers.append("Deposit1 Power: ").append(String.format("%.2f", deposit1.getPower())).append("\n");
        if (deposit2 != null) powers.append("Deposit2 Power: ").append(String.format("%.2f", deposit2.getPower())).append("\n");

        return powers.toString();
    }
}

