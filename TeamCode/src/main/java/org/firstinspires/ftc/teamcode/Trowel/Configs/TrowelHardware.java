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
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import android.util.Size;
import java.util.List;
import java.util.ArrayList;
import org.firstinspires.ftc.robotcore.external.navigation.Position;

/**
 * TrowelHardware - Robot hardware abstraction for Trowel
 *
 * This class handles initialization of all robot hardware components including:
 * - 4-wheel mecanum drive motors
 * - Intake motors (intake1 and intake2)
 * - Deposit motors (deposit1 and deposit2)
 * - Transfer servo
 * - IMU and Pinpoint odometry
 */
public class TrowelHardware {
    // Drive motors
    public DcMotor frontLeft, frontRight, backLeft, backRight;

    // Intake motors
    public DcMotor intake1, intake2;

    // Deposit motors with encoder support
    public DcMotorEx deposit1, deposit2;

    // Transfer servo
    public Servo transferServo;

    // Sensors
    public IMU imu;
    public GoBildaPinpointDriver pinpoint;

    // IMU availability flag
    private boolean imuAvailable = false;

    // Configuration
    private final TrowelConfig config;
    private HardwareMap hwMap;

    // Pinpoint initialization tracking
    private boolean pinpointInitialized = false;
    public int pinpointBadReadCount = 0;
    public boolean pinpointBusDowngraded = false;
    public String pinpointRecoveryAction = "";

    public static final PIDFCoefficients DEPOSIT_PIDF = new PIDFCoefficients(2.0, 0.1, 0.5, 20.0);
    // Flag to control whether we apply custom PIDF from DepositPIDFConfig
    public boolean useCustomDepositPIDF = true;

    // Additional software feedforward multiplier (0.0 = no extra FF). This is separate from the
    // REV PIDF F constant and allows increasing the commanded velocity to drive the motor harder
    // during spin-up without changing the PIDF coefficients.
    private double depositFeedforwardFactor = 0.0;
    // Additional absolute feedforward boost in ticks/sec. This adds a fixed number of ticks/sec
    // to the commanded velocity to help overcome static friction / increase spin-up speed.
    private double depositFeedforwardBoostTicks = 0.0;

    // Minimum effective velocity to command to avoid low-speed hunting/oscillation (ticks/sec)
    // Lower this to allow motors to reach lower speeds. Set to 0.0 to disable clamping.
    public static double MIN_EFFECTIVE_DEPOSIT_VELOCITY = 0.0;

    // Vision / camera (AprilTag) objects
    private AprilTagProcessor aprilTagProcessor = null;
    private VisionPortal visionPortal = null;
    private boolean visionEnabled = false;
    private String visionInitError = null;
    private String visionWebcamName = null;
    private Size visionResolution = new Size(640, 480);

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

            // Transfer servo
            try {
                transferServo = hardwareMap.get(Servo.class, config.transfer1Name);
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
                backLeft.setDirection(DcMotor.Direction.FORWARD);
                backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }
            if (backRight != null) {
                backRight.setDirection(DcMotor.Direction.REVERSE);
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
                if (useCustomDepositPIDF) {
                    applyCustomDepositPIDF(
                        DepositPIDFConfig.depositKp,
                        DepositPIDFConfig.depositKi,
                        DepositPIDFConfig.depositKd,
                        DepositPIDFConfig.depositKf
                    );
                }
            }
            if (deposit2 != null) {
                deposit2.setDirection(DcMotor.Direction.REVERSE);
                deposit2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                deposit2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                if (useCustomDepositPIDF) {
                    applyCustomDepositPIDF(
                        DepositPIDFConfig.depositKp,
                        DepositPIDFConfig.depositKi,
                        DepositPIDFConfig.depositKd,
                        DepositPIDFConfig.depositKf
                    );
                }
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
                    imuAvailable = true;
                } else {
                    imuAvailable = false;
                }
            } catch (Exception ignored) {
                imuAvailable = false;
            }

            // Configure transfer servo - don't set position during init to prevent movement
            if (transferServo != null) {
                transferServo.scaleRange(0.0, 1.0);
                // Don't set position here - wait until start
            }

        } catch (Exception e) {
            System.err.println("TrowelHardware init error: " + e.getMessage());
        }
    }

    /**
     * Initialize transfer servo to idle position
     */
    public void initTransferServo() {
        if (transferServo != null) transferServo.setPosition(0.5);
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
        if (Double.isNaN(velocity)) return;

        // Apply software feedforward multiplier (keeps PIDF coefficients unchanged)
        double ffContribution = computeDepositFeedforwardContribution(velocity);
        double commandedVelocity = velocity + ffContribution;

        // If the requested (raw) velocity is essentially zero, stop the motors
        if (Math.abs(velocity) < 1e-6) {
            if (deposit1 != null) deposit1.setPower(0.0);
            if (deposit2 != null) deposit2.setPower(0.0);
            return;
        }

        // Enforce a minimum magnitude to avoid low-speed hunting/oscillation while preserving sign
        if (Math.abs(commandedVelocity) < MIN_EFFECTIVE_DEPOSIT_VELOCITY) {
            commandedVelocity = Math.signum(commandedVelocity) * MIN_EFFECTIVE_DEPOSIT_VELOCITY;
        }

        // Set both motors to the same velocity (no negation)
        if (deposit1 != null) {
            deposit1.setVelocity(commandedVelocity);
        }
        if (deposit2 != null) {
            deposit2.setVelocity(commandedVelocity * 1.2);
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
     * Get deposit motor 1 position in encoder ticks
     */
    public double getDeposit1Position() {
        return deposit1 != null ? deposit1.getCurrentPosition() : 0;
    }

    /**
     * Get deposit motor 2 position in encoder ticks
     */
    public double getDeposit2Position() {
        return deposit2 != null ? deposit2.getCurrentPosition() : 0;
    }

    /**
     * Get deposit motor 1 velocity in ticks per second
     */
    public double getDeposit1Velocity() {
        return deposit1 != null ? deposit1.getVelocity() : 0;
    }

    /**
     * Get deposit motor 2 velocity in ticks per second
     */
    public double getDeposit2Velocity() {
        return deposit2 != null ? deposit2.getVelocity() : 0;
    }

    /**
     * Get deposit motor 1 RPM
     * Assumes 28 ticks per revolution (standard for REV motors)
     */
    public double getDeposit1RPM() {
        double velocity = getDeposit1Velocity();
        return (velocity / 28.0) * 60.0;
    }

    /**
     * Get deposit motor 2 RPM
     * Assumes 28 ticks per revolution (standard for REV motors)
     */
    public double getDeposit2RPM() {
        double velocity = getDeposit2Velocity();
        return (velocity / 28.0) * 60.0;
    }

    /**
     * Get average deposit RPM from both motors
     */
    public double getAverageDepositRPM() {
        return (getDeposit1RPM() + getDeposit2RPM()) / 2.0;
    }

    /**
     * Apply custom PIDF coefficients to deposit motors
     */
    public void applyCustomDepositPIDF(double kp, double ki, double kd, double kf) {
        if (deposit1 != null && deposit2 != null) {
            PIDFCoefficients customPIDF = new PIDFCoefficients(kp, ki, kd, kf);
            deposit1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, customPIDF);
            deposit2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, customPIDF);
        }
    }

    /**
     * Update deposit PIDF values from configuration (if enabled)
     */
    public void updateDepositPIDFFromConfig() {
        if (useCustomDepositPIDF) {
            applyCustomDepositPIDF(
                DepositPIDFConfig.depositKp,
                DepositPIDFConfig.depositKi,
                DepositPIDFConfig.depositKd,
                DepositPIDFConfig.depositKf
            );
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

        status.append("=== TRANSFER SERVO ===\n");
        status.append("TransferServo: ").append(transferServo != null ? "OK" : "MISSING").append("\n");

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

        config.append("=== TRANSFER SERVO POSITION ===\n");
        if (transferServo != null) config.append("TransferServo Pos: ").append(String.format("%.2f", transferServo.getPosition())).append("\n");

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

    /**
     * Check if IMU is available
     */
    public boolean isImuAvailable() {
        return imuAvailable && imu != null;
    }

    /**
     * Get the IMU yaw (heading) in degrees, or Double.NaN if unavailable
     */
    public double getImuYawDegrees() {
        if (!isImuAvailable()) return Double.NaN;
        try {
            // IMU.getRobotYawPitchRollAngles() returns angles; getYaw with DEGREES returns degrees directly
            YawPitchRollAngles ypr = imu.getRobotYawPitchRollAngles();
            return ypr.getYaw(AngleUnit.DEGREES);
        } catch (Exception e) {
            return Double.NaN;
        }
    }

    /**
     * Set the software feedforward multiplier applied to deposit velocity commands.
     * @param factor fractional additive factor (e.g. 0.1 adds 10% to the commanded velocity)
     */
    public void setDepositFeedforwardFactor(double factor) {
        this.depositFeedforwardFactor = factor;
    }

    /**
     * Set an absolute feedforward boost (ticks/sec) to add to deposit velocity commands.
     * This is additive on top of the multiplicative feedforward factor.
     */
    public void setDepositFeedforwardBoostTicks(double boostTicks) {
        this.depositFeedforwardBoostTicks = boostTicks;
    }

    /**
     * Get the absolute feedforward boost (ticks/sec).
     */
    public double getDepositFeedforwardBoostTicks() {
        return this.depositFeedforwardBoostTicks;
    }

    /**
     * Get the current software feedforward factor applied to deposit commands.
     */
    public double getDepositFeedforwardFactor() {
        return this.depositFeedforwardFactor;
    }

    /**
     * Compute the feedforward contribution (in ticks/sec) that will be added for a given target
     * velocity. Useful for telemetry (how much extra velocity is being requested due to FF).
     */
    public double computeDepositFeedforwardContribution(double targetVelocity) {
        if (Double.isNaN(targetVelocity)) return 0.0;
        // Contribution = multiplicative fraction + absolute boost
        double mult = targetVelocity * depositFeedforwardFactor;
        double abs = depositFeedforwardBoostTicks;
        return mult + abs;
    }

    /**
     * Initialize the camera and AprilTag processor. Non-throwing: returns false
     * on failure and sets getVisionInitError() for diagnostics.
     */
    public boolean initVision(String webcamName, double camXInches, double camYInches, double camZInches,
                              double yawDeg, double pitchDeg, double rollDeg) {
        // If already initialized with same webcam, treat as success
        if (visionEnabled && visionWebcamName != null && visionWebcamName.equals(webcamName)) return true;

        try {
            aprilTagProcessor = new AprilTagProcessor.Builder()
                    .setDrawAxes(true)
                    .setDrawCubeProjection(true)
                    .setDrawTagOutline(true)
                    .setCameraPose(
                            new Position(DistanceUnit.INCH, camXInches, camYInches, camZInches, 0),
                            new YawPitchRollAngles(AngleUnit.DEGREES, yawDeg, pitchDeg, rollDeg, 0)
                    )
                    .build();

            visionPortal = new VisionPortal.Builder()
                    .setCamera(hwMap.get(WebcamName.class, webcamName))
                    .addProcessor(aprilTagProcessor)
                    .setCameraResolution(visionResolution)
                    .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                    .enableLiveView(true)
                    .build();

            visionEnabled = true;
            visionInitError = null;
            visionWebcamName = webcamName;
            return true;
        } catch (Exception e) {
            visionInitError = e.getMessage();
            visionEnabled = false;
            aprilTagProcessor = null;
            visionPortal = null;
            return false;
        }
    }

    /** Return the current AprilTag detections list (safe to call). */
    public List<AprilTagDetection> getDetections() {
        if (aprilTagProcessor == null) return new ArrayList<>();
        try {
            List<AprilTagDetection> d = aprilTagProcessor.getDetections();
            return d != null ? d : new ArrayList<>();
        } catch (Exception e) {
            return new ArrayList<>();
        }
    }

    public boolean isVisionEnabled() {
        return visionEnabled;
    }

    public String getVisionInitError() {
        return visionInitError;
    }

    /**
     * Close and release camera/vision resources. Safe to call multiple times.
     */
    public void closeVision() {
        try {
            if (visionPortal != null) {
                visionPortal.close();
            }
        } catch (Exception ignored) {}
        visionPortal = null;
        aprilTagProcessor = null;
        visionEnabled = false;
        visionWebcamName = null;
    }

    /**
     * Single-line status for telemetry describing vision state.
     */
    public String getVisionStatusString() {
        if (visionEnabled) {
            int count = getDetections().size();
            return String.format("ENABLED %s %dx%d (%d tags)",
                    visionWebcamName != null ? visionWebcamName : "(unknown)",
                    visionResolution.getWidth(), visionResolution.getHeight(), count);
        } else {
            return visionInitError != null ? "DISABLED: " + visionInitError : "DISABLED";
        }
    }

}
