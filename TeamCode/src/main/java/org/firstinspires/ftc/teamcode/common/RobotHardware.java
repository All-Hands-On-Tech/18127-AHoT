package org.firstinspires.ftc.teamcode.common;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.teamcode.ZSupport.Claw;

public class RobotHardware {
    // Drive motors
    public DcMotor frontLeft, frontRight, backLeft, backRight;

    // Odometry wheels
    public DcMotor odoParallel, odoPerpendicular;

    // IMU
    public IMU imu;

    // Claw system
    public Claw claw;

    // Configuration
    private RobotConfig config;

    public RobotHardware(HardwareMap hardwareMap) {
        config = new RobotConfig();
        initializeHardware(hardwareMap);
    }

    private void initializeHardware(HardwareMap hardwareMap) {
        try {
            // Initialize drive motors
            frontLeft = hardwareMap.get(DcMotor.class, config.frontLeftName);
            frontRight = hardwareMap.get(DcMotor.class, config.frontRightName);
            backLeft = hardwareMap.get(DcMotor.class, config.backLeftName);
            backRight = hardwareMap.get(DcMotor.class, config.backRightName);

            // Set motor directions (adjust based on your robot)
            frontLeft.setDirection(DcMotor.Direction.FORWARD);
            frontRight.setDirection(DcMotor.Direction.REVERSE);
            backLeft.setDirection(DcMotor.Direction.FORWARD);
            backRight.setDirection(DcMotor.Direction.REVERSE);

            // Set zero power behavior
            frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            // Initialize odometry wheels
            try {
                odoParallel = hardwareMap.get(DcMotor.class, config.odoParallelName);
                odoPerpendicular = hardwareMap.get(DcMotor.class, config.odoPerpendicularName);
            } catch (Exception e) {
                System.out.println("Warning: Odometry wheels not found in hardware map");
            }

            // Initialize IMU
            try {
                imu = hardwareMap.get(IMU.class, config.imuName);
            } catch (Exception e) {
                System.out.println("Warning: IMU not found in hardware map");
            }

            // Initialize claw system
            try {
                DcMotor clawLowerLeft = hardwareMap.get(DcMotor.class, config.clawLowerLeftName);
                DcMotor clawLowerRight = hardwareMap.get(DcMotor.class, config.clawLowerRightName);
                DcMotor clawGripLeft = hardwareMap.get(DcMotor.class, config.clawGripLeftName);
                DcMotor clawGripRight = hardwareMap.get(DcMotor.class, config.clawGripRightName);

                claw = new Claw(clawLowerLeft, clawLowerRight, clawGripLeft, clawGripRight);
            } catch (Exception e) {
                System.out.println("Warning: Claw motors not found in hardware map");
            }

            System.out.println("RobotHardware: Initialization complete");

        } catch (Exception e) {
            System.err.println("RobotHardware: Error during initialization - " + e.getMessage());
        }
    }

    public RobotConfig getConfig() {
        return config;
    }
}
