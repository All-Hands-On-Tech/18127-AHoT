package org.firstinspires.ftc.teamcode.Trowel.Configs;

import com.qualcomm.robotcore.hardware.DcMotor;

public class RandyButterNubs {
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;

    // Deposit PIDF is centralized in TrowelHardware.DEPOSIT_PIDF

    // Default deposit velocity in ticks per second
    public static final double DEFAULT_DEPOSIT_VELOCITY = 221.0;

    /**
     * Constructor for RandyButterNubs (Mecanum Drive)
     *
     * @param frontLeft Front-left motor
     * @param frontRight Front-right motor
     * @param backLeft Back-left motor
     * @param backRight Back-right motor
     */
    public RandyButterNubs(DcMotor frontLeft, DcMotor frontRight, DcMotor backLeft, DcMotor backRight) {
        this.frontLeft = frontLeft;
        this.frontRight = frontRight;
        this.backLeft = backLeft;
        this.backRight = backRight;

        // Motors are already configured in TrowelHardware - don't override directions here
    }

    /**
     * Setup individual motor with proper configuration (directions set in TrowelHardware)
     */
    private void setupMotor(DcMotor motor) {
        if (motor != null) {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }

    /**
     * Drive the robot with forward, strafe, and rotate commands
     *
     * @param forward Forward/backward movement (-1.0 to 1.0)
     * @param strafe Left/right strafing (-1.0 to 1.0)
     * @param rotate Rotation (-1.0 to 1.0)
     */
    public void drive(double forward, double strafe, double rotate) {
        // Standard mecanum drive equations
        // Forward: all wheels same direction
        // Strafe: diagonal pairs opposite (FL+BR vs FR+BL)
        // Rotate: left side vs right side opposite
        double frontLeftPower = forward + strafe + rotate;
        double frontRightPower = forward - strafe - rotate;
        double backLeftPower = forward - strafe + rotate;
        double backRightPower = forward + strafe - rotate;

        // Normalize power values to be within [-1, 1]
        double maxPower = getMaxAbsValue(frontLeftPower, frontRightPower, backLeftPower, backRightPower);
        if (maxPower > 1.0) {
            frontLeftPower /= maxPower;
            frontRightPower /= maxPower;
            backLeftPower /= maxPower;
            backRightPower /= maxPower;
        }

        // Set motor powers
        setMotorPower(frontLeft, frontLeftPower);
        setMotorPower(frontRight, frontRightPower);
        setMotorPower(backLeft, backLeftPower);
        setMotorPower(backRight, backRightPower);
    }

    /**
     * Drive with optional slow-mode modifiers based on bumpers.
     * If rightBumper is held, scale = 0.3 (higher priority). If leftBumper is held, scale = 0.7.
     */
    public void drive(double forward, double strafe, double rotate, boolean leftBumper, boolean rightBumper) {
        double scale = 1.0;
        if (rightBumper) scale = 0.3; // RB = 30% speed
        else if (leftBumper) scale = 0.7; // LB = 70% speed

        // Apply scaling to inputs
        forward *= scale;
        strafe *= scale;
        rotate *= scale;

        // Standard mecanum drive equations
        double frontLeftPower = forward + strafe + rotate;
        double frontRightPower = forward - strafe - rotate;
        double backLeftPower = forward - strafe + rotate;
        double backRightPower = forward + strafe - rotate;

        // Normalize power values to be within [-1, 1]
        double maxPower = getMaxAbsValue(frontLeftPower, frontRightPower, backLeftPower, backRightPower);
        if (maxPower > 1.0) {
            frontLeftPower /= maxPower;
            frontRightPower /= maxPower;
            backLeftPower /= maxPower;
            backRightPower /= maxPower;
        }

        // Set motor powers
        setMotorPower(frontLeft, frontLeftPower);
        setMotorPower(frontRight, frontRightPower);
        setMotorPower(backLeft, backLeftPower);
        setMotorPower(backRight, backRightPower);
    }

    /**
     * Drive forward/backward only
     *
     * @param power Power (-1.0 to 1.0)
     */
    public void driveForward(double power) {
        drive(power, 0, 0);
    }

    /**
     * Strafe left/right only
     *
     * @param power Power (-1.0 to 1.0, positive = strafe right)
     */
    public void strafe(double power) {
        drive(0, power, 0);
    }

    /**
     * Rotate clockwise/counterclockwise only
     *
     * @param power Power (-1.0 to 1.0, positive = clockwise)
     */
    public void rotate(double power) {
        drive(0, 0, power);
    }

    /**
     * Stop all motors
     */
    public void stop() {
        drive(0, 0, 0);
    }

    /**
     * Set individual motor power
     */
    private void setMotorPower(DcMotor motor, double power) {
        if (motor != null) {
            motor.setPower(power);
        }
    }

    /**
     * Get the maximum absolute value from multiple numbers
     */
    private double getMaxAbsValue(double... values) {
        double max = 0;
        for (double value : values) {
            max = Math.max(max, Math.abs(value));
        }
        return max;
    }

    /**
     * Get individual motor power
     */
    public double getFrontLeftPower() {
        return frontLeft != null ? frontLeft.getPower() : 0;
    }

    public double getFrontRightPower() {
        return frontRight != null ? frontRight.getPower() : 0;
    }

    public double getBackLeftPower() {
        return backLeft != null ? backLeft.getPower() : 0;
    }

    public double getBackRightPower() {
        return backRight != null ? backRight.getPower() : 0;
    }

    /**
     * Reinitialize all drive motors and add telemetry for debugging
     */
    public void reinitializeMotors() {
        setupMotor(frontLeft);
        setupMotor(frontRight);
        setupMotor(backLeft);
        setupMotor(backRight);

        // Add telemetry for debugging
        System.out.println("Front Left Motor Power: " + getFrontLeftPower());
        System.out.println("Front Right Motor Power: " + getFrontRightPower());
        System.out.println("Back Left Motor Power: " + getBackLeftPower());
        System.out.println("Back Right Motor Power: " + getBackRightPower());
    }

    /**
     * Set the brake mode for all motors
     *
     * @param brakeMode True to enable BRAKE mode, false for FLOAT mode
     */
    public void setBrakeMode(boolean brakeMode) {
        DcMotor.ZeroPowerBehavior behavior = brakeMode ? DcMotor.ZeroPowerBehavior.BRAKE : DcMotor.ZeroPowerBehavior.FLOAT;
        if (frontLeft != null) frontLeft.setZeroPowerBehavior(behavior);
        if (frontRight != null) frontRight.setZeroPowerBehavior(behavior);
        if (backLeft != null) backLeft.setZeroPowerBehavior(behavior);
        if (backRight != null) backRight.setZeroPowerBehavior(behavior);
    }
}
