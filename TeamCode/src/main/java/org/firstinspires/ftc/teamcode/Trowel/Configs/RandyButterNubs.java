package org.firstinspires.ftc.teamcode.Trowel.Configs;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

public class RandyButterNubs {
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;

    // PIDF coefficients for deposit motors - tuned for stable velocity control
    public static final PIDFCoefficients DEPOSIT_PIDF = new PIDFCoefficients(80.0, 0.4, 6.0, 12.0);

    // Default deposit velocity in ticks per second
    public static final double DEFAULT_DEPOSIT_VELOCITY = 1800.0;

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
        // Negate rotate to fix turning direction
        rotate = -rotate;

        // Mecanum drive equations
        // Each wheel's power is a combination of forward, strafe, and rotate
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

        // Full speed - no power scaling
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
}

