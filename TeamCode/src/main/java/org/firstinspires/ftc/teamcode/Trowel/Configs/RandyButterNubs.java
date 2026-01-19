package org.firstinspires.ftc.teamcode.Trowel.Configs;

import com.qualcomm.robotcore.hardware.DcMotor;

public class RandyButterNubs {
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;

    // Stabilization / slew-rate limiting state
    private double lastFL = 0.0, lastFR = 0.0, lastBL = 0.0, lastBR = 0.0;
    private long lastUpdateNanos = System.nanoTime();

    // Tunables: max change in motor power per second (power units/sec)
    // Increase to allow more aggressive changes; decrease to smooth more
    private static final double DEFAULT_SLEW_RATE = 4.0; // power units per second
    private double slewRatePerSec = DEFAULT_SLEW_RATE;

    // Deadzone and small-change threshold
    private static final double OUTPUT_DEADBAND = 0.02; // ignore very small commands
    private static final double APPLY_THRESHOLD = 0.005; // only call setPower if change larger than this

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
        // Ensure motors use RUN_USING_ENCODER and BRAKE behavior for stronger braking and
        // improved stability when controlling power.
        setupMotor(this.frontLeft);
        setupMotor(this.frontRight);
        setupMotor(this.backLeft);
        setupMotor(this.backRight);
    }

    /**
     * Setup individual motor with proper configuration (directions set in TrowelHardware)
     */
    private void setupMotor(DcMotor motor) {
        if (motor != null) {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            // Use RUN_USING_ENCODER so the motor controller actively regulates speed
            // which tends to reduce jitter in power output compared to raw open-loop.
            try {
                motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            } catch (Exception ignored) {
                // Not fatal, keep previous mode if unsupported
            }
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

        // Set motor powers (with smoothing)
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
        if (motor == null) return;

        // Apply deadband to avoid hunting
        if (Math.abs(power) < OUTPUT_DEADBAND) power = 0.0;

        long now = System.nanoTime();
        double dt = Math.max(1e-6, (now - lastUpdateNanos) / 1e9);
        lastUpdateNanos = now;

        double maxDelta = slewRatePerSec * dt;

        // Select proper "last" value by motor identity
        double last = 0.0;
        if (motor == frontLeft) last = lastFL;
        else if (motor == frontRight) last = lastFR;
        else if (motor == backLeft) last = lastBL;
        else if (motor == backRight) last = lastBR;

        // Limit change (slew rate)
        double delta = power - last;
        if (delta > maxDelta) delta = maxDelta;
        if (delta < -maxDelta) delta = -maxDelta;
        double applied = last + delta;

        // Small-change suppression to reduce rapid tiny updates
        if (Math.abs(applied - last) < APPLY_THRESHOLD) applied = last;

        // Write back last value
        if (motor == frontLeft) lastFL = applied;
        else if (motor == frontRight) lastFR = applied;
        else if (motor == backLeft) lastBL = applied;
        else if (motor == backRight) lastBR = applied;

        // Finally apply to hardware
        motor.setPower(applied);
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
