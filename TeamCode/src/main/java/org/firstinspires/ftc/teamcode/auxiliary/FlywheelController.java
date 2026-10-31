package org.firstinspires.ftc.teamcode.auxiliary;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.config.RobotConstants;

/**
 * Custom flywheel velocity controller.
 * <p>
 * Replaces the SDK built-in velocity PIDF (setVelocityPIDF/setVelocity) with a
 * custom control law whose gains are expressed in motor power (at nominal
 * voltage) per tick/s, driven straight into setPower() on both flywheel motors.
 * <p>
 * Call update() once per OpMode loop (including while shooting) and use
 * isReady() to decide whether the shooter is at speed. All gains and parameters
 * are re-read from RobotConstants on every update() so FTC Dashboard edits take
 * effect immediately.
 */
public class FlywheelController {
    private final DcMotorEx leftFlywheel;
    private final DcMotorEx rightFlywheel;
    private final VoltageSensor battery;

    private final ElapsedTime loopTimer = new ElapsedTime();
    private final ElapsedTime batteryTimer = new ElapsedTime();

    private double target = 0.0;
    private double filteredVelocity = 0.0;
    private double lastFilteredVelocity = 0.0;
    private boolean haveLastVelocity = false;
    private double integral = 0.0;
    private double outputPower = 0.0;
    private double batteryVoltage = 0.0;

    public FlywheelController(DcMotorEx leftFlywheel, DcMotorEx rightFlywheel, VoltageSensor battery) {
        this.leftFlywheel = leftFlywheel;
        this.rightFlywheel = rightFlywheel;
        this.battery = battery;

        // Velocity reading still works in RUN_WITHOUT_ENCODER; FLOAT lets the
        // wheel coast so the controller fully commands the power (including 0).
        if (leftFlywheel != null) {
            leftFlywheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            leftFlywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }
        if (rightFlywheel != null) {
            rightFlywheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rightFlywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }
    }

    public void setTarget(double targetTicksPerSecond) {
        target = targetTicksPerSecond;
    }

    public double getTarget() {
        return target;
    }

    /**
     * Runs one controller step. Read the velocities, evaluate the control law,
     * and drive both motors with the computed power. Call once per loop.
     */
    public void update() {
        if (leftFlywheel == null && rightFlywheel == null) {
            return;
        }

        // Read all gains/parameters live so FTC Dashboard edits apply instantly.
        double ks = RobotConstants.FLYWHEEL_KS;
        double kv = RobotConstants.FLYWHEEL_KV;
        double kp = RobotConstants.FLYWHEEL_KP;
        double ki = RobotConstants.FLYWHEEL_KI;
        double kd = RobotConstants.FLYWHEEL_KD;
        double boostFraction = RobotConstants.FLYWHEEL_BOOST_FRACTION;
        double integralLimit = RobotConstants.FLYWHEEL_INTEGRAL_LIMIT;
        double nominalVoltage = RobotConstants.FLYWHEEL_NOMINAL_VOLTAGE;

        double rawVelocity;
        if (leftFlywheel != null && rightFlywheel != null) {
            rawVelocity = 0.5 * (leftFlywheel.getVelocity() + rightFlywheel.getVelocity());
        } else if (leftFlywheel != null) {
            rawVelocity = leftFlywheel.getVelocity();
        } else {
            rawVelocity = rightFlywheel.getVelocity();
        }
        filteredVelocity += 0.6 * (rawVelocity - filteredVelocity);

        double dt = clamp(loopTimer.seconds(), 0.001, 0.1);
        loopTimer.reset();

        double dVelocity = 0.0;
        if (haveLastVelocity && dt > 0.0) {
            dVelocity = (filteredVelocity - lastFilteredVelocity) / dt;
        }
        lastFilteredVelocity = filteredVelocity;
        haveLastVelocity = true;

        double u;
        if (target <= 0.0) {
            u = 0.0;
            integral = 0.0;
        } else {
            double err = target - filteredVelocity;
            if (err > boostFraction * target) {
                // Spin-up / shot-recovery boost: full power, no windup.
                u = 1.0;
                integral = 0.0;
            } else if (err < -boostFraction * target) {
                // Overspeed: coast.
                u = 0.0;
                integral = 0.0;
            } else {
                integral += err * dt;
                if (ki > 1e-9) {
                    integral = clamp(integral, -integralLimit / ki, integralLimit / ki);
                }
                u = ks + kv * target + kp * err + ki * integral;
                if (kd != 0.0) {
                    u -= kd * dVelocity;
                }
            }
        }

        // Battery sag compensation: scale the tuned-at-nominal-voltage power up
        // when the battery is low. Re-read about every 100 ms, ignore garbage.
        if (batteryTimer.seconds() >= 0.1) {
            batteryTimer.reset();
            if (battery != null) {
                double voltage = battery.getVoltage();
                if (voltage > 5.0) {
                    batteryVoltage = voltage;
                }
            }
        }
        double supplyVoltage = batteryVoltage > 5.0 ? batteryVoltage : nominalVoltage;

        outputPower = clamp(u * nominalVoltage / supplyVoltage, 0.0, 1.0);
        if (leftFlywheel != null) {
            leftFlywheel.setPower(outputPower);
        }
        if (rightFlywheel != null) {
            rightFlywheel.setPower(outputPower);
        }
    }

    /** Commands zero power to both flywheel motors and resets the controller state. */
    public void stop() {
        if (leftFlywheel != null) {
            leftFlywheel.setPower(0.0);
        }
        if (rightFlywheel != null) {
            rightFlywheel.setPower(0.0);
        }
        integral = 0.0;
        outputPower = 0.0;
        loopTimer.reset();
        haveLastVelocity = false;
    }

    /** Low-pass filtered average velocity in ticks/s. */
    public double getVelocity() {
        return filteredVelocity;
    }

    /** True when the filtered velocity is within FLYWHEEL_READY_BAND of the target. */
    public boolean isReady() {
        if (leftFlywheel == null && rightFlywheel == null) {
            return false;
        }
        return Math.abs(target - filteredVelocity) < RobotConstants.FLYWHEEL_READY_BAND * target;
    }

    /** Last motor power applied to both flywheel motors (before clamping sign, 0..1). */
    public double getOutputPower() {
        return outputPower;
    }

    private static double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }
}
