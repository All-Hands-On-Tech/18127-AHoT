package org.firstinspires.ftc.teamcode.auxiliary;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.config.RobotConstants;

/** Thin wrapper around the motor controller's built-in velocity loop. */
public class FlywheelController {
    private final DcMotorEx motor;
    private double target = RobotConstants.FLYWHEEL_TICKS_PER_SECOND;

    public FlywheelController(DcMotorEx motor) {
        this.motor = motor;
        if (motor != null) {
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }
    }

    public void setTarget(double ticksPerSecond) {
        target = ticksPerSecond;
    }

    /** Re-applies the Dashboard PIDF every call so live edits take effect. */
    public void update() {
        if (motor == null) return;
        motor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(RobotConstants.FLYWHEEL_P, RobotConstants.FLYWHEEL_I,
                        RobotConstants.FLYWHEEL_D, RobotConstants.FLYWHEEL_F));
        motor.setVelocity(target);
    }

    public void stop() {
        if (motor != null) {
            motor.setVelocity(0.0);
            motor.setPower(0.0);
        }
    }

    public double getVelocity() {
        return motor != null ? motor.getVelocity() : 0.0;
    }

    public double getOutputPower() {
        return motor != null ? motor.getPower() : 0.0;
    }
}