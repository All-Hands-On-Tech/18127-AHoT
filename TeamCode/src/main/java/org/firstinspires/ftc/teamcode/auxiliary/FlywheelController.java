package org.firstinspires.ftc.teamcode.auxiliary;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.config.RobotConstants;

/**
 * Thin wrapper around the native DcMotorEx velocity controller.
 *
 * This intentionally does not contain any custom PID math. The built-in velocity
 * loop runs on the motor controller itself, which is the normal FTC way to drive
 * a flywheel. If the motor needs more tuning, tune the controller coefficients or
 * the target speed; don't simulate a second custom controller in Java.
 */
public class FlywheelController {
    private final DcMotorEx shooterMotor;
    private double target = RobotConstants.FLYWHEEL_TICKS_PER_SECOND;
    private double outputPower = 0.0;

    public FlywheelController(DcMotorEx shooterMotor, DcMotorEx ignored) {
        this.shooterMotor = shooterMotor;

        if (this.shooterMotor != null) {
            this.shooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            this.shooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            applyPidf();
            this.shooterMotor.setVelocity(0.0);
        }
    }

    public void setTarget(double targetTicksPerSecond) {
        this.target = targetTicksPerSecond;
    }

    public double getTarget() {
        return target;
    }

    public void update() {
        if (shooterMotor == null) {
            return;
        }

        applyPidf();
        shooterMotor.setVelocity(target);
        outputPower = shooterMotor.getPower();
    }

    public void stop() {
        if (shooterMotor != null) {
            shooterMotor.setVelocity(0.0);
            shooterMotor.setPower(0.0);
        }
        outputPower = 0.0;
    }

    public double getVelocity() {
        return shooterMotor != null ? shooterMotor.getVelocity() : 0.0;
    }

    public boolean isReady() {
        if (shooterMotor == null) {
            return false;
        }
        return Math.abs(target - shooterMotor.getVelocity()) <= RobotConstants.FLYWHEEL_READY_BAND * target;
    }

    public double getOutputPower() {
        return outputPower;
    }

    private void applyPidf() {
        if (shooterMotor == null) {
            return;
        }

        shooterMotor.setPIDFCoefficients(
                DcMotor.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(
                        RobotConstants.FLYWHEEL_P,
                        RobotConstants.FLYWHEEL_I,
                        RobotConstants.FLYWHEEL_D,
                        RobotConstants.FLYWHEEL_F));
    }
}
