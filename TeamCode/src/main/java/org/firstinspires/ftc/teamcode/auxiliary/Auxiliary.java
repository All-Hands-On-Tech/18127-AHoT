package org.firstinspires.ftc.teamcode.auxiliary;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.config.RobotConstants;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;

public class Auxiliary {
    private final RobotHardware hardware;

    public Auxiliary(RobotHardware hardware) {
        this.hardware = hardware;
    }

    public void setIntakePower(double power) {
        if (hardware.intakeMotor != null) {
            hardware.intakeMotor.setPower(power);
        }
    }

    public void applyFlywheelPIDCoefficients() {
        if (hardware.leftFlywheel == null && hardware.rightFlywheel == null) {
            return;
        }

        PIDFCoefficients coefficients = new PIDFCoefficients(
                RobotConstants.FLYWHEEL_KP,
                RobotConstants.FLYWHEEL_KI,
                RobotConstants.FLYWHEEL_KD,
                RobotConstants.FLYWHEEL_KF);

        if (hardware.leftFlywheel != null) {
            hardware.leftFlywheel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, coefficients);
        }
        if (hardware.rightFlywheel != null) {
            hardware.rightFlywheel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, coefficients);
        }
    }

    public void setFlywheelTicksPerSecond(double ticksPerSecond) {
        applyFlywheelPIDCoefficients();
        if (hardware.leftFlywheel != null) {
            hardware.leftFlywheel.setVelocity(ticksPerSecond);
        }
        if (hardware.rightFlywheel != null) {
            hardware.rightFlywheel.setVelocity(ticksPerSecond);
        }
    }

    public void runFlywheels() {
        if (hardware.leftFlywheel != null || hardware.rightFlywheel != null) {
            setFlywheelTicksPerSecond(RobotConstants.FLYWHEEL_TICKS_PER_SECOND);
        }
    }

    public void stopIntake() {
        if (hardware.intakeMotor != null) {
            hardware.intakeMotor.setPower(0.0);
        }
    }

    public void stopFlywheels() {
        if (hardware.leftFlywheel != null) {
            hardware.leftFlywheel.setPower(0.0);
        }
        if (hardware.rightFlywheel != null) {
            hardware.rightFlywheel.setPower(0.0);
        }
    }

    public void stopAll() {
        stopIntake();
        stopFlywheels();
    }
}
