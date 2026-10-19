package org.firstinspires.ftc.teamcode.auxiliary;

import org.firstinspires.ftc.teamcode.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.config.RobotConstants;

public class Auxiliary {
    private final RobotHardware hardware;

    public Auxiliary(RobotHardware hardware) {
        this.hardware = hardware;
    }

    public void setIntakePower(double power) {
        hardware.intakeMotor.setPower(power);
    }

    public void setFlywheelTicksPerSecond(double ticksPerSecond) {
        hardware.leftFlywheel.setVelocity(ticksPerSecond);
        hardware.rightFlywheel.setVelocity(ticksPerSecond);
    }

    public void runFlywheels() {
        setFlywheelTicksPerSecond(RobotConstants.FLYWHEEL_TICKS_PER_SECOND);
    }

    public void stopIntake() {
        hardware.intakeMotor.setPower(0.0);
    }

    public void stopFlywheels() {
        hardware.leftFlywheel.setPower(0.0);
        hardware.rightFlywheel.setPower(0.0);
    }

    public void stopAll() {
        stopIntake();
        stopFlywheels();
    }
}
