package org.firstinspires.ftc.teamcode.auxiliary;

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

    public void stopIntake() {
        if (hardware.intakeMotor != null) {
            hardware.intakeMotor.setPower(0.0);
        }
    }

    public void stopAll() {
        stopIntake();
    }
}
