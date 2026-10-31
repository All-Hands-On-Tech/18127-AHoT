package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.auxiliary.Auxiliary;
import org.firstinspires.ftc.teamcode.config.RobotConstants;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;

public class Drivetrain {
    private final RobotHardware hardware;
    private final Auxiliary auxiliary;

    private Alliance alliance = Alliance.BLUE;

    public Drivetrain(HardwareMap hardwareMap) {
        this.hardware = new RobotHardware(hardwareMap);
        this.auxiliary = new Auxiliary(hardware);
    }

    public boolean hasDrive() {
        return hardware.hasDrive;
    }

    public void setAlliance(Alliance alliance) {
        this.alliance = alliance;
    }

    public Alliance getAlliance() {
        return alliance;
    }

    public void update(Gamepad gamepad, Gamepad gamepad2, boolean turnOverrideActive, double turnOverride) {
        if (hasDrive()) {
            updateDrive(gamepad, turnOverrideActive, turnOverride);
        }
        updateAuxiliaries(gamepad, gamepad2);
    }

    public void updateDrive(Gamepad gamepad, boolean turnOverrideActive, double turnOverride) {
        if (!hasDrive()) {
            return;
        }

        double forward = -gamepad.left_stick_y * RobotConstants.DRIVE_SPEED_SCALE;
        double strafe = -gamepad.left_stick_x * RobotConstants.STRAFE_SPEED_SCALE;
        double turn;
        if (turnOverrideActive) {
            /*
             * Auto-aim: the external turn passed in (e.g. the output of the
             * AutoAimController) replaces the driver's turn stick. Positive
             * turn rotates the chassis CCW, matching the Pinpoint CCW-positive
             * heading, so the controller output can be passed straight through.
             */
            turn = turnOverride;
        } else {
            turn = -gamepad.right_stick_x * RobotConstants.TURN_SPEED_SCALE;
        }

        if (Math.abs(forward) < 0.01 && Math.abs(strafe) < 0.01 && Math.abs(turn) < 0.01) {
            hardware.stopDrive();
            return;
        }

        /*
         * Standard mecanum mix. Drive motor directions are set in RobotHardware so that
         * positive power means "push the robot forward" on every wheel:
         * positive forward = drive forward, positive strafe = strafe right,
         * positive turn = rotate CCW (left wheels get +turn, right wheels -turn).
         */
        double frontLeft = forward + strafe + turn;
        double backLeft = forward - strafe + turn;
        double frontRight = forward - strafe - turn;
        double backRight = forward + strafe - turn;

        double max = Math.max(Math.abs(frontLeft), Math.max(Math.abs(backLeft),
                Math.max(Math.abs(frontRight), Math.abs(backRight))));
        if (max > 1.0) {
            frontLeft /= max;
            backLeft /= max;
            frontRight /= max;
            backRight /= max;
        }

        hardware.setDrivePowers(frontLeft, backLeft, frontRight, backRight);
    }

    public void updateAuxiliaries(Gamepad gamepad, Gamepad gamepad2) {
        if (hardware.intakeMotor != null) {
            if (gamepad.right_trigger > 0.05) {
                auxiliary.setIntakePower(gamepad.right_trigger * RobotConstants.INTAKE_POWER);
            } else if (gamepad.left_trigger > 0.05) {
                auxiliary.setIntakePower(-gamepad.left_trigger * RobotConstants.INTAKE_POWER);
            } else {
                auxiliary.stopIntake();
            }
        }
    }

    public void stop() {
        if (hardware.hasDrive) {
            hardware.stopDrive();
        }
        auxiliary.stopAll();
    }
}
