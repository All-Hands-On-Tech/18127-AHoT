package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.auxiliary.Auxiliary;
import org.firstinspires.ftc.teamcode.config.RobotConstants;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;

public class Drivetrain {
    private final RobotHardware hardware;
    private final Auxiliary auxiliary;
    private final OdometryAimAssist aimAssist;

    private Alliance alliance = Alliance.BLUE;

    public Drivetrain(HardwareMap hardwareMap) {
        this.hardware = new RobotHardware(hardwareMap);
        this.auxiliary = new Auxiliary(hardware);
        this.aimAssist = new OdometryAimAssist(hardwareMap);
    }

    public boolean hasDrive() {
        return hardware.hasDrive;
    }

    public boolean hasAimAssist() {
        return aimAssist != null && aimAssist.isEnabled();
    }

    public void setAlliance(Alliance alliance) {
        this.alliance = alliance;
    }

    public Alliance getAlliance() {
        return alliance;
    }

    public OdometryAimAssist getAimAssist() {
        return aimAssist;
    }


    public void update(Gamepad gamepad, Gamepad gamepad2, boolean aimHeld) {
        if (aimAssist != null) {
            aimAssist.updatePose();
        }
        if (hasDrive()) {
            updateDrive(gamepad, aimHeld);
        }
        updateAuxiliaries(gamepad, gamepad2);
    }

    public void updateDrive(Gamepad gamepad, boolean aimHeld) {
        if (!hasDrive()) {
            return;
        }

        double forward = -gamepad.left_stick_y * RobotConstants.DRIVE_SPEED_SCALE;
        double strafe = -gamepad.left_stick_x * RobotConstants.STRAFE_SPEED_SCALE;
        double turn = -gamepad.right_stick_x * RobotConstants.TURN_SPEED_SCALE;

        if (aimHeld && aimAssist != null) {
            /*
             * Chassis-only aiming: the robot turns its whole base toward the target point using
             * odometry heading error, rather than rotating a separate turret mechanism.
             */
            turn = aimAssist.getTurnCorrection(alliance);
        }

        if (Math.abs(forward) < 0.01 && Math.abs(strafe) < 0.01 && Math.abs(turn) < 0.01) {
            hardware.stopDrive();
            return;
        }

        /*
         * Standard mecanum mix. Drive motor directions are set in RobotHardware so that
         * positive power means "push the robot forward" on every wheel:
         * positive forward = drive forward, positive strafe = strafe right,
         * positive turn = rotate clockwise.
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
