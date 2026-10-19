package org.firstinspires.ftc.teamcode.drive;

import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.auxiliary.Auxiliary;
import org.firstinspires.ftc.teamcode.config.RobotConstants;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;

public class Drivetrain {
    private final RobotHardware hardware;
    private final Mecanum drive;
    private final Auxiliary auxiliary;
    private final OdometryAimAssist aimAssist;

    private Alliance alliance = Alliance.BLUE;

    public Drivetrain(HardwareMap hardwareMap) {
        this.hardware = new RobotHardware(hardwareMap);
        this.drive = RobotConstants.createDrive(hardwareMap);
        this.auxiliary = new Auxiliary(hardware);
        this.aimAssist = new OdometryAimAssist(hardwareMap);
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

    public void update(Gamepad gamepad, boolean aimHeld) {
        updateDrive(gamepad, aimHeld);
        updateAuxiliaries(gamepad);
    }

    public void updateDrive(Gamepad gamepad, boolean aimHeld) {
        double forward = -gamepad.left_stick_y * RobotConstants.DRIVE_SPEED_SCALE;
        double strafe = -gamepad.left_stick_x * RobotConstants.STRAFE_SPEED_SCALE;
        double turn = -gamepad.right_stick_x * RobotConstants.TURN_SPEED_SCALE;

        if (aimHeld) {
            turn = aimAssist.getTurnCorrection(alliance);
        }

        drive.drive(new DrivePowers(forward, strafe, turn), false);
    }

    public void updateAuxiliaries(Gamepad gamepad) {
        if (gamepad.right_trigger > 0.05) {
            auxiliary.setIntakePower(gamepad.right_trigger * Constants.INTAKE_POWER);
        } else if (gamepad.left_trigger > 0.05) {
            auxiliary.setIntakePower(-gamepad.left_trigger * Constants.INTAKE_POWER);
        } else {
            auxiliary.stopIntake();
        }

        if (gamepad.right_bumper) {
            auxiliary.runFlywheels();
        } else {
            auxiliary.stopFlywheels();
        }
    }

    public void stop() {
        drive.stop();
        auxiliary.stopAll();
    }
}
