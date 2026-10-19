package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.config.RobotConstants;

public class RobotHardware {
    public final DcMotorEx frontLeftDrive;
    public final DcMotorEx backLeftDrive;
    public final DcMotorEx frontRightDrive;
    public final DcMotorEx backRightDrive;
    public final DcMotorEx intakeMotor;
    public final DcMotorEx leftFlywheel;
    public final DcMotorEx rightFlywheel;
    public RobotHardware(HardwareMap hardwareMap) {
        frontLeftDrive = hardwareMap.get(DcMotorEx.class, RobotConstants.FRONT_LEFT_DRIVE);
        backLeftDrive = hardwareMap.get(DcMotorEx.class, RobotConstants.BACK_LEFT_DRIVE);
        frontRightDrive = hardwareMap.get(DcMotorEx.class, RobotConstants.FRONT_RIGHT_DRIVE);
        backRightDrive = hardwareMap.get(DcMotorEx.class, RobotConstants.BACK_RIGHT_DRIVE);

        intakeMotor = hardwareMap.get(DcMotorEx.class, RobotConstants.INTAKE_MOTOR);
        leftFlywheel = hardwareMap.get(DcMotorEx.class, RobotConstants.LEFT_FLYWHEEL_MOTOR);
        rightFlywheel = hardwareMap.get(DcMotorEx.class, RobotConstants.RIGHT_FLYWHEEL_MOTOR);

        configureMechanisms();
    }

    private void configureMechanisms() {
        intakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftFlywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFlywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftFlywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFlywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFlywheel.setDirection(DcMotor.Direction.REVERSE);
        leftFlywheel.setDirection(DcMotor.Direction.FORWARD);

    }

    public void stopMechanisms() {
        intakeMotor.setPower(0.0);
        leftFlywheel.setPower(0.0);
        rightFlywheel.setPower(0.0);
    }

}
