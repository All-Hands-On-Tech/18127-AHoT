package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.config.RobotConstants;

/** Mechanisms only. The drivetrain belongs to Pedro (see Constants.java). */
public class RobotHardware {
    public final DcMotorEx intakeMotor;
    public final DcMotorEx shooterMotor;
    private final CRServo flowerLeft;
    private final CRServo flowerRight;
    private final Servo blockerServo;
    private final Servo axonBlockerServo;

    public RobotHardware(HardwareMap map) {
        intakeMotor = map.tryGet(DcMotorEx.class, RobotConstants.INTAKE_MOTOR);
        shooterMotor = map.tryGet(DcMotorEx.class, RobotConstants.SHOOTER_MOTOR);
        flowerLeft = map.tryGet(CRServo.class, RobotConstants.CONTINUOUS_SERVO_LEFT);
        flowerRight = map.tryGet(CRServo.class, RobotConstants.CONTINUOUS_SERVO_RIGHT);
        blockerServo = map.tryGet(Servo.class, RobotConstants.BLOCKER_SERVO);
        axonBlockerServo = map.tryGet(Servo.class, RobotConstants.AXON_BLOCKER_SERVO);

        if (intakeMotor != null) {
            intakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
        if (shooterMotor != null) {
            shooterMotor.setDirection(DcMotor.Direction.FORWARD);
        }
        if (flowerLeft != null) flowerLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        if (flowerRight != null) flowerRight.setDirection(DcMotorSimple.Direction.FORWARD);
        setFlowerPower(0.0);
        setBlockerClosed(false);
    }

    public void setIntakePower(double power) {
        if (intakeMotor != null) intakeMotor.setPower(power);
    }

    public void setFlowerPower(double power) {
        if (flowerLeft != null) flowerLeft.setPower(power);
        if (flowerRight != null) flowerRight.setPower(power);
    }

    public void setBlockerClosed(boolean closed) {
        double position = closed ? 0.9 : 0.5;
        if (blockerServo != null) blockerServo.setPosition(position);
        if (axonBlockerServo != null) axonBlockerServo.setPosition(position);
    }

    public void stopMechanisms() {
        setIntakePower(0.0);
        setFlowerPower(0.0);
        if (shooterMotor != null) shooterMotor.setPower(0.0);
    }
}