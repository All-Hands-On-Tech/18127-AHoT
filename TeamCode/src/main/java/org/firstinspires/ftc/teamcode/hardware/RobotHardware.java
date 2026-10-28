package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.config.RobotConstants;

public class RobotHardware {
    public final DcMotorEx frontLeftDrive;
    public final DcMotorEx backLeftDrive;
    public final DcMotorEx frontRightDrive;
    public final DcMotorEx backRightDrive;
    public final DcMotorEx intakeMotor;
    public final DcMotorEx leftFlywheel;
    public final DcMotorEx rightFlywheel;
    public final CRServo continuousServoLeft;
    public final CRServo continuousServoRight;
    public final Servo flowerLeftServo;
    public final Servo flowerRightServo;
    public final Servo blockerServo;
    public final Servo axonBlockerServo;

    public final boolean hasDrive;
    public final boolean hasIntake;
    public final boolean hasFlywheels;
    public final boolean hasContinuousServos;

    public RobotHardware(HardwareMap hardwareMap) {
        frontLeftDrive = hardwareMap != null ? hardwareMap.tryGet(DcMotorEx.class, RobotConstants.FRONT_LEFT_DRIVE) : null;
        backLeftDrive = hardwareMap != null ? hardwareMap.tryGet(DcMotorEx.class, RobotConstants.BACK_LEFT_DRIVE) : null;
        frontRightDrive = hardwareMap != null ? hardwareMap.tryGet(DcMotorEx.class, RobotConstants.FRONT_RIGHT_DRIVE) : null;
        backRightDrive = hardwareMap != null ? hardwareMap.tryGet(DcMotorEx.class, RobotConstants.BACK_RIGHT_DRIVE) : null;

        intakeMotor = hardwareMap != null ? hardwareMap.tryGet(DcMotorEx.class, RobotConstants.INTAKE_MOTOR) : null;
        leftFlywheel = hardwareMap != null ? hardwareMap.tryGet(DcMotorEx.class, RobotConstants.LEFT_FLYWHEEL_MOTOR) : null;
        rightFlywheel = hardwareMap != null ? hardwareMap.tryGet(DcMotorEx.class, RobotConstants.RIGHT_FLYWHEEL_MOTOR) : null;
        continuousServoLeft = hardwareMap != null ? hardwareMap.tryGet(CRServo.class, RobotConstants.CONTINUOUS_SERVO_LEFT) : null;
        continuousServoRight = hardwareMap != null ? hardwareMap.tryGet(CRServo.class, RobotConstants.CONTINUOUS_SERVO_RIGHT) : null;
        flowerLeftServo = hardwareMap != null ? hardwareMap.tryGet(Servo.class, RobotConstants.FLOWER_LEFT_SERVO) : null;
        flowerRightServo = hardwareMap != null ? hardwareMap.tryGet(Servo.class, RobotConstants.FLOWER_RIGHT_SERVO) : null;
        blockerServo = hardwareMap != null ? hardwareMap.tryGet(Servo.class, RobotConstants.BLOCKER_SERVO) : null;
        axonBlockerServo = hardwareMap != null ? hardwareMap.tryGet(Servo.class, RobotConstants.AXON_BLOCKER_SERVO) : null;

        hasDrive = frontLeftDrive != null && backLeftDrive != null && frontRightDrive != null && backRightDrive != null;
        hasIntake = intakeMotor != null;
        hasFlywheels = leftFlywheel != null && rightFlywheel != null;
        hasContinuousServos = continuousServoLeft != null && continuousServoRight != null;

        configureMechanisms();
    }

    private void configureMechanisms() {
        configureDriveMotors();

        if (intakeMotor != null) {
            intakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        if (leftFlywheel != null) {
            leftFlywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftFlywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            leftFlywheel.setDirection(DcMotor.Direction.FORWARD);
        }

        if (rightFlywheel != null) {
            rightFlywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightFlywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rightFlywheel.setDirection(DcMotor.Direction.REVERSE);
        }

        if (continuousServoLeft != null) {
            continuousServoLeft.setPower(0.0);
        }
        if (continuousServoRight != null) {
            continuousServoRight.setPower(0.0);
        }
        if (flowerLeftServo != null) {
            flowerLeftServo.setPosition(RobotConstants.FLOWER_DEFAULT_POSITION);
        }
        if (flowerRightServo != null) {
            flowerRightServo.setPosition(RobotConstants.FLOWER_DEFAULT_POSITION);
        }
        if (blockerServo != null) {
            blockerServo.setDirection(Servo.Direction.FORWARD);
            blockerServo.setPosition(0.5);
        }
        if (axonBlockerServo != null) {
            axonBlockerServo.setDirection(Servo.Direction.FORWARD);
            axonBlockerServo.setPosition(0.5);
        }
    }

    /**
     * Directions are set so that positive power means "push the robot forward" on
     * every wheel, which the standard mecanum mix in Drivetrain relies on.
     */
    private void configureDriveMotors() {
        if (frontLeftDrive != null) {
            frontLeftDrive.setDirection(DcMotorSimple.Direction.FORWARD);
            frontLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
        if (backLeftDrive != null) {
            backLeftDrive.setDirection(DcMotorSimple.Direction.FORWARD);
            backLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
        if (frontRightDrive != null) {
            frontRightDrive.setDirection(DcMotorSimple.Direction.REVERSE);
            frontRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
        if (backRightDrive != null) {
            backRightDrive.setDirection(DcMotorSimple.Direction.REVERSE);
            backRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
    }

    public void setDrivePowers(double frontLeft, double backLeft, double frontRight, double backRight) {
        if (frontLeftDrive != null) {
            frontLeftDrive.setPower(frontLeft);
        }
        if (backLeftDrive != null) {
            backLeftDrive.setPower(backLeft);
        }
        if (frontRightDrive != null) {
            frontRightDrive.setPower(frontRight);
        }
        if (backRightDrive != null) {
            backRightDrive.setPower(backRight);
        }
    }

    public void stopDrive() {
        setDrivePowers(0.0, 0.0, 0.0, 0.0);
    }

    public void setBlockerClosed(boolean closed) {
        if (blockerServo != null) {
            blockerServo.setPosition(closed ? 0.9 : 0.5);
        }
        if (axonBlockerServo != null) {
            axonBlockerServo.setPosition(closed ? 0.9 : 0.5);
        }
    }

    public void setFlowerPositions(double leftPosition, double rightPosition) {
        double clampedLeft = Math.max(RobotConstants.FLOWER_MIN_POSITION,
                Math.min(RobotConstants.FLOWER_MAX_POSITION, leftPosition));
        double clampedRight = Math.max(RobotConstants.FLOWER_MIN_POSITION,
                Math.min(RobotConstants.FLOWER_MAX_POSITION, rightPosition));

        if (flowerLeftServo != null) {
            flowerLeftServo.setPosition(clampedLeft);
        }
        if (flowerRightServo != null) {
            flowerRightServo.setPosition(clampedRight);
        }
    }

    public void stopMechanisms() {
        if (intakeMotor != null) {
            intakeMotor.setPower(0.0);
        }
        if (leftFlywheel != null) {
            leftFlywheel.setPower(0.0);
        }
        if (rightFlywheel != null) {
            rightFlywheel.setPower(0.0);
        }
        if (continuousServoLeft != null) {
            continuousServoLeft.setPower(0.0);
        }
        if (continuousServoRight != null) {
            continuousServoRight.setPower(0.0);
        }
        if (flowerLeftServo != null) {
            flowerLeftServo.setPosition(RobotConstants.FLOWER_DEFAULT_POSITION);
        }
        if (flowerRightServo != null) {
            flowerRightServo.setPosition(RobotConstants.FLOWER_DEFAULT_POSITION);
        }
    }
}
