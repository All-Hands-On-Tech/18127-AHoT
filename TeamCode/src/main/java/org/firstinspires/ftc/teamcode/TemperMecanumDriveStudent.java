package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "Mecanum Drive Temper", group = "TeleOp")
public class TemperMecanumDriveStudent extends LinearOpMode {
    private DcMotor leftBackDrive;
    private DcMotor rightBackDrive;
    private DcMotor iMotor;
    private DcMotor leftFrontDrive;
    private DcMotor rightFrontDrive;
    private CRServo intakeLeft;
    private CRServo intakeRight;

    @Override
    public void runOpMode() {

        // Hardware setup
        leftBackDrive = hardwareMap.get(DcMotor.class, "leftBackDrive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "rightBackDrive");
        iMotor = hardwareMap.get(DcMotor.class, "iMotor");
        leftFrontDrive = hardwareMap.get(DcMotor.class, "leftFrontDrive");

        rightFrontDrive = hardwareMap.get(DcMotor.class, "rightFrontDrive");

        intakeLeft = hardwareMap.get(CRServo.class, "intakeLeft");
        intakeRight = hardwareMap.get(CRServo.class, "intakeRight");

        leftBackDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        leftFrontDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        iMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intakeLeft.setPower(0);
        intakeRight.setPower(0);

        waitForStart();

        while(opModeIsActive()) {
            double y = gamepad1.left_stick_y;
            double rx = -gamepad1.right_stick_x;
            double x = -gamepad1.left_stick_x;


            leftFrontDrive.setPower(y+x+rx);
            rightFrontDrive.setPower(y-x-rx);
            leftBackDrive.setPower(y-x+rx);
            rightBackDrive.setPower(y+x-rx);

            if(gamepad1.right_bumper) {
                iMotor.setPower(1);
                intakeLeft.setPower(-1);
                intakeRight.setPower(1);
            } else if (gamepad1.left_bumper) {
                iMotor.setPower(-1);
                intakeLeft.setPower(1);
                intakeRight.setPower(-1);
            } else {
                    iMotor.setPower(0);
                    intakeLeft.setPower(0);
                    intakeRight.setPower(0);
                }

            // Telemetry
            telemetry.addData("Left Front Power", leftFrontDrive.getPower());
            telemetry.addData("Right Front Power", rightFrontDrive.getPower());
            telemetry.addData("Left Back Power", leftBackDrive.getPower());
            telemetry.addData("Right Back Power", rightBackDrive.getPower());

            telemetry.addData("Gamepad1 y value", gamepad1.left_stick_y);
            telemetry.addData("Gamepad1 x value", gamepad1.left_stick_x);
            telemetry.addData("Gamepad1 rx value", gamepad1.right_stick_x);
            telemetry.update();
        }
    }
}