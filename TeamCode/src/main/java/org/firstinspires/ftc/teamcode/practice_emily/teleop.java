package org.firstinspires.ftc.teamcode.practice_emily;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp(name="Emily's teleop",group="main")
public class teleop extends LinearOpMode{
    //note to self make sure to make all objects private (is there a reason?)
    private DcMotor blWheel;
    private DcMotor brWheel;
    private DcMotor tlWheel;
    private DcMotor trWheel;
    private DcMotor intakeMotor;
    private CRServo transferBlocker;
    private CRServo lPollenIntake;
    private CRServo rPollenIntake;
    private DcMotor lOuttakeWheel;
    private DcMotor rOuttakeWheel;

    //(I'm skipping odometry and imu for now for simplicity)

    @Override //what is @override for?
    public void runOpMode(){
        //hardware setup part
        blWheel = hardwareMap.get(DcMotor.class,"leftBackDrive");
        brWheel = hardwareMap.get(DcMotor.class,"rightBackDrive");
        tlWheel = hardwareMap.get(DcMotor.class,"leftFrontDrive");
        trWheel = hardwareMap.get(DcMotor.class,"rightFrontDrive");
        intakeMotor = hardwareMap.get(DcMotor.class,"iMotor");

        transferBlocker = hardwareMap.get(CRServo.class,"transferBlocker");
        lPollenIntake = hardwareMap.get(CRServo.class,"intakeLeft");
        rPollenIntake = hardwareMap.get(CRServo.class,"intakeRight");

        lOuttakeWheel = hardwareMap.get(DcMotor.class,"leftOuttake");
        rOuttakeWheel = hardwareMap.get(DcMotor.class,"rightOuttake");

        //set initial DcMotor stuff-- set direction, zero power behavior
        blWheel.setDirection(DcMotorSimple.Direction.FORWARD);
        brWheel.setDirection(DcMotorSimple.Direction.FORWARD);
        tlWheel.setDirection(DcMotorSimple.Direction.FORWARD);
        tlWheel.setDirection(DcMotorSimple.Direction.FORWARD);
        lOuttakeWheel.setDirection(DcMotorSimple.Direction.FORWARD);
        rOuttakeWheel.setDirection(DcMotorSimple.Direction.FORWARD);

        blWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        brWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        tlWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        trWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lOuttakeWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rOuttakeWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //set initial CRServo stuff-- set power
        transferBlocker.setPower(0);
        lPollenIntake.setPower(0);
        rPollenIntake.setPower(0);

        //update drive hub on status
        telemetry.addLine("Finished intializing and stuff cool");
        telemetry.addLine("Also wait someone is actually running this??");
        telemetry.update();

        waitForStart();
        while(opModeIsActive()){
            //controllers
            double inputX = gamepad1.left_stick_x; //why is gamepad1 being used and not gamepad 2? What's the difference in the code (is it driver 1)
            double inputY = gamepad1.left_stick_y;
            double rotateX = gamepad1.right_stick_x;
        }
    }


}
