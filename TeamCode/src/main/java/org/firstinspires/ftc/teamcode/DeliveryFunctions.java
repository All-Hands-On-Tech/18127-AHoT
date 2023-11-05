package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Disabled
public class DeliveryFunctions {
    private DcMotor leftSlide = null;
    private DcMotor rightSlide = null;
    private LinearOpMode linearOpMode;
    private double CLICKS_PER_METER = 2492.788;

    private int targetPosition;
    private double leftError;
    private double rightError;
    private double currentPosition;

    public final double TICK_STOP_THRESHOLD = 20;
    public final double TICK_SLOW_THRESHOLD = 250;


    public DeliveryFunctions(LinearOpMode l)
    {
        linearOpMode = l;
        Initialize();
    }


    private void Initialize(){
        leftSlide  = linearOpMode.hardwareMap.get(DcMotor.class, "leftSlide");
        rightSlide  = linearOpMode.hardwareMap.get(DcMotor.class, "rightSlide");
        leftSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void setSlidesTargetPosition(int clicks){
        targetPosition = clicks;
        leftSlide.setTargetPosition(targetPosition);
        rightSlide.setTargetPosition(targetPosition);
    }

    public void setSlidesPower(double power){
        leftSlide.setPower(power);
        rightSlide.setPower(power);
    }

    public int getMotorPositionByIndex(int index){
        if(index == 0){
            return leftSlide.getCurrentPosition();
        } else {
            return rightSlide.getCurrentPosition();
        }

    }

    public void PControlPower(){
        leftError = targetPosition - leftSlide.getCurrentPosition();
        rightError = targetPosition - rightSlide.getCurrentPosition();

        leftSlide.setPower(leftError / TICK_SLOW_THRESHOLD);
        rightSlide.setPower(rightError / TICK_SLOW_THRESHOLD);
    }

    public void Dump(){
        linearOpMode.telemetry.addLine("WIP");
    }

    public void WristMovementByLiftPosition(){
        //move wrist if outside of transfer area
    }

}
