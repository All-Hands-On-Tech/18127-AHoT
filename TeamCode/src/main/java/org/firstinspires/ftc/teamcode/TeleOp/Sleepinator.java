package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.ZSupport.Utilities000;

import java.util.List;


@TeleOp(name="Sleepinator TeleOp", group="A")
public class Sleepinator extends OpMode {
    private ElapsedTime loopTime = new ElapsedTime();
    private int targetID = 0;
    private double flywheelSpeed = 0;
    private double pCoef = 0.03;

    private boolean targetIsDetected = false;
    Utilities000 bot = new Utilities000(this);

    @Override
    public void init() {
        bot.initialize(this);
        bot.setHoodYawMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        loopTime.reset();
    }

    @Override
    public void init_loop() {

    }

    @Override
    public void start() {
        loopTime.reset();
    }

    @Override
    public void loop() {

        if(gamepad1.dpadRightWasPressed()){
            targetID++;
        }

        if(gamepad1.dpadLeftWasPressed()){
            targetID--;
        }

        if(gamepad1.dpadDownWasPressed()){
            flywheelSpeed -= 100;
        }

        if(gamepad1.dpadUpWasPressed()){
            flywheelSpeed += 100;
        }

        if(gamepad1.right_trigger > 0.1){
            bot.intakePower(gamepad1.right_trigger);
        } else if(gamepad1.left_trigger > 0.1){
            bot.intakePower(gamepad1.left_trigger);
        } else{
            bot.intakePower(0);
        }

        bot.flywheelController(flywheelSpeed);

        telemetry.addData("Flywheel Speed", bot.getFlywheelSpeed());

        LLResult results = bot.getLimeLightResults();

        targetIsDetected = false;
        List<LLResultTypes.FiducialResult> fiducials = results.getFiducialResults();
        for (LLResultTypes.FiducialResult fiducial : fiducials) {
            int id = fiducial.getFiducialId(); // The ID number of the fiducial
            double x = fiducial.getTargetXDegrees(); // Where it is (left-right)
            double y = fiducial.getTargetYDegrees(); // Where it is (up-down)
            telemetry.addData("Detected ID:", id);
            telemetry.addData("x:", x);
            telemetry.addData("y:", y);

            if(id == targetID){
                targetIsDetected = true;
                bot.setHoodYawPower(-x * pCoef);
            }
        }

        if(!targetIsDetected){
            bot.setHoodYawPower(0);
        }

        telemetry.update();
    }
}

