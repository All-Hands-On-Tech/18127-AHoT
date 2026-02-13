package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;


@TeleOp(name="MotorTuner", group="A")
public class MotorTunerOpMode extends LinearOpMode {

    private DcMotorEx motor;
    private String lastMotorName = "";
    private static final double YAW_PULSES_PER_DEGREE = (1.0/360.0) * (70.0/10.0) * (384.5);


    @Override
    public void runOpMode() {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        while (opModeIsActive()) {

            if(!TunerParams.motorName.equals(lastMotorName)){
                try{
                    motor = hardwareMap.get(DcMotorEx.class, TunerParams.motorName);
                    motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    lastMotorName = TunerParams.motorName;
                } catch (Exception e){
                    telemetry.addData("Error", "Motor not found: " + TunerParams.motorName);
                }
            }

            if(motor != null){

                if(Math.hypot(-gamepad1.left_stick_y,gamepad1.left_stick_x) > 0.8f){
                    double angle = Math.atan2(-gamepad1.left_stick_y,gamepad1.left_stick_x);
                    angle = Math.toDegrees(angle);
                    angle = 180-angle;
                    TunerParams.target = (int)(angle * YAW_PULSES_PER_DEGREE);
                }

                tuneMotor(motor);


                telemetry.addData("01 - Target: ", TunerParams.target);
                telemetry.addData("02 - Actual: ", motor.getCurrentPosition());
                telemetry.addData("03 - Error: ", TunerParams.target - motor.getCurrentPosition());

                telemetry.addData("Power: ", motor.getPower());
                telemetry.addData("Current Velocity: ", motor.getVelocity());
                telemetry.addData("Current (A): ", motor.getCurrent(CurrentUnit.AMPS));

                telemetry.update();
            }

        }

    }

    public void tuneMotor(DcMotorEx motor){
        PIDFCoefficients vCoefficients = new PIDFCoefficients(TunerParams.P, TunerParams.I, TunerParams.D, TunerParams.F);
        motor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, vCoefficients);

        PIDFCoefficients pCoefficients = new PIDFCoefficients(TunerParams.pP, 0,0,0);
        motor.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION, pCoefficients);

        motor.setTargetPosition(TunerParams.target);
        motor.setPower(TunerParams.power);
    }

    @Config
    public static class TunerParams{
        public static String motorName = "";
        public static double P = 10, I = 3, D = 0, F = 0, pP = 0;
        public static int target = 0;
        public static double power = 0;

    }

}
