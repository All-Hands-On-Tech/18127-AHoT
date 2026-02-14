package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;


@TeleOp(name="Dual Mode Tuner", group="A")
public class DualMotorTunerOpMode extends MotorTunerOpMode {

    private DcMotorEx motor1;
    private DcMotorEx motor2;
    private String lastMotor1Name = "";
    private String lastMotor2Name = "";


    @Override
    public void runOpMode() {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        while (opModeIsActive()) {

            if(!DualMotorTunerParams.motor1Name.equals(lastMotor1Name)){
                try{
                    motor1 = hardwareMap.get(DcMotorEx.class, DualMotorTunerParams.motor1Name);
                    motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    lastMotor1Name = DualMotorTunerParams.motor1Name;
                } catch (Exception e){
                    telemetry.addData("Error", "Motor not found: " + DualMotorTunerParams.motor1Name);
                }
            }

            if(!DualMotorTunerParams.motor2Name.equals(lastMotor2Name)){
                try{
                    motor2 = hardwareMap.get(DcMotorEx.class, DualMotorTunerParams.motor2Name);
                    motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    lastMotor2Name = DualMotorTunerParams.motor2Name;
                } catch (Exception e){
                    telemetry.addData("Error", "Motor not found: " + DualMotorTunerParams.motor2Name);
                }
            }

            if(motor1 != null){
                motor1.setDirection(DualMotorTunerParams.direction1);
                tuneMotor(motor1);

                telemetry.addData("01 - Target: ", DualMotorTunerParams.target);
                telemetry.addData("02 - Actual: ", motor1.getVelocity());
                telemetry.addData("03 - Error: ", DualMotorTunerParams.target - motor1.getVelocity());

                telemetry.addData("Power: ", motor1.getPower());
                telemetry.addData("Current Velocity: ", motor1.getVelocity());
                telemetry.addData("Current (A): ", motor1.getCurrent(CurrentUnit.AMPS));

                telemetry.update();
            }

            if(motor2 != null){

                motor2.setDirection(DualMotorTunerParams.direction2);
                tuneMotor(motor2);


                telemetry.addData("01 - Target: ", DualMotorTunerParams.target);
                telemetry.addData("02 - Actual: ", motor2.getVelocity());
                telemetry.addData("03 - Error: ", DualMotorTunerParams.target - motor2.getVelocity());

                telemetry.addData("Power: ", motor2.getPower());
                telemetry.addData("Current Velocity: ", motor2.getVelocity());
                telemetry.addData("Current (A): ", motor2.getCurrent(CurrentUnit.AMPS));

                telemetry.update();
            }

        }

    }

    public void tuneMotor(DcMotorEx motor){
        PIDFCoefficients vCoefficients = new PIDFCoefficients(DualMotorTunerParams.P, DualMotorTunerParams.I, DualMotorTunerParams.D, DualMotorTunerParams.F);
        motor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, vCoefficients);

        motor.setVelocity(DualMotorTunerParams.target);
        motor.setPower(DualMotorTunerParams.power);
    }

    @Config
    public static class DualMotorTunerParams {
        public static String motor1Name = "", motor2Name = "";
        public static double P = 10, I = 3, D = 0, F = 0;
        public static int target = 0;
        public static double power = 0;
        public static DcMotor.Direction direction1 = DcMotor.Direction.FORWARD;
        public static DcMotor.Direction direction2 = DcMotor.Direction.FORWARD;

    }

}
