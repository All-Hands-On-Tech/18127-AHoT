package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;


@TeleOp(name="ServoTuner", group="A")
public class ServoTunerOpMode extends LinearOpMode {

    private ServoImplEx servoImpl;
    private String lastServoName = "";
    boolean servoEnabledLast = false;


    @Override
    public void runOpMode() {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        while (opModeIsActive()) {

            if(!TunerParams.servoName.equals(lastServoName)){
                try{
                    servoImpl = hardwareMap.get(ServoImplEx.class, TunerParams.servoName);
                    lastServoName = TunerParams.servoName;
                } catch (Exception e){
                    telemetry.addData("Error", "Motor not found: " + TunerParams.servoName);
                }
            }

            if(servoImpl != null){

                if(Math.hypot(-gamepad1.left_stick_y,gamepad1.left_stick_x) > 0.8f){
                    double angle = Math.atan2(-gamepad1.left_stick_y,gamepad1.left_stick_x);
                    angle = Math.toDegrees(angle);
                    TunerParams.position = angle / (Math.PI * 2);
                }

                tuneServo(servoImpl);


                telemetry.update();
            }

        }

    }

    public void tuneServo(ServoImplEx servo){

        if(TunerParams.servoEnabled && !servoEnabledLast){
            servo.setPwmEnable();
        }

        if(!TunerParams.servoEnabled && servoEnabledLast){
            servo.setPwmDisable();
        }

        servo.setPwmRange(new PwmControl.PwmRange(TunerParams.min, TunerParams.max));
        servo.setPosition(TunerParams.position);
        servo.setDirection(TunerParams.direction);

        telemetry.addData("Is PWM Enabled: ", servo.isPwmEnabled());
        telemetry.addData("Range: ", servo.getPwmRange());
        telemetry.addData("Position: ", servo.getPosition());
        telemetry.update();
    }

    @Config
    public static class TunerParams{
        public static String servoName = "";
        public static boolean servoEnabled;
        public static double min = 0, max = 1, position = 0.5;

        public static Servo.Direction direction = Servo.Direction.FORWARD;
    }

}
