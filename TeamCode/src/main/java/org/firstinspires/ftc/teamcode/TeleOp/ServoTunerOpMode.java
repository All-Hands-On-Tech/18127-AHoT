package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;


@TeleOp(name="ServoTuner", group="A")
public class ServoTunerOpMode extends LinearOpMode {

    private Servo servo;
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
                    servo = hardwareMap.get(Servo.class, TunerParams.servoName);
                    lastServoName = TunerParams.servoName;
                } catch (Exception e){
                    telemetry.addData("Error", "Servo not found: " + TunerParams.servoName);
                }
            }

            if(servo != null){

                if(Math.hypot(-gamepad1.left_stick_y,gamepad1.left_stick_x) > 0.8f){
                    double angleDeg = Math.toDegrees(Math.atan2(-gamepad1.left_stick_y, gamepad1.left_stick_x));
                    TunerParams.position = (angleDeg + 180.0) / 360.0;
                }

                tuneServo(servo);


                telemetry.update();
            }

        }

    }

    public void tuneServo(Servo servo){
        if(TunerParams.servoEnabled ){
            servo.getController().pwmEnable();
        }

        if(!TunerParams.servoEnabled ){
            servo.getController().pwmDisable();
        }


        servo.scaleRange(TunerParams.min,TunerParams.max);

        if(servo.getController().getPwmStatus() == ServoController.PwmStatus.ENABLED){
            telemetry.addLine("updating position");
            servo.setPosition(TunerParams.position);
        }

        servo.setDirection(TunerParams.direction);
        telemetry.addData("Position: ", servo.getPosition());
        telemetry.addData("Enabled: ", ServoController.PwmStatus.ENABLED);

        servoEnabledLast = TunerParams.servoEnabled;
    }

    @Config
    public static class TunerParams{
        public static String servoName = "";
        public static boolean servoEnabled;
        public static double min = 0, max = 1, position = 0.5;

        public static Servo.Direction direction = Servo.Direction.FORWARD;
    }

}
