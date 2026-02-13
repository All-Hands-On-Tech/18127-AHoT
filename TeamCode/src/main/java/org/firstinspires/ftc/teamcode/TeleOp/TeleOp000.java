package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.ZSupport.Utilities000;

@TeleOp(name="0-0-0 TeleOp", group="A")
public class TeleOp000 extends LinearOpMode {
    Utilities000 bot = new Utilities000(this);
    private ElapsedTime loopTime = new ElapsedTime();

    private int shotPower = 0;
    private double yaw = 0;
    private double pitch = 0.6;
    private double speedFactor = 1;

    public enum TelemetryMode {
        DELIVERY,
        CIRCUIT,
        DEBUG;
    }
    private TelemetryMode telemetryMode;

    @Override
    public void runOpMode() {
        bot.initialize(this);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        loopTime.reset();

    }

    public void handleDrivetrain(Gamepad gamepad){
        if(gamepad.right_bumper){
            speedFactor = 0.6;
        }else{
            speedFactor = 0.8;
        }

        bot.move(-gamepad.left_stick_y*speedFactor, gamepad.left_stick_x*speedFactor, -gamepad.right_stick_x*speedFactor);
    }

    public void handleDelivery(Gamepad gamepad){
        if(gamepad.rightBumperWasPressed()) {
            shotPower += 100;
        } else if (gamepad.leftBumperWasPressed()) {
            shotPower -= 100;
        }
        bot.setFlywheelSpeed_DO_NOT_USE(shotPower);

        if(gamepad.dpad_right) {
            yaw += 5;
        } else if (gamepad.dpad_left) {
            yaw -= 5;
        }
        bot.setHoodYawAngleTicks(yaw);
        bot.setHoodYawPower(0.5);

        if(gamepad.dpad_up) {
            pitch += 0.006;
        } else if (gamepad.dpad_down) {
            pitch -= 0.006;
        }
        bot.setHoodPitchAngleTicks(pitch);
    }

    public void handleIntake(Gamepad gamepad){
        if(gamepad.right_trigger > 0.01){
            bot.intakePower(gamepad.right_trigger);
        }

        if(gamepad.left_trigger > 0.01){
            bot.intakePower(gamepad.left_trigger);
        }

        if(gamepad.aWasPressed()){
            bot.setTransferDown();
        }

        if(gamepad.aWasReleased()){
            bot.setTransferUp();
        }
    }

    public void handleTelemetry(){
        telemetry.addData("Telemetry Mode: ", telemetryMode);
        telemetry.addLine("=========");
        switch (telemetryMode){
            case DEBUG:
                bot.addAmpTelemetry();
                telemetry.addData("loop time: ", (int)loopTime.milliseconds());
                telemetry.addData("Turret Yaw Deg: ", bot.getHoodYawAngleDegrees());
                telemetry.addData("Flywheel ticks/s: ", shotPower);
                telemetry.addData("Flywheel speed: ", bot.getFlywheelSpeed());
                telemetry.addData("Flywheel yaw:     ", yaw);
                telemetry.addData("Flywheel pitch:   ", pitch);
                telemetry.addData("loop time: ", (int)loopTime.milliseconds());
            case DELIVERY:
                telemetry.addData("Turret Yaw Deg: ", bot.getHoodYawAngleDegrees());
                telemetry.addData("Flywheel ticks/s: ", shotPower);
                telemetry.addData("Flywheel speed: ", bot.getFlywheelSpeed());
                telemetry.addData("Flywheel yaw:     ", yaw);
                telemetry.addData("Flywheel pitch:   ", pitch);
            case CIRCUIT:
                bot.addAmpTelemetry();
                telemetry.addData("loop time: ", (int)loopTime.milliseconds());
        }

        loopTime.reset();
        telemetry.update();
    }

    public void setTelemetryMode(TelemetryMode telemetryMode){
        this.telemetryMode = telemetryMode;
    }

}
