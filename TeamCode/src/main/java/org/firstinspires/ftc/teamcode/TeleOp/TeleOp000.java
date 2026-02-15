package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.ZSupport.Utilities000;

import java.util.Timer;

@Disabled
@TeleOp(name="0-0-0 TeleOp", group="A")
public class TeleOp000 extends LinearOpMode {
    Utilities000 bot = new Utilities000(this);
    private ElapsedTime loopTime = new ElapsedTime();

    private int shotPower = 0;
    private double yaw = 0;
    private double pitch = 0.6;
    private double speedFactor = 1;
    private ElapsedTime transferTimer = new ElapsedTime();
    private boolean transfered = false;

    public enum TelemetryMode {
        DELIVERY,
        CIRCUIT,
        DEBUG,
        ODOMETRY;;
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
        bot.updateOdo();
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

//        bot.aimAtPoint(80.025,176.475);
        bot.aimAtPoint(-10,0);
//        bot.aimAtPoint(0,0);
//        bot.setHoodYawAngleTicks(yaw);
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
        } else if(gamepad.left_trigger > 0.01){
            bot.intakePower(-gamepad.left_trigger);
        } else{
            bot.intakePower(0);
        }



        if(gamepad.aWasPressed()){
            transfered = false;
            bot.setTransferDown();
        } else if(gamepad.aWasReleased()){
            transfered = true;
            bot.setTransferUp();
            transferTimer.reset();
        } else if(transfered && transferTimer.seconds() > 0.5){
            bot.setTransferBlock();
            transfered = false;
        }
    }

    public void handleTelemetry(){
        telemetry.addData("Telemetry Mode: ", telemetryMode);
        telemetry.addLine("=========");
        switch (telemetryMode){
            case DEBUG:
                bot.addAmpTelemetry();
                telemetry.addData("loop time: ", (int)loopTime.milliseconds());
                telemetry.addData("Angle to point: ", bot.getAngleRelativeToPoint(0,0));
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
                telemetry.addData("Angle to point: ", bot.getAngleRelativeToPoint(0,0));
            case CIRCUIT:
                bot.addAmpTelemetry();
                telemetry.addData("loop time: ", (int)loopTime.milliseconds());
            case ODOMETRY:
                telemetry.addData("Position: ", bot.getPosition());
                telemetry.addData("Angle to point: ", bot.getAngleRelativeToPoint(0,0));
        }

        loopTime.reset();
        telemetry.update();
    }

    public void setTelemetryMode(TelemetryMode telemetryMode){
        this.telemetryMode = telemetryMode;
    }

}
