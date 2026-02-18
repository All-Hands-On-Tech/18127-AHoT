package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.ZSupport.Utilities000;

import java.util.Timer;

@Disabled
@TeleOp(name="0-0-0 TeleOp", group="A")
public class TeleOp000 extends OpMode {
    Utilities000 bot = new Utilities000(this);
    private ElapsedTime loopTime = new ElapsedTime();

    private int shotPower = 0;
    private double yaw = 0;
    private double pitch = 0.6;
    private double speedFactor = 1;
    private ElapsedTime transferTimer = new ElapsedTime();
    private ElapsedTime limeLightStaller = new ElapsedTime(10);
    private boolean transfered = false;
    private boolean aiming = false;

    public enum TelemetryMode {
        DELIVERY,
        CIRCUIT,
        DEBUG,
        ODOMETRY;;
    }
    private TelemetryMode telemetryMode;

    @Override
    public void init() {
        bot.initialize(this);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        telemetry.addData("Status", "Initialized");
        telemetry.update();

    }

    @Override
    public void start() {
        loopTime.reset();
    }

    @Override
    public void loop() {}

    public void handleDrivetrain(Gamepad gamepad){
        if(gamepad.left_bumper){
            speedFactor = 0.3;
        }else{
            speedFactor = 0.8;
        }

        if (gamepad.yWasPressed()) {
            Pose[] blobs = bot.getArtifactPoses();
            if (blobs.length>0) {
                telemetry.addData("blobbby: ",    blobs[0].getX());
                bot.follower.followPath(bot.pathToArtifacts(blobs));
            }
        } else if(gamepad.yWasReleased()) {
            bot.follower.startTeleOpDrive();
        }
        bot.follower.update();

        bot.move(-gamepad.left_stick_y, -gamepad.left_stick_x, -gamepad.right_stick_x, speedFactor);
        bot.updateOdo();
        if (limeLightStaller.seconds()>5) {
            if (bot.limelightUpdate()) {
                limeLightStaller.reset();
            }
        }
    }

    public void handleDelivery(Gamepad gamepad){
//        if(gamepad.rightBumperWasPressed()) {
//            shotPower += 50;
//        } else if (gamepad.leftBumperWasPressed()) {
//            shotPower -= 50;
//        }
//        bot.flywheelController(shotPower);
//
//        if(gamepad.dpad_right) {
//            yaw += 20;
//        } else if (gamepad.dpad_left) {
//            yaw -= 20;
//        }
//
////        bot.aimAtPoint(80.025,176.475);
////        bot.aimAtPoint(-10,0);
////        bot.aimAtPoint(0,0);
//        bot.setHoodYawAngleTicks(yaw);
//        bot.setHoodYawPower(0.5);
//
//        if(gamepad.dpad_up) {
//            pitch += 0.006;
//        } else if (gamepad.dpad_down) {
//            pitch -= 0.006;
//        }
//        bot.setHoodPitchAngleTicks(pitch);
//        if(gamepad.xWasPressed() && !aiming) {
//            aiming = true;
//        } else if(gamepad.xWasPressed() && aiming){
//            aiming = false;
//        }

        if(gamepad.xWasPressed()){
            aiming = !aiming;
        }

        if(aiming){
            bot.turrentUpdate();
            bot.hoodYawMotor.setPower(1);
        } else{
            bot.flywheelController(0);
            bot.hoodYawMotor.setPower(0);
        }
    }

    public void handleIntake(Gamepad gamepad){
        if(gamepad.right_trigger > 0.01){
            bot.intakePower(gamepad.right_trigger);
        } else if(gamepad.left_trigger > 0.01){
            bot.intakePower(-gamepad.left_trigger * 0.5);
        } else{
            bot.intakePower(0);
        }


        if(gamepad.a && gamepad.left_trigger < 0.01) {
            bot.intakePower(1);
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
                telemetry.addData("Heading: ", Math.toDegrees(bot.getPosition().getHeading()));
                telemetry.addData("Angle to point: ", bot.getAngleRelativeToPoint(0,0));
        }

        loopTime.reset();
        telemetry.update();
    }

    public void setTelemetryMode(TelemetryMode telemetryMode){
        this.telemetryMode = telemetryMode;
    }

}
