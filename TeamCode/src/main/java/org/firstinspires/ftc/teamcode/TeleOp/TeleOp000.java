package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.ZSupport.Utilities000;

@Disabled
@TeleOp(name="0-0-0 TeleOp", group="A")
public class TeleOp000 extends OpMode {

    private int maxManualAimOffsetMagnitude = 10;
    Utilities000 bot = new Utilities000(this);
    private ElapsedTime loopTime = new ElapsedTime();
    private ElapsedTime cameraDelay = new ElapsedTime();

    private int shotPower = 0;
    private double yaw = 0;
    private double pitch = 0.6;
    private double speedFactor = 1;
    private ElapsedTime transferTimer = new ElapsedTime();
    private ElapsedTime limeLightStaller = new ElapsedTime(10);
    private boolean transfered = false;
    private boolean aiming = false;
    private boolean shootNMove = true;

    public boolean flyWheelPowerIsAllowed = true;

    public enum TelemetryMode {
        DELIVERY,
        CIRCUIT,
        DEBUG,
        ODOMETRY,
        NONE;
    }
    private TelemetryMode telemetryMode;

    @Override
    public void init() {
        bot.initialize(this);
        bot.turnOffCamera();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        if(Utilities000.RobotStateAfterAuto.wasAuto){
            bot.follower.setPose(Utilities000.RobotStateAfterAuto.postAutoPose);
            bot.initialYawAfterAuto = Utilities000.RobotStateAfterAuto.postAutoYawAngle;
        }

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
            speedFactor = 1.0;
        }

        if (gamepad.yWasPressed()) {
            bot.turnOnCamera();
            Pose[] blobs = bot.getArtifactPoses();
            if (blobs.length>0) {
                telemetry.addData("blobbby: ",    blobs[0].getX());
                bot.follower.followPath(bot.pathToArtifacts(blobs));
            }
        } else if(gamepad.yWasReleased()) {
            bot.follower.startTeleOpDrive();
            bot.turnOffCamera();
        }

        if(bot.getAllianceColor() == Utilities000.AllianceColor.BLUE){
            bot.move(gamepad.left_stick_y, gamepad.left_stick_x, -gamepad.right_stick_x, speedFactor);
        }else {
            bot.move(-gamepad.left_stick_y, -gamepad.left_stick_x, -gamepad.right_stick_x, speedFactor);
        }
        bot.updateOdo();
        if (limeLightStaller.seconds()>0.5) {
            if (bot.limelightUpdate()) {
                limeLightStaller.reset();
            }
        }
    }

    public void handleDelivery(Gamepad gamepad){

        if(gamepad.dpad_up) {
            bot.pitchShift += 0.003;
        } else if (gamepad.dpad_down) {
            bot.pitchShift -= 0.003;
        }

//        if(gamepad.xWasPressed() && !aiming) {
//            aiming = true;
//        } else if(gamepad.xWasPressed() && aiming){
//            aiming = false;
//        }

        if(gamepad.xWasPressed()){
            aiming = !aiming;
        }

        if(gamepad.y){
            if(gamepad.dpad_up){
                shootNMove = true;
            }
            if(gamepad.dpad_down){
//                shootNMove = false;
            }
        }

        if(aiming){
            bot.turrentUpdate(shootNMove);
            bot.hoodYawMotor.setPower(1);
        } else{
            bot.flywheelController(0);
            bot.hoodYawMotor.setPower(0);
        }
    }

    public void handleDelivery(Gamepad gamepad, boolean isCameraOnly){
        if(Math.floor(cameraDelay.milliseconds()) > 10) {
            bot.updateOdo();
            bot.limelightUpdate();
            cameraDelay.reset();
        }
        handleDelivery(gamepad);
    }

    public void handleAimAssist(Gamepad gamepad){
        if(gamepad.dpadRightWasPressed()) {
            bot.yawShift += 2;
        } else if (gamepad.dpadLeftWasPressed()) {
            bot.yawShift -= 2;
        }

        if(gamepad.dpadUpWasPressed()){
            bot.manualFlywheelPowerConstant += 100;
        }
        if(gamepad.dpadDownWasPressed()){
            bot.manualFlywheelPowerConstant -= 100;
        }
        if(gamepad.y){
            if(Math.abs(gamepad.left_stick_x) > 0.05 || Math.abs(gamepad.left_stick_y) > 0.05){
                double x = gamepad.left_stick_x;
                double y = -gamepad.left_stick_y;
                double theta = Math.atan(y/x);
                bot.setHoodYawAngleDegrees(Math.toDegrees(theta));
                aiming = false;
                bot.setHoodYawPower(1);
                if(gamepad.xWasPressed()){
                    bot.hoodYawMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    bot.hoodYawMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                }
            }
        }
        if(Math.abs(gamepad.right_stick_x) > 0.05 || Math.abs(gamepad.right_stick_y) > 0.05){
            double x = gamepad.right_stick_x / Math.sqrt(2);
            double y = -gamepad.right_stick_y / Math.sqrt(2);
            if(bot.getAllianceColor() == Utilities000.AllianceColor.BLUE){
                x = -x;
                y = -y;
            }
            double tempX = x;
            double tempY = y;
            x = (tempX+tempY)/Math.sqrt(2);
            y = (tempY-tempX)/Math.sqrt(2);
            x *= maxManualAimOffsetMagnitude;
            y *= maxManualAimOffsetMagnitude;
            bot.setManualAimOffsets(x, y);
        }else{
            bot.setManualAimOffsets(0,0);
        }
        if(gamepad.aWasPressed()){
            bot.limelightUpdate();
        }
        if(gamepad.left_trigger_pressed){
            maxManualAimOffsetMagnitude = 30;
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


        if(gamepad.aWasPressed() || gamepad.rightBumperWasPressed()){
            transfered = false;
            bot.setTransferDown();
//            bot.intakePower(1);
        } else if(gamepad.aWasReleased() || gamepad.rightBumperWasReleased()){
            transfered = true;
            bot.setTransferUp();
            transferTimer.reset();
        } else if(transfered && transferTimer.seconds() > 0.5){
            bot.setTransferBlock();
            transfered = false;
        }
    }

    public void handleIntake(Gamepad gamepad, Gamepad auxGamepad){
        if(gamepad.right_trigger > 0.01){
            bot.intakePower(gamepad.right_trigger);
        } else if(gamepad.left_trigger > 0.01){
            bot.intakePower(-gamepad.left_trigger * 0.5);
        }

        if(auxGamepad.right_trigger > 0.01){
            bot.intakePower(auxGamepad.right_trigger);
        }else if(auxGamepad.left_trigger > 0.01){
            bot.intakePower(-auxGamepad.left_trigger * 0.5);
        }

        if(gamepad.right_trigger < 0.01 && gamepad.left_trigger < 0.01 && auxGamepad.right_trigger < 0.01 && auxGamepad.left_trigger < 0.01){
            bot.intakePower(0);
        }


        if(auxGamepad.aWasPressed() || auxGamepad.rightBumperWasPressed()){
            transfered = false;
            bot.setTransferDown();
//            bot.intakePower(1);
        } else if(auxGamepad.aWasReleased() || auxGamepad.rightBumperWasReleased()){
            transfered = true;
            bot.setTransferUp();
            transferTimer.reset();
        } else if(transfered && transferTimer.seconds() > 0.5){
            bot.setTransferBlock();
            transfered = false;
        }
    }

    public void handleTilt(Gamepad gamepad){
        if(gamepad.dpad_up){
            bot.retractTilt();
        }
        if(gamepad.dpad_down){
            bot.deployTilt();
        }
    }

    public void toggleFlywheelPower(Gamepad gamepad){
        if(gamepad.rightStickButtonWasPressed()){
            flyWheelPowerIsAllowed = !flyWheelPowerIsAllowed;
        }
        if(!flyWheelPowerIsAllowed) bot.zeroFlywheelPower();
    }

    public void handleTelemetry(){
        telemetry.addData("Telemetry Mode: ", telemetryMode);
        telemetry.addLine("=========");
        switch (telemetryMode){
            case DEBUG:
                bot.addAmpTelemetry();
                Pose LLResult = bot.readLimeLight();
                telemetry.addData("LLResult x - offset", LLResult.getX());
                telemetry.addData("LLResult Y - offset", LLResult.getY());
                telemetry.addData("LLResult Heading - offset", LLResult.getHeading());
                telemetry.addLine();
                telemetry.addLine();
                telemetry.addData("loop time: ", (int)loopTime.milliseconds());
                telemetry.addData("Angle to point: ", bot.getAngleRelativeToPoint(0,0));
                telemetry.addData("Turret Yaw Deg: ", bot.getHoodYawAngleDegrees());
                telemetry.addData("Flywheel ticks/s: ", shotPower);
                telemetry.addData("Flywheel speed: ", bot.getFlywheelSpeed());
                telemetry.addData("Flywheel yaw:     ", yaw);
                telemetry.addData("Flywheel pitch:   ", pitch);
                telemetry.addData("Unnormalized Heading", bot.getUnnormalizedRobotHeading());
                telemetry.addData("loop time: ", (int)loopTime.milliseconds());
                telemetry.addData("Limelight Readout X: ", bot.readLimeLight().getX());
                telemetry.addData("Limelight Readout Y: ", bot.readLimeLight().getY());
                telemetry.addData("Limelight Readout YAW: ", bot.readLimeLight().getHeading());
                telemetry.addData("Follower Pose X: ", bot.follower.getPose().getX());
                telemetry.addData("Follower Pose Y: ", bot.follower.getPose().getY());
                telemetry.addData("Follower Pose YAW: ", bot.follower.getPose().getHeading());
            case DELIVERY:
                telemetry.addData("Turret Yaw Deg: ", bot.getHoodYawAngleDegrees());
                telemetry.addData("Flywheel ticks/s: ", bot.flywheelSpeedFit());
                telemetry.addData("Flywheel speed: ", bot.getFlywheelSpeed());
                telemetry.addData("Flywheel yaw:     ", yaw);
                telemetry.addData("Flywheel pitch:   ", pitch);
                telemetry.addData("Angle to point: ", bot.getAngleRelativeToPoint(0,0));
                telemetry.addData("Initial Yaw Angle (ticks):", Utilities000.RobotStateAfterAuto.postAutoYawAngle);
                telemetry.addData("Was Auto Previously?", Utilities000.RobotStateAfterAuto.wasAuto);
            case CIRCUIT:
                bot.addAmpTelemetry();
                telemetry.addData("loop time: ", (int)loopTime.milliseconds());
            case ODOMETRY:
                telemetry.addData("Position: ", bot.getPosition());
                telemetry.addData("Heading: ", Math.toDegrees(bot.getPosition().getHeading()));
                telemetry.addData("Angle to point: ", bot.getAngleRelativeToPoint(0,0));
                telemetry.addData("Distance to Goal (inch): ", bot.getGoal().distanceFrom(bot.follower.getPose()));
            case NONE:
        }

        loopTime.reset();
        telemetry.update();
    }

    public void setTelemetryMode(TelemetryMode telemetryMode){
        this.telemetryMode = telemetryMode;
    }

    @Override
    public void stop(){
        Utilities000.RobotStateAfterAuto.setPostAutoState(null,0);
        Utilities000.RobotStateAfterAuto.wasAuto = false;
    }
}
