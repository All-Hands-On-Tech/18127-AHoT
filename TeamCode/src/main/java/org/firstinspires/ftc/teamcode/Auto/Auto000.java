package org.firstinspires.ftc.teamcode.Auto;

import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.ZSupport.Utilities000;

@Disabled
@TeleOp(name="Auto", group="Z")
public class Auto000 extends OpMode {

    Utilities000 bot = new Utilities000(this);
    Timer pathTimer, actionTimer, opmodeTimer;
    ElapsedTime clock = new ElapsedTime();

    int pathState;

    boolean wasBusy = false;

    public enum Alliance { RED, BLUE }
    public Alliance alliance = Alliance.RED;

    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        bot.initialize(this);
        bot.turnOffCamera();

        bot.hoodYawMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bot.hoodYawMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        bot.setAllianceColor(Utilities000.AllianceColor.RED);
    }

    @Override
    public void start() {
        opmodeTimer.resetTimer();
        bot.hoodYawMotor.setPower(1);
        clock.reset();
        bot.setTransferBlock();

        if (alliance == Alliance.BLUE) {
            bot.setAllianceColor(Utilities000.AllianceColor.BLUE);
        } else {
            bot.setAllianceColor(Utilities000.AllianceColor.RED);
        }
    }

    @Override
    public void loop() {
        telemetry.addData("path state", pathState);
        telemetry.addData("x", bot.follower.getPose().getX());
        telemetry.addData("y", bot.follower.getPose().getY());
        telemetry.addData("heading", bot.follower.getPose().getHeading());
        telemetry.update();
    }

    @Override
    public void stop(){
        Utilities000.RobotStateAfterAuto.setPostAutoState(bot.follower.getPose(), bot.getHoodYawAngleTicks());
    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    void deliver(int nextState){
        if(wasBusy){
            clock.reset();
        }
        if (clock.milliseconds() > 0 && clock.milliseconds() < 1800) {
            bot.intakePower(0.9);
            bot.setTransferUp();
        } else if (clock.milliseconds() > 1800) {
            bot.setTransferDown();
            setPathState(nextState);
        }
    }

    void deliverBackZone(int nextState, int intakePower){
        if(wasBusy){
            clock.reset();
        }
        if (clock.milliseconds() < 4000) {
            bot.intakePower(intakePower);
            bot.setTransferUp();
        } else if (clock.milliseconds() > 4000) {
            bot.setTransferDown();
            setPathState(nextState);
        }
    }
}