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

    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        bot.initialize(this);
        bot.turnOffCamera();
//        buildPaths();

        bot.hoodYawMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bot.hoodYawMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

//        bot.follower.setStartingPose(startPose);
        bot.setAllianceColor(Utilities000.AllianceColor.RED);
    }

    @Override
    public void start() {
        opmodeTimer.resetTimer();
        bot.hoodYawMotor.setPower(1);
        clock.reset();
        bot.setTransferBlock();
    }

    @Override
    public void loop() {
        // Feedback to Driver Hub for debugging
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

    /**
     * These change the states of the paths and actions. It will also reset the timers of the individual switches
     **/
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }
    void deliver(int nextState){
        if(wasBusy){
            clock.reset();
        }
        if (clock.milliseconds() > 0 && clock.milliseconds() < 1650) {
            bot.intakePower(1);
            bot.setTransferUp();
        } else if (clock.milliseconds() > 1650) {
            bot.setTransferDown();
            setPathState(nextState);
        }
    }

    void deliverBackZone(int nextState){
        if(wasBusy){
            clock.reset();
        }
        if(clock.milliseconds() > 0 && clock.milliseconds() < 1500){

        }
        else if (clock.milliseconds() > 1500 && clock.milliseconds() < 3000) {
            bot.intakePower(1);
            bot.setTransferUp();
        } else if (clock.milliseconds() > 4500) {
            bot.setTransferDown();
            setPathState(nextState);
        }
    }
}
