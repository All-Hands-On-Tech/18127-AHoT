package org.firstinspires.ftc.teamcode.Auto; // make sure this aligns with class location

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.ZSupport.Utilities000;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants000;

@Autonomous(name = "Back Auto Blue", group = "A")
public class BackBlueAuto extends OpMode {

    Utilities000 bot = new Utilities000(this);
    private Timer pathTimer, actionTimer, opmodeTimer;
    private ElapsedTime clock = new ElapsedTime();

    private int pathState;

    public PathChain Path1;
    public PathChain Path2;

    public Pose startPose = new Pose(57.229, 9.229, Math.toRadians(-90));


    public void buildPaths() {
        Path1 = bot.follower.pathBuilder().addPath(
                        new BezierCurve(
                                startPose,
                                new Pose(44.535, 12.861),
                                new Pose(10.761, 36.778),
                                new Pose(6.225, 7.118)
                        )
                ).setTangentHeadingInterpolation()

                .build();

        Path2 = bot.follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(6.225, 7.118),

                                startPose
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(-90))

                .build();
    }



    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                if(clock.milliseconds()>1000 && clock.milliseconds()<3000){
                    bot.intakePower(1);
                    bot.setTransferDown();
                } else if(clock.milliseconds()>3000){
                    bot.setTransferUp();
                } else if(clock.milliseconds()>4000){
                    bot.setTransferBlock();
                }
                if(clock.milliseconds()>4000) {
                    setPathState(1);
                }
                break;
            case 1:
                bot.follower.followPath(Path1);
                setPathState(2);
                break;
            case 2:

                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!bot.follower.isBusy()) {
                    /* Score Preload */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    bot.follower.followPath(Path2,true);
                    setPathState(3);
                }
                break;

            case 3:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!bot.follower.isBusy()) {
                    /* Set the state to a Case we won't use or define, so it just stops running an new paths */
                    setPathState(4);
                    clock.reset();
                }
                break;
            case 4:
                if(clock.milliseconds()>1000 && clock.milliseconds()<3000){
                    bot.intakePower(1);
                    bot.setTransferDown();
                } else if(clock.milliseconds()>3000){
                    bot.setTransferUp();
                } else if(clock.milliseconds()>4000){
                    bot.setTransferBlock();
                }
                if(clock.milliseconds()>4000) {
                    setPathState(-1);
                    bot.hoodYawMotor.setPower(0);
                    bot.intakePower(0);
                    bot.setFlywheelVolts(0);
                }
                break;
        }
    }

    /**
     * These change the states of the paths and actions. It will also reset the timers of the individual switches
     **/
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    /**
     * This is the main loop of the OpMode, it will run repeatedly after clicking "Play".
     **/
    @Override
    public void loop() {

        // These loop the movements of the robot, these must be called continuously in order to work
        bot.follower.update();
        if (pathState!=-1) {
            bot.turrentUpdate();
        }
        autonomousPathUpdate();

        // Feedback to Driver Hub for debugging
        telemetry.addData("path state", pathState);
        telemetry.addData("x", bot.follower.getPose().getX());
        telemetry.addData("y", bot.follower.getPose().getY());
        telemetry.addData("heading", bot.follower.getPose().getHeading());
        telemetry.update();
    }

    /**
     * This method is called once at the init of the OpMode.
     **/
    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        bot.initialize(this);
        bot.turnOffCamera();
        buildPaths();

        bot.follower.setStartingPose(startPose);
        bot.setAllianceColor(Utilities000.AllianceColor.BLUE);

    }

    /**
     * This method is called continuously after Init while waiting for "play".
     **/
    @Override
    public void init_loop() {
    }

    /**
     * This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system
     **/
    @Override
    public void start() {
        opmodeTimer.resetTimer();
        bot.hoodYawMotor.setPower(1);
        clock.reset();
        setPathState(0);
        bot.setTransferBlock();
    }

    /**
     * We do not use this because everything should automatically disable
     **/
    @Override
    public void stop() {
    }

}