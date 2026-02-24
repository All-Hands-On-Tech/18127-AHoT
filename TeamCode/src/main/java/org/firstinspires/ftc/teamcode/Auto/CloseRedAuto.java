package org.firstinspires.ftc.teamcode.Auto; // make sure this aligns with class location

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.ZSupport.Utilities000;

@Autonomous(name = "Close Auto Red", group = "A")
public class CloseRedAuto extends OpMode {

    Utilities000 bot = new Utilities000(this);
    private Timer pathTimer, actionTimer, opmodeTimer;
    private ElapsedTime clock = new ElapsedTime();

    private int pathState;

    public PathChain Path1;
    public PathChain Path2;
    public PathChain Path3;
    public PathChain Path4;
    public PathChain Path5;
    public PathChain Path6; //Shift Paths and make Path 3 to hit gate

    private boolean wasBusy = false;

    public Pose startPose = new Pose(117.414, 131.042, Math.toRadians(-143));


    public void buildPaths() {
        Path1 = bot.follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(startPose.getX(), startPose.getY()),
                                new Pose(90.489, 92.069)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-143), Math.toRadians(-45))
                .build();

        Path2 = bot.follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(90.489, 92.069),
                                new Pose(120, 82.909)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-45), Math.toRadians(0))
                .build();

        Path3 = bot.follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(120, 82.909),
                                new Pose(90, 83.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-45))
                .build();

        Path4 = bot.follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(90, 83.000),
                                new Pose(106.087, 82.306),
                                new Pose(77.628, 53),
                                new Pose(125, 53)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-45), Math.toRadians(0))
                .build();

        Path5 = bot.follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(125, 53),
                                new Pose(77.913, 58.488),
                                new Pose(86.522, 82.739)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-45))
                .build();
    }



    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                bot.follower.followPath(Path1);
                setPathState(1);
                break;
            case 1:
                if(!bot.follower.isBusy()) {
                    if(wasBusy){
                        clock.reset();
                    }
                    //bot.follower.setPose(new Pose(60, 0, Math.toRadians(-90))); Removed because Khai doesn't understand why it was used in Back Autos
                    if (clock.milliseconds() > 0 && clock.milliseconds() < 1000) {
                        bot.intakePower(1);
                        bot.setTransferDown();
                    } else if (clock.milliseconds() > 1000 && clock.milliseconds() < 1500) {
                        bot.setTransferUp();
                    } else if (clock.milliseconds() > 1500) {
                        bot.setTransferBlock();
                        setPathState(2);
                    }
                }
                break;
            case 2:

                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!bot.follower.isBusy()) {
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    bot.follower.followPath(Path2,true);
                    setPathState(3);
                }
                break;

            case 3:
                if(!bot.follower.isBusy()){
                    bot.intakePower(0.8);
                    if(wasBusy){
                        clock.reset();
                    }
                    if(clock.milliseconds() > 1000){
                        bot.follower.followPath(Path3);
                        setPathState(4);
                    }
                }
                break;
            case 4:
                setPathState(5); // Hit Gate First!!!
                break;
            case 5:
                if(!bot.follower.isBusy()) {
                    if(wasBusy){
                        clock.reset();
                    }
                    if (clock.milliseconds() > 500 && clock.milliseconds() < 1500) {
                        bot.intakePower(1);
                        bot.setTransferDown();
                    } else if (clock.milliseconds() > 1500 && clock.milliseconds() < 2000) {
                        bot.setTransferUp();
                    } else if (clock.milliseconds() > 2000) {
                        bot.setTransferBlock();
                        setPathState(6);
                    }
                }
                break;
            case 6:
                if(!bot.follower.isBusy()) {
                    bot.follower.followPath(Path4,true);
                    setPathState(7);
                }
                break;
            case 7:
                if(!bot.follower.isBusy()) {
                    bot.intakePower(0.8);
                    bot.follower.followPath(Path5,true);
                    setPathState(8);
                }
                break;
            case 8:
                if(!bot.follower.isBusy()) {
                    if(wasBusy){
                        clock.reset();
                    }
                    if (clock.milliseconds() > 500 && clock.milliseconds() < 1500) {
                        bot.intakePower(1);
                        bot.setTransferDown();
                    } else if (clock.milliseconds() > 1500 && clock.milliseconds() < 2000) {
                        bot.setTransferUp();
                    } else if (clock.milliseconds() > 2000) {
                        bot.setTransferBlock();
                        bot.setHoodYawPower(0);
                        bot.setFlywheelVolts(0);
                        bot.intakePower(0);
                        setPathState(9);
                    }
                }
                break;
            case 9:
                if(!bot.follower.isBusy()) {
                    setPathState(-1);
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

        wasBusy = bot.follower.isBusy();

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
        bot.setAllianceColor(Utilities000.AllianceColor.RED);

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