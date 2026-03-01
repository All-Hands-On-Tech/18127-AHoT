package org.firstinspires.ftc.teamcode.Auto; // make sure this aligns with class location

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.ZSupport.Utilities000;

@Autonomous(name = "Back Auto Blue", group = "A")
public class BackBlueAuto extends OpMode {

    Utilities000 bot = new Utilities000(this);
    private Timer pathTimer, actionTimer, opmodeTimer;
    private ElapsedTime clock = new ElapsedTime();

    private int pathState;

    public PathChain Path1;
    public PathChain Path2;
    public PathChain Spike3PreIntake;
    public PathChain Spike3Intake;
    public PathChain Spike3Return;
    public PathChain Path3;
    public PathChain LeftoverPreIntake;
    public PathChain LeftoverIntake;
    public PathChain LeftoverReturn;

    public Pose startPose = new Pose(55, 10, Math.toRadians(-90));

    private boolean wasBusy = false;


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

                                new Pose(55, 13)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(-90))
                .build();

        Spike3PreIntake = bot.follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(55, 13),
                                new Pose(42.886, 35.152)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(180))
                .build();

        Spike3Intake = bot.follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(42.886, 35.152),
                                new Pose(9.107, 35.289)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        Spike3Return = bot.follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(9.107, 35.289),
                                new Pose(50.300, 36.860),
                                new Pose(55, 13)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(-90))
                .build();

        Path3 = bot.follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(55, 13),

                                new Pose(55, 30)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(180))
                .build();

        LeftoverPreIntake = bot.follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(55, 30),
                                new Pose(15, 40)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(-120))
                .build();

        LeftoverIntake = bot.follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(15, 40),
                                new Pose(13, 15)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-120), Math.toRadians(-90))
                .build();

        LeftoverReturn = bot.follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(13, 15),
                                new Pose(55, 13)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(-90))
                .build();
    }



    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
//                bot.follower.setPose(new Pose(84, 0, Math.toRadians(-90)));
                bot.setManualAimOffsets(-5,10);
                if(clock.milliseconds()>2000 && clock.milliseconds()<4000){
                    bot.intakePower(0.7);
                    bot.setTransferDown();
                } else if(clock.milliseconds()>4000 && clock.milliseconds()<5000){
                    bot.setTransferUp();
                } else if(clock.milliseconds()>5000){
                    bot.follower.setPose(startPose);
                    bot.setTransferBlock();
                    setPathState(1);
                }
                break;
            case 1:

                if(wasBusy){
                    clock.reset();
                }
                if(clock.milliseconds() > 5000){
                    setPathState(2);
                    bot.follower.breakFollowing();
                }

                bot.follower.followPath(Path1);
                setPathState(2);
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
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!bot.follower.isBusy()) {
                    /* Set the state to a Case we won't use or define, so it just stops running an new paths */
                    setPathState(4);
                    clock.reset();
                }
                break;
            case 4:
                if(!bot.follower.isBusy()) {
                    if(wasBusy){
                        clock.reset();
                    }
                    if (clock.milliseconds() > 0 && clock.milliseconds() < 1500) {
                        bot.intakePower(-0.0);
                        bot.setTransferDown();
                    }
                    if (clock.milliseconds() > 1500 && clock.milliseconds() < 2500) {
                        bot.intakePower(0.8);
                        bot.setTransferDown();
                    } else if (clock.milliseconds() > 2500 && clock.milliseconds() < 3000) {
                        bot.setTransferUp();
                    } else if (clock.milliseconds() > 3000) {
                        bot.setTransferBlock();
                        setPathState(5);
                    }
                }
                break;
            case 5:
                if(!bot.follower.isBusy()) {
                    bot.follower.followPath(Spike3PreIntake);
                    setPathState(6);
                }
                break;
            case 6:
                if(!bot.follower.isBusy()) {
                    bot.intakePower(1);
                    bot.follower.followPath(Spike3Intake);
                    setPathState(7);
                }
                break;
            case 7:
                if(!bot.follower.isBusy()) {
                    bot.intakePower(0.8);
                    bot.follower.followPath(Spike3Return);
                    setPathState(8);
                }
                break;
            case 8:
                if(!bot.follower.isBusy()) {
                    if(wasBusy){
                        clock.reset();
                    }
                    if (clock.milliseconds() > 0 && clock.milliseconds() < 1500) {
                        bot.intakePower(-0.0);
                        bot.setTransferDown();
                    }
                    if (clock.milliseconds() > 1500 && clock.milliseconds() < 2500) {
                        bot.intakePower(0.8);
                        bot.setTransferDown();
                    } else if (clock.milliseconds() > 2500 && clock.milliseconds() < 3000) {
                        bot.setTransferUp();
                    } else if (clock.milliseconds() > 3000) {
                        bot.setTransferBlock();
                        setPathState(9);
                    }
                }
                break;
            case 9:
                if(!bot.follower.isBusy()){
                    bot.follower.followPath(Path3);
                    setPathState(10);
                } //lofan was here
                break;
            case 10:
                if(!bot.follower.isBusy()){
                    bot.follower.followPath(LeftoverPreIntake);
                    setPathState(11);
                }
                break;
            case 11:
                if(!bot.follower.isBusy()){
                    bot.follower.followPath(LeftoverIntake);
                    setPathState(12);
                }
                break;
            case 12:
                if(!bot.follower.isBusy()){
                    bot.follower.followPath(LeftoverReturn);
                    setPathState(13);
                }
                break;
            case 13:
                if(!bot.follower.isBusy()) {
                    if(wasBusy){
                        clock.reset();
                    }
                    if (clock.milliseconds() > 0 && clock.milliseconds() < 1500) {
                        bot.intakePower(-0.0);
                        bot.setTransferDown();
                    }
                    if (clock.milliseconds() > 1500 && clock.milliseconds() < 2500) {
                        bot.intakePower(0.8);
                        bot.setTransferDown();
                    } else if (clock.milliseconds() > 2500 && clock.milliseconds() < 3000) {
                        bot.setTransferUp();
                    } else if (clock.milliseconds() > 3000) {
                        bot.setTransferBlock();
                        setPathState(14);
                    }
                }
                break;
            case 14:
                if(!bot.follower.isBusy()){
                    bot.follower.followPath(Path3);
                    setPathState(-1);
                }
                break;
            case -1:
                if(!bot.follower.isBusy()){
                    bot.setFlywheelVolts(0);
                    bot.setHoodYawPower(0);
                    bot.intakePower(0);
                    bot.setTransferBlock();
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

        bot.hoodYawMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bot.hoodYawMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

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
        Utilities000.RobotStateAfterAuto.setPostAutoState(bot.follower.getPose(), bot.getHoodYawAngleTicks());
    }

}