package org.firstinspires.ftc.teamcode.Auto; // make sure this aligns with class location

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Close Auto Red (Gate)", group = "A")
public class CloseRedAutoGate extends Auto000 {

    public PathChain shot1, shot2, gateToShoot;
    public PathChain gate;
    public PathChain intake1, intake2;

    public Pose startPose = new Pose(117.414, 131.042, Math.toRadians(-143));


    public void buildPaths() {
        shot1 = bot.follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(114.634, 130.457),
                                new Pose(85.000, 85.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-143), Math.toRadians(-30))
                .build();

        intake1 = bot.follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(85.000, 85.000),
                                new Pose(82.855, 57.759),
                                new Pose(118.000, 52.661)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-30), Math.toRadians(0))
                .build();

        shot2 = bot.follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(118.000, 52.661),
                                new Pose(85.000, 85.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-30))
                .build();
        gate = bot.follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(85.000, 85.000),
                                new Pose(107.516, 55.233),
                                new Pose(134.000, 54.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-30), Math.toRadians(45))
                .build();

        gateToShoot = bot.follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(134.000, 54.000),
                                new Pose(107.419, 55.234),
                                new Pose(85.000, 85.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(-30))
                .build();

        intake2 = bot.follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(85.000, 85.000),
                                new Pose(124.386, 82.473)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-30), Math.toRadians(0))
                .build();
    }




    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                bot.follower.followPath(shot1);
                setPathState(1);
                break;
            case 1:
                if(!bot.follower.isBusy()) {
                    deliver(2);
                }
                break;
            case 2:

                if(!bot.follower.isBusy()) {
                    bot.intakePower(1);
                    if(wasBusy){
                        clock.reset();
                    }
                    bot.follower.followPath(intake1);
                    setPathState(3);
                }
                break;

            case 3:
                if(!bot.follower.isBusy()){
                    bot.intakePower(0);
                    if(wasBusy){
                        clock.reset();
                    }
                    if(clock.milliseconds() > 100){
                        bot.follower.followPath(shot2);
                        setPathState(4);
                    }
                }
                break;
            case 4:
                if(!bot.follower.isBusy()){
                    if(wasBusy){
                        clock.reset();
                    }
                    if(clock.milliseconds() > 250){
                        deliver(5);
                    }
                }
                break;
            case 5:
                if(!bot.follower.isBusy()){
                    bot.intakePower(1);
                    bot.follower.followPath(gate, 0.85,false);
                    setPathState(6);
                }
                break;
            case 6:
                if(!bot.follower.isBusy()) {
                    if(wasBusy){
                        clock.reset();
                    }
                    if(clock.milliseconds() > 500){
                        bot.intakePower(0);
                        setPathState(7);
                    }
                }
                break;
            case 7:
                if(!bot.follower.isBusy()) {

                    bot.follower.followPath(gateToShoot);
                    setPathState(8);
                }
                break;
            case 8:
                if(!bot.follower.isBusy()) {
                    if(wasBusy){
                        clock.reset();
                    }
                    if(clock.milliseconds() > 250){
                        deliver(9);
                    }
                }
                break;
            case 9:
                if(!bot.follower.isBusy()) {
                    bot.intakePower(1);
                    bot.follower.followPath(gate, 0.85, false);
                    setPathState(10);
                }
                break;
            case 10:
                if(!bot.follower.isBusy()) {
                    if(wasBusy){
                        clock.reset();
                    }
                    if(clock.milliseconds() > 500){
                        bot.intakePower(0);
                        setPathState(11);
                    }
                }
                break;
            case 11:
                if(!bot.follower.isBusy()) {

                    bot.follower.followPath(gateToShoot);
                    setPathState(12);
                }
                break;
            case 12:
                if(!bot.follower.isBusy()) {

                    if(wasBusy){
                        clock.reset();
                    }
                    if(clock.milliseconds() > 250){
                        deliver(13);
                    }
                }
                break;
            case 13:
                if(!bot.follower.isBusy()) {
                    bot.intakePower(1);
                    bot.follower.followPath(intake2);
                    setPathState(14);
                }
                break;
            case 14:
                if(!bot.follower.isBusy()) {
                    bot.intakePower(0);
                    setPathState(-1);
                }
                break;
            case 15:
                if(!bot.follower.isBusy()) {
                    if(wasBusy){
                        clock.reset();
                    }
                    if(clock.milliseconds() > 3000){
                        bot.intakePower(0);
                        setPathState(16);
                    }
                }
                break;
            case 17:
                if(!bot.follower.isBusy()) {
                    setPathState(-1);
                }
                break;
        }
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

        super.loop(); //Just runs telemetry
    }

    /**
     * This method is called once at the init of the OpMode.
     **/
    @Override
    public void init() {
        super.init();
        buildPaths();
        bot.follower.setStartingPose(startPose);
    }

    /**
     * This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system
     **/
    @Override
    public void start() {
        super.start();
    }

    /**
     * We do not use this because everything should automatically disable
     **/
    @Override
    public void stop() {
        super.stop();
    }

}