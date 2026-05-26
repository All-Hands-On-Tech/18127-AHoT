package org.firstinspires.ftc.teamcode.Auto; // make sure this aligns with class location

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.ZSupport.Utilities000;

@Autonomous(name = "Back Auto Red", group = "A")
public class BackRedAuto extends Auto000 {
    public PathChain Path1;
    public PathChain Path2;
    public PathChain Spike3PreIntake;
    public PathChain Spike3Intake;
    public PathChain Spike3Return;
    public PathChain Path3;
    public PathChain LeftoverPreIntake;
    public PathChain LeftoverIntake;
    public PathChain LeftoverReturn;
    public PathChain FinalPath3;

    public Pose startPose = new Pose(89, 10, Math.toRadians(-90));


    public void buildPaths() {
        Path1 = bot.follower.pathBuilder().addPath(
                        new BezierCurve(
                                startPose,
                                new Pose(97.980, 14.346),
                                new Pose(133.239, 36.778),
                                new Pose(134.805, 9.683)
                        )
                ).setTangentHeadingInterpolation()
                .build();

        Path2 = bot.follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(134.805, 9.683),
                                new Pose(89, 13)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(-90))
                .build();

        Spike3PreIntake = bot.follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(89, 13),
                                new Pose(101.114, 38.122)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(0))
                .build();

        Spike3Intake = bot.follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(101.114, 38.122),
                                new Pose(128.952, 39.475)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        Spike3Return = bot.follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(128.952, 39.475),
                                new Pose(93.700, 36.860),
                                new Pose(89, 13)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-90))
                .build();

        Path3 = bot.follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(89, 13),
                                new Pose(88.865, 32.970)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(0))
                .build();

        LeftoverPreIntake = bot.follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(88.865, 32.970),
                                new Pose(129, 40)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-60))
                .build();

        LeftoverIntake = bot.follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(129, 40),
                                new Pose(128.840, 15.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-60), Math.toRadians(-90))
                .build();

        LeftoverReturn = bot.follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(128.840, 15.000),
                                new Pose(89, 13)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(-90))
                .build();

        FinalPath3 = bot.follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(89, 13),
                                new Pose(88.865, 32.700)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(0))
                .build();
    }




    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                if(clock.milliseconds() > 650) {
                    deliverBackZone(1);
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
                if(!bot.follower.isBusy()) {
                    deliverBackZone(5);
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
                    deliverBackZone(9);
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
                    deliverBackZone(14);
                }
                break;
            case 14:
                if(!bot.follower.isBusy()){
                    bot.follower.followPath(FinalPath3);
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

        super.loop();
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