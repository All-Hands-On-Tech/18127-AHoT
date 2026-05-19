package org.firstinspires.ftc.teamcode.Auto; // make sure this aligns with class location

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.ZSupport.Utilities000;
@Disabled
@Autonomous(name = "Close Auto Red_NOSPIKE3", group = "A")
public class NOSPIKE3_CloseRedAuto extends OpMode {

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
    public PathChain HitGate2;
    public PathChain LeavePath1;
    public PathChain Spike3PreIntake;
    public PathChain Spike3Intake;
    public PathChain Spike3Return;

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
                                new Pose(125.006, 75),
                                new Pose(90, 83.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(-45))
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
        Path6 = bot.follower.pathBuilder() //OUT OF SEQUENCE (hit gate after path2
                .addPath(
                        new BezierLine(
                                new Pose(128.905, 84.489),
                                new Pose(115, 80)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-90))
                .build();

        HitGate2 = bot.follower.pathBuilder() //OUT OF SEQUENCE (hit gate after path2
                .addPath(
                        new BezierLine(
                                new Pose(115, 80),
                                new Pose(125.006, 75)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(-90))
                .build();

        LeavePath1 = bot.follower.pathBuilder() // VARIABLE SEQUENCE
                .addPath(
                        new BezierLine(
                                new Pose(86.522, 82.739),
                                new Pose(100, 82.739)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-45), Math.toRadians(-90))
                .build();

        Spike3PreIntake = bot.follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(90, 82.739),
                                new Pose(90, 34.611)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(0))
                .build();

        Spike3Intake = bot.follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(100, 34.611),
                                new Pose(133.977, 34.611)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        Spike3Return = bot.follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(133.977, 34.611),
                                new Pose(96.684, 40.695),
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
                    bot.intakePower(1);
                    if(wasBusy){
                        clock.reset();
                    }
                    if(clock.milliseconds() > 100){
                        bot.follower.followPath(Path6);
                        setPathState(4);
                    }
                }
                break;
            case 4:
                if(!bot.follower.isBusy()){
                    bot.follower.followPath(HitGate2);
                    setPathState(5);
                }
                break;
            case 5:
                if(!bot.follower.isBusy()){
                    bot.follower.followPath(Path3);
                    setPathState(6);
                }
                break;
            case 6:
                if(!bot.follower.isBusy()) {
                    if(wasBusy){
                        clock.reset();
                    }
                    if (clock.milliseconds() > 0 && clock.milliseconds() < 1000) {
                        bot.intakePower(-0.0);
                        bot.setTransferDown();
                    }
                    if (clock.milliseconds() > 1000 && clock.milliseconds() < 2000) {
                        bot.intakePower(1);
                        bot.setTransferDown();
                    } else if (clock.milliseconds() > 2000 && clock.milliseconds() < 2500) {
                        bot.setTransferUp();
                    } else if (clock.milliseconds() > 2500) {
                        bot.setTransferBlock();
                        setPathState(7);
                    }
                }
                break;
            case 7:
                if(!bot.follower.isBusy()) {

                    bot.follower.followPath(Path4,true);
                    setPathState(8);
                }
                break;
            case 8:
                if(!bot.follower.isBusy()) {
                    bot.intakePower(1);
                    bot.follower.followPath(Path5,true);
                    setPathState(9);
                }
                break;
            case 9:
                if(!bot.follower.isBusy()) {
                    if(wasBusy){
                        clock.reset();
                    }
                    if (clock.milliseconds() > 0 && clock.milliseconds() < 1000) {
                        bot.intakePower(-0.0);
                        bot.setTransferDown();
                    }
                    if (clock.milliseconds() > 1000 && clock.milliseconds() < 2000) {
                        bot.intakePower(1);
                        bot.setTransferDown();
                    } else if (clock.milliseconds() > 2000 && clock.milliseconds() < 2500) {
                        bot.setTransferUp();
                    } else if (clock.milliseconds() > 2500) {
                        bot.setTransferBlock();
                        setPathState(10);
                    }
                }
                break;
            case 10:
                if(!bot.follower.isBusy()) {
                    bot.follower.followPath(LeavePath1);
                    setPathState(17); //SKIP SPIKE 3
                }
                break;
            case 11:
                if(!bot.follower.isBusy()) {

                    bot.follower.followPath(Spike3PreIntake,true);
                    setPathState(12);
                }
                break;
            case 12:
                if(!bot.follower.isBusy()) {

                    bot.follower.followPath(Spike3Intake,true);
                    setPathState(13);
                }
                break;
            case 13:
                if(!bot.follower.isBusy()) {
                    bot.intakePower(1);
                    bot.follower.followPath(Spike3Return,true);
                    setPathState(14);
                }
                break;
            case 14:
                if(!bot.follower.isBusy()) {
                    if(wasBusy){
                        clock.reset();
                    }
                    if (clock.milliseconds() > 0 && clock.milliseconds() < 1000) {
                        bot.intakePower(-0.0);
                        bot.setTransferDown();
                    }
                    if (clock.milliseconds() > 1000 && clock.milliseconds() < 2000) {
                        bot.intakePower(1);
                        bot.setTransferDown();
                    } else if (clock.milliseconds() > 2000 && clock.milliseconds() < 2500) {
                        bot.setTransferUp();
                    } else if (clock.milliseconds() > 2500) {
                        bot.setTransferBlock();
                        setPathState(15);
                    }
                }
                break;
            case 15:
                if(!bot.follower.isBusy()) {
                    bot.follower.followPath(LeavePath1);
                    setPathState(16);
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
        bot.setAllianceColor(Utilities000.AllianceColor.RED);
        bot.setManualAimOffsets(10, 5);

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