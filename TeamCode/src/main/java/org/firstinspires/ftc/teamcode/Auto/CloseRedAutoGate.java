package org.firstinspires.ftc.teamcode.Auto; // make sure this aligns with class location

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "Close Auto Red (Gate)", group = "A")
public class CloseRedAutoGate extends Auto000 {

    public PathChain pathChain11PreloadRed;
    public PathChain pathChain22FarSpikeGreen;
    public PathChain pathChain33CloseSpikeBlue;
    public PathChain pathChain44Gate1Orange;
    public PathChain pathChain55Gate2Magenta;
    public PathChain pathChain66ParkWhite;

    public Pose startPose = new Pose(114.634, 130.457, Math.toRadians(-143));

    private final ElapsedTime stateClock = new ElapsedTime();
    private int previousPathState = Integer.MIN_VALUE;

    private boolean finishedOrTimeout(double timeoutMs) {
        return !bot.follower.isBusy() || bot.follower.atParametricEnd() || stateClock.milliseconds() > timeoutMs;
    }

    private void followAndAdvance(PathChain chain, int nextState) {
        bot.follower.followPath(chain);
        setPathState(nextState);
    }


    public void buildPaths() {
        pathChain11PreloadRed = bot.follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(114.634, 130.457),
                                new Pose(85.000, 85.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-143), Math.toRadians(-30))
                .build();

        pathChain22FarSpikeGreen = bot.follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(85.000, 85.000),
                                new Pose(118.997, 82.888)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-30), Math.toRadians(0))
                .addPath(
                        new BezierLine(
                                new Pose(118.997, 82.888),
                                new Pose(85.000, 85.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-30))
                .build();

        pathChain33CloseSpikeBlue = bot.follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(85.000, 85.000),
                                new Pose(82.855, 57.759),
                                new Pose(114.131, 50.588)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-30), Math.toRadians(0))
                .addPath(
                        new BezierLine(
                                new Pose(114.131, 50.588),
                                new Pose(85.000, 85.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-30))
                .build();

        pathChain44Gate1Orange = bot.follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(85.000, 85.000),
                                new Pose(103.196, 48.482),
                                new Pose(132.380, 54.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-30), Math.toRadians(45))
                .addPath(
                        new BezierCurve(
                                new Pose(132.380, 54.000),
                                new Pose(103.369, 48.078),
                                new Pose(85.000, 85.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(-30))
                .build();

        pathChain55Gate2Magenta = bot.follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(85.000, 85.000),
                                new Pose(103.466, 48.347),
                                new Pose(132.515, 54.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-30), Math.toRadians(45))
                .addPath(
                        new BezierCurve(
                                new Pose(132.515, 54.000),
                                new Pose(103.774, 49.293),
                                new Pose(85.000, 85.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(-30))
                .build();

        pathChain66ParkWhite = bot.follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(85.000, 85.000),
                                new Pose(86.130, 67.718)
                        )
                )
                .build();
    }


    public void autonomousPathUpdate() {
        if (pathState != previousPathState) {
            stateClock.reset();
            previousPathState = pathState;
        }

        switch (pathState) {
            case 0:
                followAndAdvance(pathChain11PreloadRed, 1);
                break;
            case 1:
                if(finishedOrTimeout(5000)) {
                    deliver(2);
                }
                break;
            case 2:
                if(finishedOrTimeout(6000)) {
                    followAndAdvance(pathChain22FarSpikeGreen, 3);
                }
                break;
            case 3:
                if(finishedOrTimeout(5000)) {
                    bot.intakePower(0);
                    deliver(4);
                }
                break;
            case 4:
                if(finishedOrTimeout(6000)) {
                    followAndAdvance(pathChain33CloseSpikeBlue, 5);
                }
                break;
            case 5:
                if(finishedOrTimeout(5000)) {
                    bot.intakePower(0);
                    deliver(6);
                }
                break;
            case 6:
                if(finishedOrTimeout(7000)) {
                    followAndAdvance(pathChain44Gate1Orange, 7);
                }
                break;
            case 7:
                if(finishedOrTimeout(5000)) {
                    bot.intakePower(0);
                    deliver(8);
                }
                break;
            case 8:
                if(finishedOrTimeout(7000)) {
                    followAndAdvance(pathChain55Gate2Magenta, 9);
                }
                break;
            case 9:
                if(finishedOrTimeout(5000)) {
                    bot.intakePower(0);
                    deliver(10);
                }
                break;
            case 10:
                if(finishedOrTimeout(5000)) {
                    followAndAdvance(pathChain66ParkWhite, 11);
                }
                break;
            case 11:
                if(finishedOrTimeout(3000)) {
                    bot.intakePower(0);
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
        bot.follower.update();
        if (pathState != -1) {
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
