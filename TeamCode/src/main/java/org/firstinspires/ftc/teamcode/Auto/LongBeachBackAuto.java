package org.firstinspires.ftc.teamcode.Auto;

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Long Beach Back Auto", group = "A")
public class LongBeachBackAuto extends Auto000 {

    private PathChain Spike3Cycle;
    private PathChain Path1;
    private PathChain Path2;
    private PathChain LeftoverCycle;
    private PathChain FinalPath3;

    private final Pose startPose = new Pose(85.249, 5.949, Math.toRadians(0));

    private boolean declogStarted = false;

    private void buildRedPaths() {
        // First: spike mark cycle
        Spike3Cycle = bot.follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(85.249, 5.949),
                                new Pose(94.512, 36.352)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .addPath(
                        new BezierLine(
                                new Pose(94.512, 36.352),
                                new Pose(135.643, 37.540)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .addPath(
                        new BezierCurve(
                                new Pose(135.643, 37.540),
                                new Pose(93.700, 36.860),
                                new Pose(85.249, 17.352)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-30))
                .build();

        // Second: far pickup (only path at 0.8 speed)
        Path1 = bot.follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(85.249, 17.352),
                                new Pose(92.713, 28.316),
                                new Pose(133.239, 36.778),
                                new Pose(135.074, 9.669)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-30), Math.toRadians(-40))
                .build();

        // Return from far pickup
        Path2 = bot.follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(135.074, 9.669),
                                new Pose(112.612, 14.109),
                                new Pose(85.249, 17.201)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-40), Math.toRadians(-90))
                .build();

        // Leftover cycle
        LeftoverCycle = bot.follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(85.249, 17.201),
                                new Pose(89.000, 30.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(0))
                .addPath(
                        new BezierLine(
                                new Pose(89.000, 30.000),
                                new Pose(129.000, 40.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-60))
                .addPath(
                        new BezierLine(
                                new Pose(129.000, 40.000),
                                new Pose(131.000, 15.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-60), Math.toRadians(-90))
                .addPath(
                        new BezierLine(
                                new Pose(131.000, 15.000),
                                new Pose(85.099, 17.352)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(-90))
                .build();

        // Final park path
        FinalPath3 = bot.follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(85.099, 17.352),
                                new Pose(89.000, 30.000)
                        )
                )
                .setTangentHeadingInterpolation()
                .build();
    }

    public void buildPaths() {
        buildRedPaths();
        bot.follower.setStartingPose(startPose);
    }

    void deliverBackZone(int nextState, double intakePower) {
        if (wasBusy) {
            clock.reset();
        }

        if (clock.milliseconds() > 0 && clock.milliseconds() < 1500) {

        } else if (clock.milliseconds() > 1500 && clock.milliseconds() < 4000) {
            bot.intakePower(intakePower);
            bot.setTransferUp();
        } else if (clock.milliseconds() > 4000) {
            bot.setTransferDown();
            setPathState(nextState);
        }
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            // ===== FIRST SHOT (preloaded) =====
            case 0:
                if (clock.milliseconds() > 650) {
                    deliverBackZone(1, 0.7);
                }
                break;

            // ===== Spike3Cycle at full speed =====
            case 1:
                if (!bot.follower.isBusy()) {
                    bot.follower.followPath(Spike3Cycle, true);
                    clock.reset();
                    declogStarted = false;
                    setPathState(2);
                }
                break;

            // ===== Declog while moving along Spike3Cycle =====
            case 2:
                if (!declogStarted && clock.milliseconds() > 300) {
                    bot.intakePower(-1);
                    clock.reset();
                    declogStarted = true;
                }
                if (declogStarted && clock.milliseconds() > 200) {
                    bot.intakePower(1);
                    declogStarted = false;
                    setPathState(3);
                }
                break;

            // ===== Wait for Spike3Cycle to finish =====
            case 3:
                if (!bot.follower.isBusy()) {
                    bot.intakePower(0);
                    setPathState(4);
                    clock.reset();
                }
                break;

            // ===== SECOND SHOT =====
            case 4:
                if (!bot.follower.isBusy()) {
                    deliverBackZone(5, 0.8);
                }
                break;

            // ===== Path1 at 0.8 speed (far scoop) =====
            case 5:
                if (!bot.follower.isBusy()) {
                    bot.follower.followPath(Path1, 0.8, false);
                    clock.reset();
                    declogStarted = false;
                    setPathState(6);
                }
                break;

            // ===== Declog while moving along Path1 =====
            case 6:
                if (!declogStarted && clock.milliseconds() > 300) {
                    bot.intakePower(-1);
                    clock.reset();
                    declogStarted = true;
                }
                if (declogStarted && clock.milliseconds() > 200) {
                    bot.intakePower(1);
                    declogStarted = false;
                    setPathState(7);
                }
                break;

            // ===== Path2 return at full speed =====
            case 7:
                if (!bot.follower.isBusy()) {
                    bot.follower.followPath(Path2, true);
                    setPathState(8);
                }
                break;

            // ===== Wait for Path2 to finish =====
            case 8:
                if (!bot.follower.isBusy()) {
                    bot.intakePower(0);
                    setPathState(9);
                    clock.reset();
                }
                break;

            // ===== THIRD SHOT =====
            case 9:
                if (!bot.follower.isBusy()) {
                    deliverBackZone(10, 0.8);
                }
                break;

            // ===== LeftoverCycle at full speed =====
            case 10:
                if (!bot.follower.isBusy()) {
                    bot.follower.followPath(LeftoverCycle, true);
                    clock.reset();
                    declogStarted = false;
                    setPathState(11);
                }
                break;

            // ===== Declog while moving along LeftoverCycle =====
            case 11:
                if (!declogStarted && clock.milliseconds() > 300) {
                    bot.intakePower(-1);
                    clock.reset();
                    declogStarted = true;
                }
                if (declogStarted && clock.milliseconds() > 200) {
                    bot.intakePower(1);
                    declogStarted = false;
                    setPathState(12);
                }
                break;

            // ===== Wait for LeftoverCycle to finish =====
            case 12:
                if (!bot.follower.isBusy()) {
                    bot.intakePower(0);
                    setPathState(13);
                    clock.reset();
                }
                break;

            // ===== FOURTH SHOT =====
            case 13:
                if (!bot.follower.isBusy()) {
                    deliverBackZone(14, 0.8);
                }
                break;

            // ===== FinalPath3 at full speed =====
            case 14:
                if (!bot.follower.isBusy()) {
                    bot.follower.followPath(FinalPath3, true);
                    clock.reset();
                    declogStarted = false;
                    setPathState(15);
                }
                break;

            // ===== Declog while moving to final position =====
            case 15:
                if (!declogStarted && clock.milliseconds() > 300) {
                    bot.intakePower(-1);
                    clock.reset();
                    declogStarted = true;
                }
                if (declogStarted && clock.milliseconds() > 200) {
                    bot.intakePower(0);
                    declogStarted = false;
                    setPathState(-1);
                }
                break;

            case -1:
                if (!bot.follower.isBusy()) {
                    bot.setFlywheelVolts(0);
                    bot.setHoodYawPower(0);
                    bot.intakePower(0);
                    bot.setTransferBlock();
                }
                break;
        }
    }

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

    @Override
    public void init() {
        super.init();
        buildPaths();
    }

    @Override
    public void init_loop() {
        super.init_loop();
        telemetry.addData("Alliance", "Red");
        telemetry.update();
    }

    @Override
    public void start() {
        super.start();
    }

    @Override
    public void stop() {
        super.stop();
    }
}