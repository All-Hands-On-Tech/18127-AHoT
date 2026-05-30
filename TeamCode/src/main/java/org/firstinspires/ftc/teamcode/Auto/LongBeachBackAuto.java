package org.firstinspires.ftc.teamcode.Auto;

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.ZSupport.Utilities000;

@Autonomous(name = "Long Beach Back Auto", group = "A")
public class LongBeachBackAuto extends Auto000 {

    private PathChain Spike3Cycle;
    private PathChain Path1;
    private PathChain Path2;
    private PathChain LeftoverCycle;
    private PathChain FinalPath3;

    private Pose startPose;
    private boolean declogStarted = false;

    public void buildPaths() {
        if (alliance == Alliance.RED) {
            buildRedPaths();
        } else {
            buildBluePaths();
        }
    }
    private void buildRedPaths() {
        startPose = new Pose(88.5, 9.2, Math.toRadians(-90));
        // First: spike mark cycle
        Spike3Cycle = bot.follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(88.5, 9.2),
                                new Pose(94.512, 36.352)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(0))
                .addPath(
                        new BezierLine(
                                new Pose(94.512, 36.352),
                                new Pose(120, 36.352)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .addPath(
                        new BezierCurve(
                                new Pose(120, 36.352),
                                new Pose(93.700, 36.860),
                                new Pose(88.249, 17.352)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-30))
                .build();

        // Second: far pickup (only path at 0.8 speed)
        Path1 = bot.follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(88.249, 17.352),
                                new Pose(92.713, 28.316),
                                new Pose(137.074, 36.778),
                                new Pose(137.074, 9.669)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-30), Math.toRadians(-25))
                .build();

        // Return from far pickup
        Path2 = bot.follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(137.074, 9.669),
                                new Pose(137.074, 7),
                                new Pose(88.249, 17.201)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-25), Math.toRadians(-30))
                .build();

        // Leftover cycle
        LeftoverCycle = bot.follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(88.249, 17.201),
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
                                new Pose(88.099, 17.352)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(-90))
                .build();

        // Final park path
        FinalPath3 = bot.follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(88.099, 17.352),
                                new Pose(89.000, 30.000)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(-90))
                .build();
    }

    private void buildBluePaths() {
        startPose = new Pose(144-88.5, 9.2, Math.toRadians(-90));
        // First: spike mark cycle
        Spike3Cycle = bot.follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(144-88.5, 9.2),
                                new Pose(144-89.512, 36.352)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180-(-90)), Math.toRadians(180-(0)))
                .addPath(
                        new BezierLine(
                                new Pose(144-89.512, 36.352),
                                new Pose(144-120, 36.352)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180-(0)), Math.toRadians(180-(0)))
                .addPath(
                        new BezierCurve(
                                new Pose(144-120, 36.352),
                                new Pose(144-89.700, 36.860),
                                new Pose(144-89.512, 15.352)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180-(0)), Math.toRadians(180-(-30)))
                .build();

        // Second: far pickup (only path at 0.8 speed)
        Path1 = bot.follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(144-89.512, 15.352),
                                new Pose(144-89.713, 28.316),
                                new Pose(144-137.074, 36.778),
                                new Pose(144-137.074, 9.669)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180-(-30)), Math.toRadians(180-(-25)))
                .build();

        // Return from far pickup
        Path2 = bot.follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(144-137.074, 9.669),
                                new Pose(144-137.074, 7),
                                new Pose(144-89.512, 15.201)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180-(-25)), Math.toRadians(180-(-30)))
                .build();


        // Final park path
        FinalPath3 = bot.follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(144-89.512, 17.352),
                                new Pose(144-89.512, 30.000)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180-(-90)))
                .build();
    }

    void deliverBackZone(int nextState, double intakePower) {
        if (wasBusy) {
            clock.reset();
        }

        if (clock.milliseconds() > 0 && clock.milliseconds() < 1000) {
            bot.intakePower(1);
            bot.setTransferDown();
        }else if(clock.milliseconds() > 1000 && clock.milliseconds() < 1750) {
            bot.intakePower(intakePower);
            bot.setTransferUp();
        }else if (clock.milliseconds() > 1750 && clock.milliseconds() < 2250) {
            bot.intakePower(0);
        } else if (clock.milliseconds() > 2250 && clock.milliseconds() < 3000) {
            bot.intakePower(intakePower);
        }else if (clock.milliseconds() > 3000 && clock.milliseconds() < 4000) {
            bot.intakePower(1);
        } else if (clock.milliseconds() > 4000) {
            bot.setTransferDown();
            setPathState(nextState);
        }
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            // ===== FIRST SHOT (preloaded) =====
            case 0:
                if (clock.milliseconds() > 1400) {
                    deliverBackZone(1, 0.7);
                }
                break;

            // ===== Spike3Cycle at full speed =====
            case 1:
                if (!bot.follower.isBusy() && clock.milliseconds() > 2000) {
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
                    setPathState(4);
                    clock.reset();
                }
                break;

            // ===== SECOND SHOT =====
            case 4:
                if (!bot.follower.isBusy()) {
                    if (clock.milliseconds() > 400) {
                        deliverBackZone(5, 0.7);
                    }
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
                if (declogStarted && clock.milliseconds() > 750) {
                    bot.intakePower(1);
                    declogStarted = false;
                    setPathState(7);
                    clock.reset();
                }
                break;

            // ===== Path2 return at full speed =====
            case 7:
                if(bot.follower.isBusy())
                    clock.reset();
                if (!bot.follower.isBusy() && clock.milliseconds() > 500) {
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
                    deliverBackZone(10, 0.7);
                }
                break;

            // ===== LeftoverCycle at full speed =====
            case 10:
                if (!bot.follower.isBusy()) {
                    bot.follower.followPath(Path1, 0.8, false);
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
                if (declogStarted && clock.milliseconds() > 750) {
                    bot.intakePower(1);
                    declogStarted = false;
                    setPathState(67);
                    clock.reset();
                }
                break;
            case 67:
                if(bot.follower.isBusy())
                    clock.reset();
                if (!bot.follower.isBusy() && clock.milliseconds() > 500) {
                    bot.follower.followPath(Path2, true);
                    setPathState(12);
                }
                break;
            // ===== Wait for LeftoverCycle to finish =====
            case 12:
                if (!bot.follower.isBusy()) {
                    setPathState(13);
                    clock.reset();
                }
                break;

            // ===== FOURTH SHOT =====
            case 13:
                if (!bot.follower.isBusy()) {
                    deliverBackZone(14, 0.7);
                }
                break;

            // ===== FinalPath3 at full speed =====
            case 14:
                if (!bot.follower.isBusy()) {
                    bot.follower.followPath(FinalPath3, true);
                    clock.reset();
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
        if(bot.getAllianceColor() == Utilities000.AllianceColor.RED) {
            bot.setManualAimOffsets(3, 1);
        }else{
            bot.setManualAimOffsets(1, 1);
        }
        bot.follower.update();
        if (pathState != -1) {
            bot.turrentUpdateAutoBack();
        }
        autonomousPathUpdate();
        wasBusy = bot.follower.isBusy();
        super.loop();
    }

    @Override
    public void init() {
        super.init();
    }

    @Override
    public void init_loop() {
        if (gamepad1.b) {
            alliance = Alliance.RED;
        }
        if (gamepad1.x) {
            alliance = Alliance.BLUE;
        }

        telemetry.addLine("=== ALLIANCE SELECTION ===");
        telemetry.addLine("");
        telemetry.addData("Current Alliance", alliance.toString());
        telemetry.addLine("");
        telemetry.addLine("Press [B] for RED alliance");
        telemetry.addLine("Press [X] for BLUE alliance");
        telemetry.addLine("");
        telemetry.addLine("Press ▶ START when ready");
        telemetry.update();
    }

    @Override
    public void start() {
        super.start();
        buildPaths();
        bot.follower.setStartingPose(startPose);
    }

    @Override
    public void stop() {
        super.stop();
    }
}