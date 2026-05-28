package org.firstinspires.ftc.teamcode.Auto;

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.ZSupport.Utilities000;

@Autonomous(name = "Long Beach Close Auto (Gate)", group = "A")
public class LongBeachCloseAuto extends Auto000 {

    // Preload
    public PathChain preload;

    // Cycle 1: Far spike
    public PathChain farSpike;
    public PathChain gate0Nudge;

    // Cycle 2: Close spike
    public PathChain closeSpike;


    // Cycle 3: Gate 1
    public PathChain gate1Approach;
    public PathChain gate1Nudge;
    public PathChain gate1Return;

    // Cycle 4: Gate 2
    public PathChain gate2Approach;
    public PathChain gate2Nudge;
    public PathChain gate2Return;

    // Park
    public PathChain park;

    public Pose startPose;

    public void buildPaths() {
        if (alliance == Alliance.RED) {
            buildRedPaths();
        } else {
            buildBluePaths();
        }
    }

    private void buildRedPaths() {
        startPose = new Pose(115.579, 129.107, Math.toRadians(-143));

        preload = bot.follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(115.579, 129.107),
                                new Pose(88.000, 85.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-143), Math.toRadians(-45))
                .build();

        farSpike = bot.follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(88.000, 85.000),
                                new Pose(116.255, 84.999)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-30), Math.toRadians(0))
                .addPath(
                        new BezierLine(
                                new Pose(116.255, 84.999),
                                new Pose(88.000, 85.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-45))
                .build();

        closeSpike = bot.follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(88.000, 85.000),
                                new Pose(79.885, 50),
                                new Pose(110, 50)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-30), Math.toRadians(0))
                .addPath(
                        new BezierCurve(
                                new Pose(110, 50),
                                new Pose(112, 65),
                                new Pose(116, 70),
                                new Pose(120, 70)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .addPath(
                        new BezierLine(
                                new Pose(120, 70),
                                new Pose(110, 70)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .addPath(
                        new BezierCurve(
                                new Pose(110, 65),
                                new Pose(90, 50),
                                new Pose(88.000, 85.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-45))
                .build();

        gate1Approach = bot.follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(88.000, 85.000),
                                new Pose(105.014, 51.992),
                                new Pose(123.253, 53.229)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-30), Math.toRadians(45))
                .build();

        gate1Nudge = bot.follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(123.253, 53.229),
                                new Pose(120.267, 45.145),
                                new Pose(129, 47.671)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(45))
                .build();

        gate1Return = bot.follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(130.157, 47.671),
                                new Pose(107.821, 65.249),
                                new Pose(88.000, 85.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(-45))
                .build();

        park = bot.follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(88.000, 85.000),
                                new Pose(115, 55)
                        )
                )
                .setReversed()
                .setTangentHeadingInterpolation()
                .build();
    }

    private void buildBluePaths() {
        startPose = new Pose(144-(115.579), 129.107, Math.toRadians(180-(-143)));

        preload = bot.follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(144-(115.579), 129.107),
                                new Pose(144-(88.000), 85.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180-(-143)), Math.toRadians(180-(-45)))
                .build();

        farSpike = bot.follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(144-(88.000), 85.000),
                                new Pose(144-(116.255), 84.999)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180-(-30)), Math.toRadians(180-(0)))
                .addPath(
                        new BezierLine(
                                new Pose(144-(116.255), 84.999),
                                new Pose(144-(88.000), 85.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180-(0)), Math.toRadians(180-(-45)))
                .build();

        closeSpike = bot.follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(144-(88.000), 85.000),
                                new Pose(144-(79.885), 50),
                                new Pose(144-(110), 50)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180-(-30)), Math.toRadians(180-(0)))
                .addPath(
                        new BezierCurve(
                                new Pose(144-(110), 50),
                                new Pose(144-(112), 65),
                                new Pose(144-(116), 70),
                                new Pose(144-(120), 70)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180-(0)), Math.toRadians(180-(0)))
                .addPath(
                        new BezierLine(
                                new Pose(144-(120), 70),
                                new Pose(144-(110), 70)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180-(0)), Math.toRadians(180-(0)))
                .addPath(
                        new BezierCurve(
                                new Pose(144-(110), 65),
                                new Pose(144-(90), 50),
                                new Pose(144-(88.000), 85.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180-(0)), Math.toRadians(180-(-45)))
                .build();

        gate1Approach = bot.follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(144-(88.000), 85.000),
                                new Pose(144-(105.014), 51.992),
                                new Pose(144-(123.253), 53.229)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180-(-30)), Math.toRadians(180-(45)))
                .build();

        gate1Nudge = bot.follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(144-(123.253), 53.229),
                                new Pose(144-(120.267), 45.145),
                                new Pose(144-(129), 47.671)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180-(45)), Math.toRadians(180-(45)))
                .build();

        gate1Return = bot.follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(144-(130.157), 47.671),
                                new Pose(144-(107.821), 65.249),
                                new Pose(144-(88.000), 85.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180-(45)), Math.toRadians(180-(-45)))
                .build();

        gate2Approach = bot.follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(88.000, 85.000),
                                new Pose(104.879, 52.398),
                                new Pose(130.412, 53.118)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180-(-30)), Math.toRadians(180-(45)))
                .build();

        park = bot.follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(144-(88.000), 85.000),
                                new Pose(144-(115), 55)
                        )
                )
                .setReversed()
                .setTangentHeadingInterpolation()
                .build();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {

            // ==================== PRELOAD SHOT ====================
            case 0:
                bot.follower.followPath(preload);
                setPathState(1);
                clock.reset();
                break;

            case 1:
                if (!bot.follower.isBusy()) {
                    if(clock.milliseconds() > 200) {
                        deliver(2);
                    }
                }
                break;

            // ==================== CYCLE 1: CLOSE SPIKE ====================
            case 2:
                if (!bot.follower.isBusy()) {
                    bot.intakePower(1);
                    bot.follower.followPath(closeSpike);
                    setPathState(3);
                }
                break;
            case 3:
                if (!bot.follower.isBusy()) {
                    if (wasBusy) {
                        clock.reset();
                    }
                    if (clock.milliseconds() > 200) {
                        deliver(4);
                    }
                }
                break;


            // ==================== CYCLE 2: GATE 1 ====================
            case 4:
                if (!bot.follower.isBusy()) {
                    bot.follower.followPath(gate1Approach, 0.85, false);
                    setPathState(5);
                }
                break;

            case 5:
                if (!bot.follower.isBusy()) {
                    bot.intakePower(1);
                    bot.follower.followPath(gate1Nudge, 1, false);

                    setPathState(6);
                }
                break;

            case 6:
                if (!bot.follower.isBusy()) {
                    if (wasBusy) {
                        clock.reset();
                    }
                    if (clock.milliseconds() > 1000) {
                        bot.follower.followPath(gate1Return);
                        setPathState(7);
                    }
                }
                break;

            case 7:
                if (!bot.follower.isBusy()) {
                    if (wasBusy) {
                        clock.reset();
                    }
                    if (clock.milliseconds() > 200) {
                        deliver(8);
                    }
                }
                break;

            // ==================== CYCLE 3: GATE 2 ====================
            case 8:
                if (!bot.follower.isBusy()) {
                    bot.follower.followPath(gate1Approach, 0.85, false);
                    setPathState(9);
                }
                break;

            case 9:
                if (!bot.follower.isBusy()) {
                    bot.intakePower(1);
                    bot.follower.followPath(gate1Nudge, 1, false);
                    setPathState(10);
                }
                break;

            case 10:
                if (!bot.follower.isBusy()) {
                    if (wasBusy) {
                        clock.reset();
                    }
                    if (clock.milliseconds() > 1000) {
                        bot.follower.followPath(gate1Return);
                        setPathState(11);
                    }
                }
                break;

            case 11:
                if (!bot.follower.isBusy()) {
                    if (wasBusy) {
                        clock.reset();
                    }
                    if (clock.milliseconds() > 200) {
                        deliver(12);
                    }
                }
                break;

            // ==================== CYCLE 4: FAR SPIKE ====================
            case 12:
                if (!bot.follower.isBusy()) {
                    bot.intakePower(1);
                    bot.follower.followPath(farSpike);
                    setPathState(13);
                }
                break;

            case 13:
                if (!bot.follower.isBusy()) {
                    if (wasBusy) {
                        clock.reset();
                    }
                    if(clock.milliseconds() > 200) {
                        deliver(14);
                    }
                }
                break;

            // ==================== PARK ====================
            case 14:
                if (wasBusy) {
                    clock.reset();
                }
                if (!bot.follower.isBusy()) {
                    bot.follower.followPath(park);
                    setPathState(15);
                }
                break;

            case 15:
                if (!bot.follower.isBusy()) {
                    setPathState(-1);
                }
                break;
        }
    }

    @Override
    public void loop() {
        if(bot.getAllianceColor() == Utilities000.AllianceColor.RED){
            bot.setManualAimOffsets(3,-0.5);
        }else{
            bot.setManualAimOffsets(3,-1);
        }
        bot.follower.update();

        if (pathState != -1) {
            bot.turrentUpdateAuto();
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
        buildPaths();
        bot.follower.setStartingPose(startPose);
        super.start();
    }

    @Override
    public void stop() {
        super.stop();
    }
}