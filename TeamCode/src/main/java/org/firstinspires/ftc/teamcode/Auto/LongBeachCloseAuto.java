package org.firstinspires.ftc.teamcode.Auto;

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Long Beach Close Auto (Gate)", group = "A")
public class LongBeachCloseAuto extends Auto000 {

    // Alliance selection
    private enum Alliance { RED, BLUE }
    private Alliance alliance = Alliance.RED;

    // Preload
    public PathChain preload;

    // Cycle 1: Far spike
    public PathChain farSpike;

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

        // ===== PRELOAD =====
        preload = bot.follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(115.579, 129.107),
                                new Pose(85.000, 85.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-143), Math.toRadians(-30))
                .build();

        // ===== CYCLE 1: Far spike =====
        farSpike = bot.follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(85.000, 85.000),
                                new Pose(116.255, 84.999)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-30), Math.toRadians(0))
                .addPath(
                        new BezierLine(
                                new Pose(116.255, 84.999),
                                new Pose(85.000, 85.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-30))
                .build();

        // ===== CYCLE 2: Close spike =====
        closeSpike = bot.follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(85.000, 85.000),
                                new Pose(79.885, 46.822),
                                new Pose(116.650, 49.286)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-30), Math.toRadians(0))
                .addPath(
                        new BezierLine(
                                new Pose(116.650, 49.286),
                                new Pose(85.000, 85.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-30))
                .build();

        // ===== CYCLE 3: Gate 1 =====
        gate1Approach = bot.follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(85.000, 85.000),
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
                                new Pose(120.807, 46.900),
                                new Pose(127.052, 47.897)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(45))
                .build();

        gate1Return = bot.follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(127.052, 47.897),
                                new Pose(107.821, 65.249),
                                new Pose(85.000, 85.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(-30))
                .build();

        // ===== CYCLE 4: Gate 2 =====
        gate2Approach = bot.follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(85.000, 85.000),
                                new Pose(104.879, 52.398),
                                new Pose(123.412, 53.118)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-30), Math.toRadians(45))
                .build();

        gate2Nudge = bot.follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(123.412, 53.118),
                                new Pose(120.996, 46.642),
                                new Pose(127.187, 47.897)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(45))
                .build();

        gate2Return = bot.follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(127.187, 47.897),
                                new Pose(107.113, 65.987),
                                new Pose(85.000, 85.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(-30))
                .build();

        // ===== PARK =====
        park = bot.follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(85.000, 85.000),
                                new Pose(86.130, 67.718)
                        )
                )
                .build();
    }

    private void buildBluePaths() {
        startPose = new Pose(25.921, 129.107, Math.toRadians(323));

        // ===== PRELOAD =====
        preload = bot.follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(25.921, 129.107),
                                new Pose(56.500, 85.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(323), Math.toRadians(210))
                .build();

        // ===== CYCLE 1: Far spike =====
        farSpike = bot.follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(56.500, 85.000),
                                new Pose(25.245, 84.999)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(210), Math.toRadians(180))
                .addPath(
                        new BezierLine(
                                new Pose(25.245, 84.999),
                                new Pose(56.500, 85.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(210))
                .build();

        // ===== CYCLE 2: Close spike =====
        closeSpike = bot.follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(56.500, 85.000),
                                new Pose(61.615, 46.822),
                                new Pose(26.740, 49.421)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(210), Math.toRadians(180))
                .addPath(
                        new BezierLine(
                                new Pose(26.740, 49.421),
                                new Pose(56.500, 85.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(210))
                .build();

        // ===== CYCLE 3: Gate 1 =====
        gate1Approach = bot.follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(56.500, 85.000),
                                new Pose(36.486, 51.992),
                                new Pose(18.247, 53.229)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(210), Math.toRadians(135))
                .build();

        gate1Nudge = bot.follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(18.247, 53.229),
                                new Pose(21.233, 45.145),
                                new Pose(18.093, 47.221)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(135))
                .build();

        gate1Return = bot.follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(18.093, 47.221),
                                new Pose(33.679, 65.249),
                                new Pose(56.500, 85.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(210))
                .build();

        // ===== CYCLE 4: Gate 2 =====
        gate2Approach = bot.follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(56.500, 85.000),
                                new Pose(36.621, 52.398),
                                new Pose(18.088, 53.118)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(210), Math.toRadians(135))
                .build();

        gate2Nudge = bot.follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(18.088, 53.118),
                                new Pose(21.044, 45.832),
                                new Pose(18.093, 46.816)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(135))
                .build();

        gate2Return = bot.follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(18.093, 46.816),
                                new Pose(34.387, 65.987),
                                new Pose(56.500, 85.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(210))
                .build();

        // ===== PARK =====
        park = bot.follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(56.500, 85.000),
                                new Pose(55.370, 67.718)
                        )
                )
                .build();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {

            // ==================== PRELOAD SHOT ====================
            case 0:
                bot.follower.followPath(preload);
                setPathState(1);
                break;

            case 1:
                if (!bot.follower.isBusy()) {
                    deliver(2);
                }
                break;

            // ==================== CYCLE 1: FAR SPIKE ====================
            case 2:
                if (!bot.follower.isBusy()) {
                    bot.intakePower(1);
                    bot.follower.followPath(farSpike);
                    setPathState(3);
                }
                break;

            case 3:
                if (!bot.follower.isBusy()) {
                    bot.intakePower(0);
                    if (wasBusy) {
                        clock.reset();
                    }
                    if (clock.milliseconds() > 150) {
                        deliver(4);
                    }
                }
                break;

            // ==================== CYCLE 2: CLOSE SPIKE ====================
            case 4:
                if (!bot.follower.isBusy()) {
                    bot.intakePower(1);
                    bot.follower.followPath(closeSpike);
                    setPathState(5);
                }
                break;

            case 5:
                if (!bot.follower.isBusy()) {
                    bot.intakePower(0);
                    if (wasBusy) {
                        clock.reset();
                    }
                    if (clock.milliseconds() > 150) {
                        deliver(6);
                    }
                }
                break;

            // ==================== CYCLE 3: GATE 1 ====================
            case 6:
                if (!bot.follower.isBusy()) {
                    bot.follower.followPath(gate1Approach, 0.85, false);
                    setPathState(7);
                }
                break;

            case 7:
                if (!bot.follower.isBusy()) {
                    bot.intakePower(1);
                    bot.follower.followPath(gate1Nudge, 0.5, false);
                    setPathState(8);
                }
                break;

            case 8:
                if (!bot.follower.isBusy()) {
                    if (wasBusy) {
                        clock.reset();
                    }
                    if (clock.milliseconds() > 700) {
                        bot.intakePower(0);
                        bot.follower.followPath(gate1Return);
                        setPathState(9);
                    }
                }
                break;

            case 9:
                if (!bot.follower.isBusy()) {
                    if (wasBusy) {
                        clock.reset();
                    }
                    if (clock.milliseconds() > 250) {
                        deliver(10);
                    }
                }
                break;

            // ==================== CYCLE 4: GATE 2 ====================
            case 10:
                if (!bot.follower.isBusy()) {
                    bot.follower.followPath(gate2Approach, 0.85, false);
                    setPathState(11);
                }
                break;

            case 11:
                if (!bot.follower.isBusy()) {
                    bot.intakePower(1);
                    bot.follower.followPath(gate2Nudge, 0.5, false);
                    setPathState(12);
                }
                break;

            case 12:
                if (!bot.follower.isBusy()) {
                    if (wasBusy) {
                        clock.reset();
                    }
                    if (clock.milliseconds() > 600) {
                        bot.intakePower(0);
                        bot.follower.followPath(gate2Return);
                        setPathState(13);
                    }
                }
                break;

            case 13:
                if (!bot.follower.isBusy()) {
                    if (wasBusy) {
                        clock.reset();
                    }
                    if (clock.milliseconds() > 250) {
                        deliver(14);
                    }
                }
                break;

            // ==================== PARK ====================
            case 14:
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
    }

    @Override
    public void init_loop() {
        // Alliance selection during init phase
        if (gamepad1.x) {
            alliance = Alliance.BLUE;
        }
        if (gamepad1.b) {
            alliance = Alliance.RED;
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
        // Build paths based on selected alliance when the match actually starts
        buildPaths();
        bot.follower.setStartingPose(startPose);
        super.start();
    }

    @Override
    public void stop() {
        super.stop();
    }
}