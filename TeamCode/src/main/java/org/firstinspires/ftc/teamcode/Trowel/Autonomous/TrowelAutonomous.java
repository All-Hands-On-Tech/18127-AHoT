package org.firstinspires.ftc.teamcode.Trowel.Autonomous;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode.Trowel.pedroPathing.Constants;

@Autonomous(name = "Trowel Auto", group = "Autonomous")
public class TrowelAutonomous extends OpMode {

    private Follower follower;
    private int pathState;
    private Paths paths;

    // State machine constants
    private static final int STATE_PATH_1 = 0;
    private static final int STATE_PATH_2 = 1;
    private static final int STATE_PATH_3 = 2;
    private static final int STATE_PATH_4 = 3;
    private static final int STATE_PATH_5 = 4;
    private static final int STATE_PATH_6 = 5;
    private static final int STATE_PATH_7 = 6;
    private static final int STATE_PATH_8 = 7;
    private static final int STATE_PATH_9 = 8;
    private static final int STATE_PATH_10 = 9;
    private static final int STATE_PATH_11 = 10;
    private static final int STATE_PATH_12 = 11;
    private static final int STATE_PATH_13 = 12;
    private static final int STATE_PATH_14 = 13;
    private static final int STATE_DONE = 14;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(117.366, 130.244, Math.toRadians(35)));
        paths = new Paths(follower);
        pathState = STATE_PATH_1;

        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    @Override
    public void loop() {
        follower.update();

        switch (pathState) {
            case STATE_PATH_1:
                follower.followPath(paths.Path1);
                pathState = STATE_PATH_2;
                break;
            case STATE_PATH_2:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path2);
                    pathState = STATE_PATH_3;
                }
                break;
            case STATE_PATH_3:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path3);
                    pathState = STATE_PATH_4;
                }
                break;
            case STATE_PATH_4:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path4);
                    pathState = STATE_PATH_5;
                }
                break;
            case STATE_PATH_5:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path5);
                    pathState = STATE_PATH_6;
                }
                break;
            case STATE_PATH_6:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path6);
                    pathState = STATE_PATH_7;
                }
                break;
            case STATE_PATH_7:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path7);
                    pathState = STATE_PATH_8;
                }
                break;
            case STATE_PATH_8:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path8);
                    pathState = STATE_PATH_9;
                }
                break;
            case STATE_PATH_9:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path9);
                    pathState = STATE_PATH_10;
                }
                break;
            case STATE_PATH_10:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path10);
                    pathState = STATE_PATH_11;
                }
                break;
            case STATE_PATH_11:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path11);
                    pathState = STATE_PATH_12;
                }
                break;
            case STATE_PATH_12:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path12);
                    pathState = STATE_PATH_13;
                }
                break;
            case STATE_PATH_13:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path13);
                    pathState = STATE_PATH_14;
                }
                break;
            case STATE_PATH_14:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path14);
                    pathState = STATE_DONE;
                }
                break;
            case STATE_DONE:
                // Auto complete
                break;
        }

        telemetry.addData("Path State", pathState);
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.update();
    }

    public static class Paths {
        public PathChain Path1, Path2, Path3, Path4, Path5, Path6, Path7;
        public PathChain Path8, Path9, Path10, Path11, Path12, Path13, Path14;

        public Paths(Follower follower) {
            Path1 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(117.366, 130.244),
                                    new Pose(87.951, 89.415)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(35), Math.toRadians(45))
                    .build();

            Path2 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(87.951, 89.415),
                                    new Pose(103.317, 71.707)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(90))
                    .build();

            Path3 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(103.317, 71.707),
                                    new Pose(125.854, 71.707)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(90))
                    .build();

            Path4 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(125.854, 71.707),
                                    new Pose(66.000, 71.000),
                                    new Pose(94.976, 83.561)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(90))
                    .build();

            Path5 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(94.976, 83.561),
                                    new Pose(102.146, 84.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(180))
                    .build();

            Path6 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(102.146, 84.000),
                                    new Pose(127.610, 84.146)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            Path7 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(127.610, 84.146),
                                    new Pose(88.244, 89.268)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(45))
                    .build();

            Path8 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(88.244, 89.268),
                                    new Pose(102.146, 58.683)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(180))
                    .build();

            Path9 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(102.146, 58.683),
                                    new Pose(132.439, 58.390)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            Path10 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(132.439, 58.390),
                                    new Pose(121.463, 65.122),
                                    new Pose(88.098, 89.268)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(45))
                    .build();

            Path11 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(88.098, 89.268),
                                    new Pose(102.439, 35.854)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(180))
                    .build();

            Path12 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(102.439, 35.854),
                                    new Pose(134.049, 35.707)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            Path13 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(134.049, 35.707),
                                    new Pose(88.098, 89.122)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(45))
                    .build();

            Path14 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(88.098, 89.122),
                                    new Pose(124.683, 70.537)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(90))
                    .build();
        }
    }
}