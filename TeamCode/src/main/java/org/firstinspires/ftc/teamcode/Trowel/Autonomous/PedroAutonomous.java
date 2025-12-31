package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.bylazar.telemetry.PanelsTelemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;
import com.pedropathing.geometry.Pose;

@Autonomous(name = "Pedro Pathing Autonomous", group = "Autonomous")
@Configurable // Panels
public class PedroAutonomous extends OpMode {
    private TelemetryManager panelsTelemetry; // Panels Telemetry instance
    public Follower follower; // Pedro Pathing follower instance
    private int pathState; // Current autonomous path state (state machine)
    private Paths paths; // Paths defined in the Paths class

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
    private static final int STATE_DONE = 10;

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(72, 8, Math.toRadians(90)));

        paths = new Paths(follower); // Build paths
        pathState = STATE_PATH_1;

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void loop() {
        follower.update(); // Update Pedro Pathing
        pathState = autonomousPathUpdate(); // Update autonomous state machine

        // Log values to Panels and Driver Station
        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", follower.getPose().getHeading());
        panelsTelemetry.update(telemetry);
    }

    public static class Paths {
        public PathChain Depo1; public PathChain IntakeStart; public PathChain IntakeEnd; public PathChain Depo2; public PathChain IntakeStart2; public PathChain IntakeEnd2; public PathChain Deposit3; public PathChain IntakeStart3; public PathChain IntakeEnd3; public PathChain Deposit4;

        public Paths(Follower follower) {
            Depo1 = follower.pathBuilder().addPath(
                    new BezierLine(
                            new Pose(117.570, 129.626),
                            new Pose(88.131, 89.664)
                    )
            ).setLinearHeadingInterpolation(Math.toRadians(35), Math.toRadians(45))
                    .build();

            IntakeStart = follower.pathBuilder().addPath(
                    new BezierLine(
                            new Pose(88.131, 89.664),
                            new Pose(102.411, 83.458)
                    )
            ).setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(180))
                    .build();

            IntakeEnd = follower.pathBuilder().addPath(
                    new BezierLine(
                            new Pose(102.411, 83.458),
                            new Pose(127.495, 83.523)
                    )
            ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            Depo2 = follower.pathBuilder().addPath(
                    new BezierLine(
                            new Pose(127.495, 83.523),
                            new Pose(88.664, 90.028)
                    )
            ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(45))
                    .build();

            IntakeStart2 = follower.pathBuilder().addPath(
                    new BezierLine(
                            new Pose(88.664, 90.028),
                            new Pose(103.065, 59.430)
                    )
            ).setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(180))
                    .build();

            IntakeEnd2 = follower.pathBuilder().addPath(
                    new BezierLine(
                            new Pose(103.065, 59.430),
                            new Pose(130.944, 59.308)
                    )
            ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            Deposit3 = follower.pathBuilder().addPath(
                    new BezierLine(
                            new Pose(130.944, 59.308),
                            new Pose(88.813, 89.598)
                    )
            ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(45))
                    .build();

            IntakeStart3 = follower.pathBuilder().addPath(
                    new BezierLine(
                            new Pose(88.813, 89.598),
                            new Pose(101.430, 35.355)
                    )
            ).setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(180))
                    .build();

            IntakeEnd3 = follower.pathBuilder().addPath(
                    new BezierLine(
                            new Pose(101.430, 35.355),
                            new Pose(131.916, 35.682)
                    )
            ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            Deposit4 = follower.pathBuilder().addPath(
                    new BezierLine(
                            new Pose(131.916, 35.682),
                            new Pose(88.841, 90.196)
                    )
            ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(45))
                    .build();
        }
    }

    public int autonomousPathUpdate() {
        switch (pathState) {
            case STATE_PATH_1:
                follower.followPath(paths.Depo1);
                pathState = STATE_PATH_2;
                break;
            case STATE_PATH_2:
                if (!follower.isBusy()) {
                    follower.followPath(paths.IntakeStart);
                    pathState = STATE_PATH_3;
                }
                break;
            case STATE_PATH_3:
                if (!follower.isBusy()) {
                    follower.followPath(paths.IntakeEnd);
                    pathState = STATE_PATH_4;
                }
                break;
            case STATE_PATH_4:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Depo2);
                    pathState = STATE_PATH_5;
                }
                break;
            case STATE_PATH_5:
                if (!follower.isBusy()) {
                    follower.followPath(paths.IntakeStart2);
                    pathState = STATE_PATH_6;
                }
                break;
            case STATE_PATH_6:
                if (!follower.isBusy()) {
                    follower.followPath(paths.IntakeEnd2);
                    pathState = STATE_PATH_7;
                }
                break;
            case STATE_PATH_7:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Deposit3);
                    pathState = STATE_PATH_8;
                }
                break;
            case STATE_PATH_8:
                if (!follower.isBusy()) {
                    follower.followPath(paths.IntakeStart3);
                    pathState = STATE_PATH_9;
                }
                break;
            case STATE_PATH_9:
                if (!follower.isBusy()) {
                    follower.followPath(paths.IntakeEnd3);
                    pathState = STATE_PATH_10;
                }
                break;
            case STATE_PATH_10:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Deposit4);
                    pathState = STATE_DONE;
                }
                break;
            case STATE_DONE:
            default:
                // Auto complete
                break;
        }

        return pathState;
    }
}
