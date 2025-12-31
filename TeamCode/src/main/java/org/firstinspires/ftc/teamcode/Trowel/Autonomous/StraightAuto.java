package org.firstinspires.ftc.teamcode.Trowel.Autonomous;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.Trowel.pedroPathing.Constants;

@Autonomous(name = "Straight Auto", group = "Autonomous")
@Configurable
public class StraightAuto extends OpMode {

    private TelemetryManager panelsTelemetry;
    public Follower follower;
    private int pathState = 0;
    private Paths paths;
    private ElapsedTime timer;

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        follower = Constants.createFollower(hardwareMap);
        // Starting pose matches the first point of the provided Path (x=56, y=8) heading 90deg
        follower.setStartingPose(new Pose(117.57009345794395, 129.62616822429908, Math.toRadians(35)));
        paths = new Paths(follower);
        timer = new ElapsedTime();

        // Initialize drive motors and add telemetry for their power multipliers
        Constants.setMotorPowerMultiplier("LEFT_FRONT", Constants.LEFT_FRONT_POWER);
        Constants.setMotorPowerMultiplier("LEFT_REAR", Constants.LEFT_REAR_POWER);
        Constants.setMotorPowerMultiplier("RIGHT_FRONT", Constants.RIGHT_FRONT_POWER);
        Constants.setMotorPowerMultiplier("RIGHT_REAR", Constants.RIGHT_REAR_POWER);

        panelsTelemetry.debug("Motor Power Multipliers", "LF: " + Constants.LEFT_FRONT_POWER + ", LR: " + Constants.LEFT_REAR_POWER + ", RF: " + Constants.RIGHT_FRONT_POWER + ", RR: " + Constants.RIGHT_REAR_POWER);

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.debug("StartPose X", follower.getPose().getX());
        panelsTelemetry.debug("StartPose Y", follower.getPose().getY());
        panelsTelemetry.debug("StartPose Heading", follower.getPose().getHeading());
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void loop() {
        follower.update();
        pathState = autonomousPathUpdate();

        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", follower.getPose().getHeading());
        panelsTelemetry.update(telemetry);
    }

    public static class Paths {
        public PathChain Depo1;
        public PathChain IntakeStart;
        public PathChain IntakeEnd;
        public PathChain Depo2;
        public PathChain IntakeStart2;
        public PathChain IntakeEnd2;
        public PathChain Deposit3;
        public PathChain IntakeStart3;
        public PathChain IntakeEnd3;
        public PathChain Deposit4;

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

    // Update the state machine to follow each path in sequence
    public int autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(paths.Depo1);
                pathState = 1;
                panelsTelemetry.debug("Transition", "Started Depo1");
                break;
            case 1:
                if (!follower.isBusy()) {
                    follower.followPath(paths.IntakeStart);
                    pathState = 2;
                    panelsTelemetry.debug("Transition", "Started IntakeStart");
                }
                break;
            case 2:
                if (!follower.isBusy()) {
                    follower.followPath(paths.IntakeEnd);
                    pathState = 3;
                    panelsTelemetry.debug("Transition", "Started IntakeEnd");
                }
                break;
            case 3:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Depo2);
                    pathState = 4;
                    panelsTelemetry.debug("Transition", "Started Depo2");
                }
                break;
            case 4:
                if (!follower.isBusy()) {
                    follower.followPath(paths.IntakeStart2);
                    pathState = 5;
                    panelsTelemetry.debug("Transition", "Started IntakeStart2");
                }
                break;
            case 5:
                if (!follower.isBusy()) {
                    follower.followPath(paths.IntakeEnd2);
                    pathState = 6;
                    panelsTelemetry.debug("Transition", "Started IntakeEnd2");
                }
                break;
            case 6:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Deposit3);
                    pathState = 7;
                    panelsTelemetry.debug("Transition", "Started Deposit3");
                }
                break;
            case 7:
                if (!follower.isBusy()) {
                    follower.followPath(paths.IntakeStart3);
                    pathState = 8;
                    panelsTelemetry.debug("Transition", "Started IntakeStart3");
                }
                break;
            case 8:
                if (!follower.isBusy()) {
                    follower.followPath(paths.IntakeEnd3);
                    pathState = 9;
                    panelsTelemetry.debug("Transition", "Started IntakeEnd3");
                }
                break;
            case 9:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Deposit4);
                    pathState = 10;
                    panelsTelemetry.debug("Transition", "Started Deposit4");
                }
                break;
            case 10:
                if (!follower.isBusy()) {
                    pathState = 11; // finished
                    panelsTelemetry.debug("Transition", "Finished Deposit4");
                }
                break;
            case 11:
                // finished - keep idle
                break;
        }
        return pathState;
    }
}