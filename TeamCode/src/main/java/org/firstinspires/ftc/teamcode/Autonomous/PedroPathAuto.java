package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.FollowerConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.paths.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.paths.PathConstraints;
import org.firstinspires.ftc.teamcode.pedroPathing.geometry.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.MecanumDrivetrain;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.PinpointLocalizer;
import org.firstinspires.ftc.teamcode.pedroPathing.paths.PathBuilder;
import org.firstinspires.ftc.teamcode.pedroPathing.geometry.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.geometry.BezierCurve;
import org.firstinspires.ftc.teamcode.common.RobotHardware;

@Autonomous(name = "PedroPathAuto", group = "Auto")
public class PedroPathAuto extends LinearOpMode {
    @Override
    public void runOpMode() {
        // Initialize robot hardware
        RobotHardware hw = new RobotHardware(hardwareMap);
        hw.initPinpoint();

        // Create Pedro pathing components using our hardware
        PinpointLocalizer localizer = new PinpointLocalizer(hw);
        MecanumDrivetrain drivetrain = new MecanumDrivetrain(hw);

        FollowerConstants followerConstants = new FollowerConstants();
        PathConstraints pathConstraints = PathConstraints.defaultConstraints;
        Follower follower = new Follower(followerConstants, localizer, drivetrain, pathConstraints);

        // --- Path definition (inline for easier editing) ---
        PathBuilder builder = new PathBuilder(follower);
        PathChain pathChain = builder
            .addPath(
                new BezierLine(new Pose(53.138, 6.899), new Pose(14.826, 9.101))
            )
            .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(90))
            .addPath(
                new BezierCurve(
                    new Pose(14.826, 9.101),
                    new Pose(53.431, 2.202),
                    new Pose(63.853, 16.294)
                )
            )
            .setTangentHeadingInterpolation()
            .addPath(
                new BezierCurve(
                    new Pose(63.853, 16.294),
                    new Pose(12.330, 1.615),
                    new Pose(14.239, 7.486)
                )
            )
            .setTangentHeadingInterpolation()
            .addPath(
                new BezierCurve(
                    new Pose(14.239, 7.486),
                    new Pose(60.330, 3.963),
                    new Pose(64.881, 15.119)
                )
            )
            .setTangentHeadingInterpolation()
            .addPath(
                new BezierCurve(
                    new Pose(64.881, 15.119),
                    new Pose(10.275, 2.349),
                    new Pose(16.734, 10.128)
                )
            )
            .setTangentHeadingInterpolation()
            .addPath(
                new BezierCurve(
                    new Pose(16.734, 10.128),
                    new Pose(66.202, 4.257),
                    new Pose(65.908, 17.174)
                )
            )
            .setTangentHeadingInterpolation()
            .build();

        Pose startPose = new Pose(53.138, 6.899, Math.toRadians(90));
        follower.setStartingPose(startPose);

        telemetry.addLine("Ready to run PedroPathAuto");
        telemetry.addData("Pinpoint Status", hw.pinpoint != null ? hw.pinpoint.getDeviceStatus().name() : "NOT FOUND");
        telemetry.addData("Start Pose", String.format("(%.1f, %.1f) @ %.0f°",
            startPose.getX(), startPose.getY(), Math.toDegrees(startPose.getHeading())));
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        // Start following the path
        follower.followPath(pathChain);

        while (opModeIsActive() && follower.isBusy()) {
            follower.update();
            localizer.update();

            Pose currentPose = follower.getPose();

            // === PINPOINT STATUS ===
            telemetry.addLine("=== PINPOINT ODOMETRY ===");
            if (hw.pinpoint != null) {
                telemetry.addData("Status", hw.pinpoint.getDeviceStatus().name());
                telemetry.addData("X Position (in)", String.format("%.2f", currentPose.getX()));
                telemetry.addData("Y Position (in)", String.format("%.2f", currentPose.getY()));
                telemetry.addData("Heading (deg)", String.format("%.1f", Math.toDegrees(currentPose.getHeading())));

                // Get velocities from Pinpoint
                try {
                    double velX = hw.pinpoint.getVelX();
                    double velY = hw.pinpoint.getVelY();
                    telemetry.addData("Velocity X (in/s)", String.format("%.2f", velX));
                    telemetry.addData("Velocity Y (in/s)", String.format("%.2f", velY));
                } catch (Exception e) {
                    telemetry.addData("Velocity", "N/A");
                }
            } else {
                telemetry.addData("Pinpoint", "NOT FOUND");
            }

            // === PATH STATUS ===
            telemetry.addLine();
            telemetry.addLine("=== PATH TRACKING ===");
            telemetry.addData("Path Status", "FOLLOWING");
            telemetry.addData("isBusy", follower.isBusy() ? "YES" : "NO");

            // === MOTOR POWERS (for debugging) ===
            telemetry.addLine();
            telemetry.addLine("=== DRIVE MOTORS ===");
            if (hw.frontLeft != null) telemetry.addData("Front Left", String.format("%.2f", hw.frontLeft.getPower()));
            if (hw.frontRight != null) telemetry.addData("Front Right", String.format("%.2f", hw.frontRight.getPower()));
            if (hw.backLeft != null) telemetry.addData("Back Left", String.format("%.2f", hw.backLeft.getPower()));
            if (hw.backRight != null) telemetry.addData("Back Right", String.format("%.2f", hw.backRight.getPower()));

            telemetry.update();
        }

        // Stop the robot
        hw.setDrivePowers(0, 0, 0, 0);

        telemetry.addLine("======================");
        telemetry.addLine("PATH COMPLETE!");
        telemetry.addLine("======================");
        telemetry.update();
        sleep(2000);
    }
}
