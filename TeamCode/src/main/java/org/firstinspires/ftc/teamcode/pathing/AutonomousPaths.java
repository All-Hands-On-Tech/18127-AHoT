package org.firstinspires.ftc.teamcode.pathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.common.RobotHardware;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants; // restore internal constants

/**
 * This class holds all the path definitions for the autonomous period.
 * Create an instance of this class in your autonomous OpMode and call buildPaths()
 * to generate the paths. You can then access the public PathChain fields
 * to run the desired path.
 *
 * Modify the Pose coordinates in the buildPaths() method to tune your routes.
 */
public class AutonomousPaths {

    private final Follower follower;
    private final RobotHardware hw;
    private final LinearOpMode opMode;

    // Define Poses. These are in inches. (0,0) is bottom-left of the field.
    // You will need to tune these to your robot and starting position.
    private final Pose startPose = new Pose(0, 0, Math.toRadians(0));

    // Paths for different detected patterns
    public PathChain gppPath;
    public PathChain pgpPath;
    public PathChain ppgPath;
    public PathChain nonePath;

    /**
     * Constructor for AutonomousPaths.
     * @param follower The Follower object from your OpMode.
     * @param hw The RobotHardware instance from your OpMode.
     * @param opMode The LinearOpMode instance (this).
     */
    public AutonomousPaths(Follower follower, RobotHardware hw, LinearOpMode opMode) {
        this.follower = follower;
        this.hw = hw;
        this.opMode = opMode;
    }

    /**
     * Runs the intake motor for a specified duration.
     * This is a blocking method that will pause the autonomous sequence.
     * @param durationMs The time to run the intake for, in milliseconds.
     * @param power The power to run the intake at (e.g., 1.0 for full power).
     */
    public void runIntake(long durationMs, double power) {
        if (hw.intakeMotor != null && opMode.opModeIsActive()) {
            hw.intakeMotor.setPower(power);
            opMode.sleep(durationMs);
            hw.intakeMotor.setPower(0);
        }
    }

    public void buildPaths() {
        // IMPORTANT: The poses here are examples. You need to define the actual
        // coordinates for your autonomous paths in inches.

        // Path for Green, Purple, Purple (e.g., Left Position)
        // This path is built from the user's request, with intake calls at each transition.
        gppPath = follower.pathBuilder()
                .addPath(new Path(new BezierLine(new Pose(56.000, 8.000), new Pose(41.255, 35.606)), Constants.pathConstraints))
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(180))
                // Run intake after the first segment
                .addCallback(() -> true, () -> runIntake(1500, 1.0))
                // Second segment
                .addPath(new Path(new BezierLine(new Pose(41.255, 35.606), new Pose(32.847, 35.606)), Constants.pathConstraints))
                .setTangentHeadingInterpolation()
                // Run intake after the second segment
                .addCallback(() -> true, () -> runIntake(1500, 1.0))
                // Third segment
                .addPath(new Path(new BezierLine(new Pose(32.847, 35.606), new Pose(22.993, 35.474)), Constants.pathConstraints))
                .setTangentHeadingInterpolation()
                .build();

        // Path for Purple, Green, Purple (e.g., Center Position)
        // This path now uses a callback to run the intake mid-path.
        pgpPath = follower.pathBuilder()
                .addPath(new Path(new BezierLine(startPose, new Pose(12, 24, 0)), Constants.pathConstraints))
                // Add a callback to run the intake for 1.5s after reaching the end of the path
                .addCallback(() -> true, () -> runIntake(1500, 1.0))
                .build();

        // Path for Purple, Purple, Green (e.g., Right Position)
        ppgPath = follower.pathBuilder()
                .addPath(new Path(new BezierLine(startPose, new Pose(24, 24, 0)), Constants.pathConstraints))
                .build();

        // Default path if no pattern is detected
        nonePath = follower.pathBuilder()
                .addPath(new Path(new BezierLine(startPose, new Pose(12, 24, 0)), Constants.pathConstraints))
                .build();
    }
}
