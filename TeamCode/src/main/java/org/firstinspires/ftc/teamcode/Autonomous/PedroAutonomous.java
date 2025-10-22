package org.firstinspires.ftc.teamcode.Autonomous;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Simplified Autonomous", group = "Autonomous")
public class PedroAutonomous extends OpMode {

  private Follower follower;
  private PathChain path;
  private int pathState;

  @Override
  public void init() {
    // Initialize the follower using the constants file
    follower = Constants.createFollower(hardwareMap);

    // Set the starting pose of the robot (in inches)
    follower.setStartingPose(new Pose(72, 8, Math.toRadians(90)));

    // Build the path
    path = follower.pathBuilder()
        .addPath(
          new BezierLine(
            new Pose(56.0, 8.0),  // Start of path
            new Pose(56.0, 36.0)   // End of path (28 inches forward)
          )
        )
        .setConstantHeadingInterpolation(Math.toRadians(90)) // Keep heading constant
        .build();

    telemetry.addLine("Robot initialized and path built.");
    telemetry.update();
  }

  @Override
  public void start() {
    // Start the autonomous path state machine
    pathState = 0;
  }

  @Override
  public void loop() {
    // Update the follower's internal state
    follower.update();

    // Update the state machine
    autonomousPathUpdate();

    // Basic telemetry
    Pose currentPose = follower.getPose();
    telemetry.addData("X (in)", "%.2f", currentPose.getX());
    telemetry.addData("Y (in)", "%.2f", currentPose.getY());
    telemetry.addData("Heading (deg)", "%.2f", Math.toDegrees(currentPose.getHeading()));
    telemetry.addData("Path State", pathState);
    telemetry.addData("Is Busy", follower.isBusy());
    telemetry.update();
  }

  /**
   * A simple state machine to manage the autonomous sequence.
   */
  public void autonomousPathUpdate() {
    switch (pathState) {
      case 0:
        // Start following the defined path
        follower.followPath(path, true);
        pathState = 1;
        break;
      case 1:
        // Wait until the follower is no longer busy
        if (!follower.isBusy()) {
          pathState = 2;
        }
        break;
      case 2:
        // The path is complete. Stop the robot.
        follower.breakFollowing();
        // You can add more states here for subsequent actions
        break;
    }
  }
}
