package org.firstinspires.ftc.teamcode.Autonomous.Checks;

import android.util.Size;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.RoadRunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.RoboMom;
import org.firstinspires.ftc.teamcode.Vision.CircleDetectionPipeline;
import org.firstinspires.ftc.teamcode.Vision.VisionConstants;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;

import java.util.List;

@Autonomous(name="AprilTagComparison", group="Z")
public class AprilTagComparison extends RoboMom {
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;
    enum State {
        TRAJECTORY_1,
        TRAJECTORY_2,
        IDLE
    }

    State currentState = State.IDLE;
    Pose2d startPose = new Pose2d(24, 120, Math.toRadians(90));
    Pose2d targetPose = new Pose2d(36, 120, Math.toRadians(90));

    @Override
    public void runOpMode() {
        super.runOpMode();
        initAprilTag();

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(startPose);

        Trajectory traj1 = drive.trajectoryBuilder(startPose)
                .lineToSplineHeading(targetPose)
                .build();

        waitForStart();
        if (isStopRequested()) return;

        currentState = State.TRAJECTORY_1;
        drive.followTrajectoryAsync(traj1);

        while (opModeIsActive() && !isStopRequested()) {
            List<AprilTagDetection> currentDetections = aprilTag.getDetections();

            int numberOfDetections = currentDetections.size();
            double AprilTagX = 0;
            double AprilTagY = 0;
            double AprilTagAngle = 0;

            telemetry.addData("# AprilTags Detected", numberOfDetections);

            // Step through the list of detections and display info for each one.
            for (AprilTagDetection detection : currentDetections) {
                Pose2d location = AbsolutePositionFromAprilTag(detection);
                AprilTagX += location.getX();
                AprilTagY += location.getY();
                AprilTagAngle += location.getHeading();
            }

            AprilTagX /= numberOfDetections;
            AprilTagY /= numberOfDetections;
            AprilTagAngle /= numberOfDetections;

            Pose2d poseEstimateAprilTag = new Pose2d(AprilTagX,AprilTagY, AprilTagAngle);
            Pose2d poseEstimateRoadRunner = drive.getPoseEstimate();

            switch (currentState) {
                case TRAJECTORY_1:
                    if (!drive.isBusy()) {
                        currentState = State.TRAJECTORY_2;
                    }
                    break;
                case TRAJECTORY_2:
                    drive.setPoseEstimate(poseEstimateAprilTag);
                    traj1 = drive.trajectoryBuilder(startPose)
                            .lineToSplineHeading(targetPose)
                            .build();

                    currentState = State.TRAJECTORY_1;
                    drive.followTrajectoryAsync(traj1);
                    break;
                case IDLE:
                    //stops here
                    break;
            }

            drive.update();

            // Print pose to telemetry
            telemetry.addLine("RR   AT");
            telemetry.addLine(String.format("X %6.1f %6.1f  (inch)", poseEstimateRoadRunner.getX(), poseEstimateAprilTag.getX()));
            telemetry.addLine(String.format("Y %6.1f %6.1f  (inch)", poseEstimateRoadRunner.getY(), poseEstimateAprilTag.getY()));
            telemetry.addLine(String.format("Heading %6.1f %6.1f  (Degree)", poseEstimateRoadRunner.getHeading(), poseEstimateAprilTag.getHeading()));
            telemetry.update();
        }
    }

    private void initAprilTag() {

        // Create the AprilTag processor.
        aprilTag = new AprilTagProcessor.Builder()
                //.setDrawAxes(false)
                //.setDrawCubeProjection(false)
                //.setDrawTagOutline(true)
                //.setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                //.setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())
                //.setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)

                // == CAMERA CALIBRATION ==
                // If you do not manually specify calibration parameters, the SDK will attempt
                // to load a predefined calibration for your camera.
                //.setLensIntrinsics(578.272, 578.272, 402.145, 221.506)
                .setLensIntrinsics(1473.69, 1473.69, 845.856, 514.97)

                // ... these parameters are fx, fy, cx, cy.

                .build();

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));

        // Choose a camera resolution. Not all cameras support all resolutions.
        builder.setCameraResolution(new Size(1920, 1080));

        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
        //builder.enableCameraMonitoring(true);

        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
        //builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);

        // Choose whether or not LiveView stops if no processors are enabled.
        // If set "true", monitor shows solid orange screen if no processors enabled.
        // If set "false", monitor shows camera view without annotations.
        //builder.setAutoStopLiveView(false);

        // Set and enable the processor.
        builder.addProcessor(aprilTag);

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();

        // Disable or re-enable the aprilTag processor at any time.
        //visionPortal.setProcessorEnabled(aprilTag, true);

    }
}