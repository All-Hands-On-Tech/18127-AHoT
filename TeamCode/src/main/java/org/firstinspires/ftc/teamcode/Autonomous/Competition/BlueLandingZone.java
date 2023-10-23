package org.firstinspires.ftc.teamcode.Autonomous.Competition;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Autonomous.newThing;
import org.firstinspires.ftc.teamcode.RoadRunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.RoadRunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.RoboMom;
import org.firstinspires.ftc.teamcode.Vision.CircleDetectionPipeline;
import org.firstinspires.ftc.teamcode.Vision.VisionConstants;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;

@Autonomous(name="Blue Landing Zone", group="Blue")
public class BlueLandingZone extends RoboMom {

    //logan was here
    double fx = VisionConstants.fx;
    double fy = VisionConstants.fy;
    double cx = VisionConstants.cx;
    double cy = VisionConstants.cy;

    int RESWIDTH = VisionConstants.RESWIDTH;
    int RESHEIGHT = VisionConstants.RESHEIGHT;
    OpenCvCamera webcam;

    CircleDetectionPipeline circleDetectionPipeline = new CircleDetectionPipeline(telemetry);

    Pose2d startPose = new Pose2d(-60, -38, Math.toRadians(0));

    String spikePosition = "center";

    @Override
    public void runOpMode() {

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        webcam.setPipeline(circleDetectionPipeline);

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(startPose);

        TrajectorySequence right = drive.trajectorySequenceBuilder(startPose)
                .splineToConstantHeading(new Vector2d(-38, -47), Math.toRadians(0))
                .back(7)
                .strafeLeft(12.5)
                .forward(33)
                .lineToLinearHeading(new Pose2d(-8, 55, Math.toRadians(0)))
                .build();

        TrajectorySequence center = drive.trajectorySequenceBuilder(startPose)
                .splineTo(new Vector2d(-31, -35), Math.toRadians(0))
                .back(5)
                .strafeRight(15)
                .forward(25)
                .lineToLinearHeading(new Pose2d(-10, 55, Math.toRadians(0)))
                .build();

        TrajectorySequence left = drive.trajectorySequenceBuilder(startPose)
                .splineToLinearHeading(new Pose2d(-30, -34, Math.toRadians(90)), Math.toRadians(90))
                .back(5)
                .strafeRight(18)
                .lineToLinearHeading(new Pose2d(-12, 55, Math.toRadians(90)))
                .build();

        waitForStart();
        if (isStopRequested()) return;

        spikePosition = "CENTER"; //circleDetectionPipeline.getSpikePosition();
        switch (spikePosition) {
            case "LEFT":
                telemetry.addLine("left");
                telemetry.update();
                drive.followTrajectorySequence(left);
                break;
            case "CENTER":
                telemetry.addLine("center");
                telemetry.update();
                drive.followTrajectorySequence(center);
                break;
            case "RIGHT":
                telemetry.addLine("right");
                telemetry.update();
                drive.followTrajectorySequence(right);
                break;
        }

    }
}