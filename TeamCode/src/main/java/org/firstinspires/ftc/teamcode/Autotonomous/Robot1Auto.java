package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain; // added import

import org.firstinspires.ftc.teamcode.common.RobotHardware;
import org.firstinspires.ftc.teamcode.pathing.AutonomousPaths;
import org.firstinspires.ftc.teamcode.vision.ObeliskProcessor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.common.PedroConfig; // external config

@Autonomous(name = "Robot1 PedroPathing Auto", group = "Auto")
public class Robot1Auto extends LinearOpMode {
    private Follower follower;
    private AutonomousPaths autonomousPaths;
    private RobotHardware hw;

    @Override
    public void runOpMode() {
        // Initialize hardware and follower
        hw = new RobotHardware(hardwareMap);
        hw.initPinpoint(); // initialize Pinpoint (optional for pose reference)
        follower = PedroConfig.createFollower(hardwareMap); // external follower only

        // Set an initial pose (adjust to your real start in inches if library expects inches)
        try { follower.setPose(new Pose(0,0,0)); } catch (Exception ignored) {}

        // Create and build the paths
        autonomousPaths = new AutonomousPaths(follower, hw, this);
        autonomousPaths.buildPaths();

        // Initialize Obelisk processor
        ObeliskProcessor obeliskProcessor = new ObeliskProcessor(hardwareMap);

        ObeliskProcessor.DetectedColorPattern detectedPattern = ObeliskProcessor.DetectedColorPattern.NONE;

        telemetry.addLine("Scanning pattern...");
        telemetry.update();
        for (int i=0;i<20 && !isStopRequested();i++) {
            detectedPattern = obeliskProcessor.getDetectedPattern();
            if (detectedPattern != ObeliskProcessor.DetectedColorPattern.NONE) break;
            sleep(100);
        }
        telemetry.addData("Detected", detectedPattern);
        telemetry.addLine("Ready. Press PLAY.");
        telemetry.update();

        // Pre-start sync preview
        while (!isStarted() && !isStopRequested()) {
            hw.updatePinpoint();
            if (hw.pinpoint!=null && hw.pinpoint.getDeviceStatus()==org.firstinspires.ftc.teamcode.GoBildaPinpointDriver.DeviceStatus.READY) {
                double xIn = hw.pinpoint.getPosX(DistanceUnit.MM)/25.4;
                double yIn = hw.pinpoint.getPosY(DistanceUnit.MM)/25.4;
                double hRad = hw.pinpoint.getHeading(AngleUnit.RADIANS);
                telemetry.addData("Pinpoint", String.format("X%.1f Y%.1f H%.1f°",xIn,yIn,Math.toDegrees(hRad)));
                if (gamepad1.a) {
                    try { follower.setPose(new Pose(xIn,yIn,hRad)); } catch (Exception ignored) {}
                }
            } else {
                telemetry.addLine("Pinpoint not ready");
            }
            try { Pose p=follower.getPose(); telemetry.addData("FollowerPose", String.format("X%.1f Y%.1f H%.1f°",p.getX(),p.getY(),Math.toDegrees(p.getHeading()))); } catch (Exception ignored) {}
            telemetry.update();
        }

        waitForStart();
        if (isStopRequested()) return;

        if (hw.pinpoint!=null && hw.pinpoint.getDeviceStatus()==org.firstinspires.ftc.teamcode.GoBildaPinpointDriver.DeviceStatus.READY) {
            double xIn = hw.pinpoint.getPosX(DistanceUnit.MM)/25.4;
            double yIn = hw.pinpoint.getPosY(DistanceUnit.MM)/25.4;
            double hRad = hw.pinpoint.getHeading(AngleUnit.RADIANS);
            try { follower.setPose(new Pose(xIn,yIn,hRad)); } catch (Exception ignored) {}
        }

        PathChain pathToRun;
        switch (detectedPattern) {
            case GPP: pathToRun = autonomousPaths.gppPath; break;
            case PGP: pathToRun = autonomousPaths.pgpPath; break;
            case PPG: pathToRun = autonomousPaths.ppgPath; break;
            case NONE:
            default: pathToRun = autonomousPaths.nonePath; break;
        }

        telemetry.addData("Running Path", detectedPattern);
        telemetry.update();

        try { follower.followPath(pathToRun); } catch (Exception e) { telemetry.addData("followPath error", e.getMessage()); telemetry.update(); }

        while (opModeIsActive() && follower.isBusy()) {
            follower.update();
            try { Pose p=follower.getPose(); telemetry.addData("Pose", String.format("X%.1f Y%.1f H%.1f°",p.getX(),p.getY(),Math.toDegrees(p.getHeading()))); } catch (Exception ignored) {}
            telemetry.addData("Busy", true);
            telemetry.update();
        }

        telemetry.addLine("Done");
        telemetry.update();
        sleep(1000);
    }
}
