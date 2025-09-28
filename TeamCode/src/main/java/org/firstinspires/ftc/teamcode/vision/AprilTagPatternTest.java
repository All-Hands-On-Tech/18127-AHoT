package org.firstinspires.ftc.teamcode.vision;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Simple TeleOp to verify AprilTag -> pattern mapping.
 * Tag IDs:
 * 21 -> GPP
 * 22 -> PGP
 * 23 -> PPG
 * Displays the pattern in telemetry continuously.
 */
@TeleOp(name = "AprilTag Pattern Test", group = "Test")
public class AprilTagPatternTest extends LinearOpMode {
    private ObeliskProcessor processor;

    @Override
    public void runOpMode() {
        processor = new ObeliskProcessor(hardwareMap);
        telemetry.addLine("AprilTag Pattern Test Init - waiting for start");
        telemetry.addLine("Show a tag 21/22/23 to see GPP/PGP/PPG");
        telemetry.update();

        // Init loop: allow user to confirm detection before pressing start
        while (!isStarted() && !isStopRequested()) {
            ObeliskProcessor.DetectedColorPattern pattern = processor.getDetectedPattern();
            telemetry.addData("Detected Pattern", pattern);
            telemetry.update();
            sleep(50);
        }

        waitForStart();
        if (isStopRequested()) return;

        // Active loop
        while (opModeIsActive()) {
            ObeliskProcessor.DetectedColorPattern pattern = processor.getDetectedPattern();
            telemetry.addData("Detected Pattern", pattern);
            telemetry.update();
            sleep(50); // modest update rate
        }
    }
}

