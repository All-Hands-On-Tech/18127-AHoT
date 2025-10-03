package org.firstinspires.ftc.teamcode.Autotonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.common.Odometry;
import org.firstinspires.ftc.teamcode.common.RobotConfig;
import org.firstinspires.ftc.teamcode.common.RobotHardware;

@Autonomous(name = "Hold Position Test", group = "Test")
public class HoldPositionTest extends LinearOpMode {
    private RobotHardware hw;
    private RobotConfig config;
    private Odometry odometry;

    // Target pose (field frame)
    private double targetXmm, targetYmm, targetHeadingDeg;

    // Gains (tune on field)
    private static final double KP_TRANSLATION = 0.005; // power per mm (50mm -> 0.25 power)
    private static final double KP_HEADING_DEG = 0.02;   // power per degree (10deg -> 0.2 power)
    private static final double MAX_TRANS_POWER = 0.5;   // max drive power
    private static final double MAX_ROT_POWER = 0.5;     // max turn power
    private static final double DEAD_MM = 2.0;           // deadband in mm
    private static final double DEAD_DEG = 1.0;          // deadband in deg

    @Override
    public void runOpMode() {
        hw = new RobotHardware(hardwareMap);
        config = new RobotConfig();
        hw.initPinpoint();

        telemetry.addLine("HoldPositionTest: Init complete");
        telemetry.addData("Pinpoint", hw.pinpoint == null ? "NOT FOUND (expect '"+config.pinpointName+"')" : hw.pinpoint.getDeviceStatus());
        telemetry.addLine("A = set current pose as target; B = pause/stop hold");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        odometry = new Odometry(hw, hw.pinpoint);

        // Capture starting pose as target
        for (int i=0; i<5; i++) { // small settle
            if (hw.pinpoint!=null) hw.updatePinpoint();
            odometry.update();
            sleep(20);
        }
        Odometry.Position p0 = odometry.getPosition();
        targetXmm = p0.getXmm();
        targetYmm = p0.getYmm();
        targetHeadingDeg = p0.getHeadingDeg();

        boolean paused = false;

        while (opModeIsActive()) {
            if (hw.pinpoint!=null) hw.updatePinpoint();
            odometry.update();
            Odometry.Position pos = odometry.getPosition();

            // Operator controls
            if (gamepad1.a) { // reset target to current
                targetXmm = pos.getXmm();
                targetYmm = pos.getYmm();
                targetHeadingDeg = pos.getHeadingDeg();
            }
            if (gamepad1.b) { // pause hold
                paused = true;
            }
            if (gamepad1.x) { // resume hold
                paused = false;
            }

            double yCmd=0, xCmd=0, rCmd=0;
            if (!paused) {
                // Errors in field frame
                double dx = targetXmm - pos.getXmm();
                double dy = targetYmm - pos.getYmm();
                double errDeg = angleWrapDeg(targetHeadingDeg - pos.getHeadingDeg());

                // Deadbands
                if (Math.abs(dx) < DEAD_MM) dx = 0;
                if (Math.abs(dy) < DEAD_MM) dy = 0;
                if (Math.abs(errDeg) < DEAD_DEG) errDeg = 0;

                // Convert field (dx,dy) to robot-frame commands (xCmd,yCmd)
                double h = pos.getHeadingRad();
                // r = R(-h) * f  => x_r = dx*cos(h) + dy*sin(h); y_r = -dx*sin(h) + dy*cos(h)
                double x_r = dx * Math.cos(h) + dy * Math.sin(h);
                double y_r = -dx * Math.sin(h) + dy * Math.cos(h);

                // Proportional control
                xCmd = clamp(x_r * KP_TRANSLATION, -MAX_TRANS_POWER, MAX_TRANS_POWER);
                yCmd = clamp(y_r * KP_TRANSLATION, -MAX_TRANS_POWER, MAX_TRANS_POWER);
                rCmd = clamp(errDeg * KP_HEADING_DEG, -MAX_ROT_POWER, MAX_ROT_POWER);
            }

            // Mecanum mixing (same as TeleOp convention)
            double denom = Math.max(Math.abs(yCmd) + Math.abs(xCmd) + Math.abs(rCmd), 1.0);
            double fl = (yCmd + xCmd + rCmd) / denom;
            double bl = (yCmd - xCmd + rCmd) / denom;
            double fr = (yCmd - xCmd - rCmd) / denom;
            double br = (yCmd + xCmd - rCmd) / denom;
            hw.setDrivePowers(fl, fr, bl, br);

            telemetry.addLine("=== HOLD ===");
            telemetry.addData("Target (mm,deg)", "x=%.1f y=%.1f hd=%.1f", targetXmm, targetYmm, targetHeadingDeg);
            telemetry.addData("Pose    (mm,deg)", "x=%.1f y=%.1f hd=%.1f", pos.getXmm(), pos.getYmm(), pos.getHeadingDeg());
            telemetry.addData("Cmd (x,y,r)", "%.2f %.2f %.2f", xCmd, yCmd, rCmd);
            telemetry.addData("Paused", paused);
            telemetry.addLine("A: set target to here | B: pause | X: resume");
            telemetry.update();

            sleep(20);
        }

        // stop
        hw.setDrivePowers(0,0,0,0);
    }

    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }
    private static double angleWrapDeg(double a) {
        while (a > 180) a -= 360;
        while (a < -180) a += 360;
        return a;
    }
}

