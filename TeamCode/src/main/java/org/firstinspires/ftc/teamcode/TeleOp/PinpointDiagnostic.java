package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.GoBildaPinpointDriver.DeviceStatus;
import org.firstinspires.ftc.teamcode.GoBildaPinpointDriver.EncoderDirection;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;


@TeleOp(name = "Pinpoint Diagnostic", group = "Diagnostics")
public class PinpointDiagnostic extends LinearOpMode {

    private GoBildaPinpointDriver pinpoint;
    private String nameUsed = null;
    private final String[] candidateNames = new String[]{"pinpoint", "odo", "goBildaPinpoint"};

    private boolean headingOnly = false;
    private double lastLoopTime = 0;
    private double yawScalar = 1.0; // tracked locally (device stores internally)

    private boolean prevA=false, prevB=false, prevX=false, prevY=false, prevUp=false, prevDown=false;
    private String lastAction = "(none)";

    private void tryInit() {
        for (String n : candidateNames) {
            try {
                pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, n);
                nameUsed = n;
                break;
            } catch (Exception ignored) {}
        }
        if (pinpoint == null) return;
        pinpoint.initialize();

        // Wait briefly for READY
        int attempts = 0;
        while (attempts < 40 && pinpoint.getDeviceStatus() != DeviceStatus.READY) {
            sleep(50);
            pinpoint.update();
            attempts++;
        }
        // Default: assume 4 bar pods; change if needed.
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(EncoderDirection.FORWARD, EncoderDirection.FORWARD);
        // Offsets left at 0; adjust if you know them (mm): pinpoint.setOffsets(xOffset,yOffset, DistanceUnit.MM);
        pinpoint.resetPosAndIMU();
        yawScalar = pinpoint.getYawScalar();
    }

    @Override
    public void runOpMode() {
        tryInit();

        telemetry.addLine("Pinpoint Diagnostic Ready");
        telemetry.addData("Found", nameUsed == null ? "NO" : nameUsed);
        telemetry.addData("Status", pinpoint == null ? "N/A" : pinpoint.getDeviceStatus());
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;
        lastLoopTime = getRuntime();

        while (opModeIsActive()) {
            // Handle button actions
            if (pinpoint != null) {
                boolean a=gamepad1.a, b=gamepad1.b, x=gamepad1.x, y=gamepad1.y, up=gamepad1.dpad_up, down=gamepad1.dpad_down;
                if (a && !prevA) { pinpoint.resetPosAndIMU(); lastAction = "A: resetPosAndIMU"; }
                if (b && !prevB) { pinpoint.recalibrateIMU(); lastAction = "B: recalibrateIMU"; }
                if (x && !prevX) { headingOnly = !headingOnly; lastAction = headingOnly?"X: heading-only ON":"X: heading-only OFF"; }
                if (y && !prevY) {
                    double h = pinpoint.getHeading(AngleUnit.RADIANS);
                    pinpoint.setPosX(0, DistanceUnit.MM);
                    pinpoint.setPosY(0, DistanceUnit.MM);
                    pinpoint.setHeading(h, AngleUnit.RADIANS);
                    lastAction = "Y: Zero XY keep heading";
                }
                if (up && !prevUp) { yawScalar *= 1.01; pinpoint.setYawScalar(yawScalar); lastAction = "DpadUp: YawScalar +1%"; }
                if (down && !prevDown) { yawScalar *= 0.99; pinpoint.setYawScalar(yawScalar); lastAction = "DpadDown: YawScalar -1%"; }
                prevA=a; prevB=b; prevX=x; prevY=y; prevUp=up; prevDown=down;

                if (headingOnly) {
                    pinpoint.update(GoBildaPinpointDriver.ReadData.ONLY_UPDATE_HEADING);
                } else {
                    pinpoint.update();
                }
            }

            double now = getRuntime();
            double dt = now - lastLoopTime;
            double revHz = dt > 0 ? 1.0 / dt : 0;
            lastLoopTime = now;

            telemetry.addLine("=== PINPOINT ===");
            if (pinpoint == null) {
                telemetry.addLine("NOT FOUND. Names tried: pinpoint, odo, goBildaPinpoint");
                telemetry.addLine("Add Pinpoint in RC config with one of those names.");
            } else {
                telemetry.addData("Name", nameUsed);
                telemetry.addData("Mode", headingOnly ? "HEADING ONLY" : "FULL");
                telemetry.addData("Status", pinpoint.getDeviceStatus());
                telemetry.addData("Enc", String.format("X%d Y%d", pinpoint.getEncoderX(), pinpoint.getEncoderY()));
                double px = pinpoint.getPosX(DistanceUnit.MM);
                double py = pinpoint.getPosY(DistanceUnit.MM);
                double ph = Math.toDegrees(pinpoint.getHeading(AngleUnit.RADIANS));
                telemetry.addData("Pos", String.format("X%.1f Y%.1f H%.1f°", px, py, ph));
                double vx = pinpoint.getVelX(DistanceUnit.MM);
                double vy = pinpoint.getVelY(DistanceUnit.MM);
                double vh = pinpoint.getHeadingVelocity(UnnormalizedAngleUnit.DEGREES);
                telemetry.addData("Vel", String.format("X%.1f Y%.1f H%.1f°/s", vx, vy, vh));
                telemetry.addData("Loop μs", pinpoint.getLoopTime());
                telemetry.addData("PP Hz", String.format("%.1f", pinpoint.getFrequency()));
                telemetry.addData("REV Hz", String.format("%.1f", revHz));
                telemetry.addData("YawScalar", String.format("%.4f", yawScalar));
            }
            telemetry.addLine("A:Reset+Cal  B:Cal  X:ToggleMode  Y:ZeroPos  DpadUp/Down:YawScalar +/-1% ");
            telemetry.addData("Last Action", lastAction);
            telemetry.update();
        }
    }
}
