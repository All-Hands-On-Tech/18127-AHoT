package org.firstinspires.ftc.teamcode.Unused;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import java.util.Locale;
import org.firstinspires.ftc.teamcode.common.RobotHardware;
import org.firstinspires.ftc.teamcode.common.RobotConfig;
import org.firstinspires.ftc.teamcode.common.Odometry;
import org.firstinspires.ftc.teamcode.common.PanelsPublisher;

@TeleOp(name = "Two Wheel Odometry TeleOp", group = "TeleOp")
public class Robot1Tele extends LinearOpMode {
    private RobotHardware hw;
    private RobotConfig config;
    private Odometry odometry;
    private PanelsPublisher panels;
    private double lastLoopTime = 0;

    // Cam control (hold-based, step-per-loop)
    private double camPosition = 0.5; // logical position tracker; servo not moved until pressed
    private static final double CAM_STEP = 0.01; // base step per 20ms loop
    private boolean camFirstUse = true; // lazy-init from servo reading on first use

    // Launcher smoothing (X button)
    private double launcherPowerCurrent = 0.0;
    private static final double LAUNCHER_RAMP_PER_SEC = 3.0; // power units per second toward target

    // Safe-combo edges
    private boolean prevComboReset = false; // start+back edge
    private boolean prevComboRecal = false; // x+y edge

    @Override
    public void runOpMode() {
        hw = new RobotHardware(hardwareMap);
        config = new RobotConfig();
        hw.initPinpoint();
        panels = new PanelsPublisher();
        panels.init();

        telemetry.addLine("Init complete - waiting start");
        telemetry.addData("Pinpoint", hw.pinpoint == null ? "NOT FOUND (expect name '"+config.pinpointName+"')" : hw.pinpoint.getDeviceStatus());
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;
        lastLoopTime = getRuntime();

        odometry = new Odometry(hw, hw.pinpoint);

        while (opModeIsActive()) {
            // Timebase
            double now = getRuntime();
            double dt = Math.max(0.0, Math.min(0.1, now - lastLoopTime)); // clamp dt to avoid spikes

            // Sensors
            if (hw.pinpoint != null) hw.updatePinpoint();
            if (odometry != null) odometry.update();
            Odometry.Position pos = odometry != null ? odometry.getPosition() : null;

            // Drive (both sticks)
            double y = Math.abs(gamepad1.left_stick_y) > 0.05 ? gamepad1.left_stick_y : 0;
            double x = Math.abs(gamepad1.left_stick_x) > 0.05 ? -gamepad1.left_stick_x : 0; // invert to match previous behavior
            double r = Math.abs(gamepad1.right_stick_x) > 0.05 ? -gamepad1.right_stick_x : 0; // invert to match previous behavior
            y = Math.copySign(y*y*y, y);
            x = Math.copySign(x*x*x, x) * 1.1;
            r = Math.copySign(r*r*r, r);
            double denom = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(r), 1);
            double fl = (y + x + r) / denom;
            double bl = (y - x + r) / denom;
            double fr = (y - x - r) / denom;
            double br = (y + x - r) / denom;
            hw.setDrivePowers(fl, fr, bl, br);

            // Intake (triggers, both => stop)
            boolean rt = gamepad1.right_trigger > 0.1;
            boolean lt = gamepad1.left_trigger > 0.1;
            double intakePower = (rt && lt) ? 0.0 : rt ? 0.9 : lt ? -0.9 : 0.0;
            if (hw.intakeMotor != null) hw.intakeMotor.setPower(intakePower);

            // Launcher (X) smooth ramp
            double launcherTarget = gamepad1.x ? 1.0 : 0.0;
            double rampStep = LAUNCHER_RAMP_PER_SEC * dt;
            if (launcherPowerCurrent < launcherTarget) launcherPowerCurrent = Math.min(launcherTarget, launcherPowerCurrent + rampStep);
            else if (launcherPowerCurrent > launcherTarget) launcherPowerCurrent = Math.max(launcherTarget, launcherPowerCurrent - rampStep);
            if (hw.depositMotorL != null) hw.depositMotorL.setPower(launcherPowerCurrent);
            if (hw.depositMotorR != null) hw.depositMotorR.setPower(launcherPowerCurrent);

            // Cam (LB/RB) hold-based step
            if (hw.cam != null) {
                double step = CAM_STEP * Math.max(0.25, Math.min(4.0, dt / 0.02)); // light time-normalization
                if (gamepad1.left_bumper ^ gamepad1.right_bumper) { // exactly one held
                    if (camFirstUse) { try { camPosition = hw.cam.getPosition(); } catch (Exception ignored) {} camFirstUse = false; }
                    if (gamepad1.left_bumper) camPosition -= step; else camPosition += step;
                    camPosition = Math.max(0.0, Math.min(1.0, camPosition));
                    hw.cam.setPosition(camPosition);
                }
            }

            // Safe combos (pinpoint reset / recal)
            boolean comboReset = gamepad1.start && gamepad1.back;
            if (comboReset && !prevComboReset) {
                if (hw.pinpoint != null) hw.pinpoint.resetPosAndIMU();
                else if (hw.imu != null) hw.imu.resetYaw();
            }
            prevComboReset = comboReset;
            boolean comboRecal = gamepad1.x && gamepad1.y;
            if (comboRecal && !prevComboRecal && hw.pinpoint != null) hw.pinpoint.recalibrateIMU();
            prevComboRecal = comboRecal;

            // Telemetry
            telemetry.addLine("=== INPUT ===");
            telemetry.addData("LB (Cam In)", gamepad1.left_bumper);
            telemetry.addData("RB (Cam Out)", gamepad1.right_bumper);
            telemetry.addData("X (Launcher)", gamepad1.x);
            telemetry.addData("Start+Back Reset", comboReset);
            telemetry.addData("X+Y Recal", comboRecal);
            telemetry.addData("RT / LT", String.format(Locale.US, "%.2f / %.2f", gamepad1.right_trigger, gamepad1.left_trigger));

            telemetry.addLine("=== DRIVE ===");
            telemetry.addData("Joy", "Y%.2f X%.2f R%.2f", y, x, r);
            telemetry.addData("Pow", "FL%.2f FR%.2f BL%.2f BR%.2f", fl, fr, bl, br);
            if (pos != null) telemetry.addData("Odo", pos.toString());

            telemetry.addLine("=== SYSTEMS ===");
            telemetry.addData("Intake", intakePower);
            telemetry.addData("Launcher target/current", "%.2f / %.2f", launcherTarget, launcherPowerCurrent);
            telemetry.addData("Cam pos", String.format(Locale.US, "%.3f", camPosition));
            telemetry.addData("PP Stat", hw.pinpoint != null ? hw.pinpoint.getDeviceStatus() : "NONE");

            double dtDbg = now - lastLoopTime; lastLoopTime = now;
            telemetry.addData("Loop Hz", dtDbg > 0 ? String.format(Locale.US, "%.1f", 1.0 / dtDbg) : "-");
            telemetry.update();

            // Panels publishing (inches)
            if (pos != null) {
                double xIn = pos.getXmm() / 25.4;
                double yIn = pos.getYmm() / 25.4;
                panels.putText("teleop/pinpoint/status", hw.pinpoint != null ? hw.pinpoint.getDeviceStatus().name() : "NONE");
                panels.publishPose("teleop/pose", xIn, yIn, pos.getHeadingDeg(), 50);
            }
        }
    }
}
