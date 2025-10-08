package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.common.RobotHardware;
import org.firstinspires.ftc.teamcode.common.RobotConfig;
import org.firstinspires.ftc.teamcode.common.Odometry;
import org.firstinspires.ftc.teamcode.common.PanelsPublisher;

@TeleOp(name = "Two Wheel Odometry TeleOp FieldCentric", group = "TeleOp")
public class Robot1TeleFieldCentric extends LinearOpMode {
    private RobotHardware hw;
    private RobotConfig config;
    private Servo sweeperServo;
    // Cam control (hold-based, step-per-loop)
    private double camPosition = 0.4;
    private static final double CAM_STEP = 0.003;
    // Sweeper control
    private double sweeperPosition = 0.5;
    private static final double SWEEPER_STEP = 0.02;
    // 2600 RPM = 2600/60 = 43.33 rev/s × 28 ticks/rev = 1213 ticks/s (for REV HD Hex motors)
    private static final double MAX_LAUNCHER_VELOCITY = 2440.0; // ticks per second for ~2600 RPM

    @Override
    public void runOpMode() {
        hw = new RobotHardware(hardwareMap);
        config = new RobotConfig();
        hw.initPinpoint();
        PanelsPublisher panels = new PanelsPublisher();
        panels.init();
        sweeperServo = hardwareMap.get(Servo.class, "sweeperServo");

        telemetry.addLine("Init complete - waiting start");
        telemetry.addLine("DRIVER 1: Chassis control");
        telemetry.addLine("DRIVER 2: Mechanisms (intake, cam, sweeper, deposit)");
        telemetry.addData("Pinpoint", hw.pinpoint == null ? "NOT FOUND (expect name '"+config.pinpointName+"')" : hw.pinpoint.getDeviceStatus());
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;
        double lastLoopTime = getRuntime();

        Odometry odometry = new Odometry(hw, hw.pinpoint);

        // Set initial servo positions
        if (hw.cam != null) hw.cam.setPosition(camPosition);
        if (sweeperServo != null) sweeperServo.setPosition(sweeperPosition);

        boolean prevComboReset = false;
        boolean prevComboRecal = false;

        while (opModeIsActive()) {
            double now = getRuntime();
            double dt = Math.max(0.0, Math.min(0.1, now - lastLoopTime));
            lastLoopTime = now;

            hw.updatePinpoint();
            odometry.update();
            Odometry.Position pos = odometry.getPosition();

            // ===== DRIVER 1: CHASSIS CONTROL =====
            // Get joystick values - REVERSED Y to make deposit the front
            double y = Math.abs(gamepad1.left_stick_y) > 0.05 ? -gamepad1.left_stick_y : 0; // NEGATED for reversed front
            double x = Math.abs(gamepad1.left_stick_x) > 0.05 ? gamepad1.left_stick_x : 0; // REMOVED negation to invert strafe
            double r = Math.abs(gamepad1.right_stick_x) > 0.05 ? -gamepad1.right_stick_x : 0;
            double speedMul = gamepad1.left_bumper ? 0.5 : 1.0;
            y*=speedMul; x*=speedMul; r*=speedMul; x*=1.1;
            y=Math.copySign(y*y*y,y); x=Math.copySign(x*x*x,x); r=Math.copySign(r*r*r,r);

            // Field-centric transformation
            double headingRad = pos.getHeadingRad();
            double tempX = x * Math.cos(-headingRad) - y * Math.sin(-headingRad);
            double tempY = x * Math.sin(-headingRad) + y * Math.cos(-headingRad);
            x = tempX;
            y = tempY;

            double denominator = Math.max(Math.abs(y)+Math.abs(x)+Math.abs(r),1); // Fix typo: denom -> denominator
            double fl=(y+x+r)/denominator, bl=(y - x + r)/denominator, fr=(y - x - r)/denominator, br=(y + x - r)/denominator;
            hw.setDrivePowers(fl,fr,bl,br);

            // Driver 1: Safe combos (pinpoint reset / recalibrate)
            boolean comboReset = gamepad1.start && gamepad1.back;
            if (comboReset && !prevComboReset) {
                if (hw.pinpoint != null) hw.pinpoint.resetPosAndIMU();
                else if (hw.imu != null) hw.imu.resetYaw();
            }
            prevComboReset = comboReset;

            boolean comboRecal = gamepad1.x && gamepad1.y;
            if (comboRecal && !prevComboRecal && hw.pinpoint != null) hw.pinpoint.recalibrateIMU();
            prevComboRecal = comboRecal;

            // ===== DRIVER 2: MECHANISM CONTROL =====
            // Intake (triggers, both => stop)
            boolean rt = gamepad2.right_trigger > 0.1;
            boolean lt = gamepad2.left_trigger > 0.1;
            double intakePower = (rt && lt) ? 0.0 : rt ? 0.9 : lt ? -0.9 : 0.0;
            if (hw.intakeMotor != null) hw.intakeMotor.setPower(intakePower);

            // Launcher (X) velocity control, opposite directions
            // TEST: Set velocity to a very high value to check RPM calculation
            double launcherVelocityTarget = gamepad2.x ? 10000.0 : 0.0;
            if (hw.depositMotorL != null) hw.depositMotorL.setVelocity(launcherVelocityTarget);
            if (hw.depositMotorR != null) hw.depositMotorR.setVelocity(-launcherVelocityTarget);

            // Cam (LB/RB) hold-based step
            if (hw.cam != null) {
                boolean camMoved = false;
                if (gamepad2.left_bumper) {
                    camPosition -= CAM_STEP;
                    camMoved = true;
                }
                if (gamepad2.right_bumper) {
                    camPosition += CAM_STEP;
                    camMoved = true;
                }
                if (camMoved) {
                    camPosition = Math.max(0.0, Math.min(1.0, camPosition));
                    hw.cam.setPosition(camPosition);
                }
            }

            // Sweeper (A/B) hold-based step
            if (sweeperServo != null) {
                boolean sweeperMoved = false;
                if (gamepad2.a) {
                    sweeperPosition -= SWEEPER_STEP;
                    sweeperMoved = true;
                }
                if (gamepad2.b) {
                    sweeperPosition += SWEEPER_STEP;
                    sweeperMoved = true;
                }
                if (sweeperMoved) {
                    sweeperPosition = Math.max(0.0, Math.min(1.0, sweeperPosition));
                    sweeperServo.setPosition(sweeperPosition);
                }
            }

            // Telemetry - each stat on its own line
            telemetry.addData("Drive Y", y);
            telemetry.addData("Drive X", x);
            telemetry.addData("Drive R", r);
            telemetry.addData("LB Slow Mode", gamepad1.left_bumper ? "ACTIVE" : "off");
            telemetry.addData("Start+Back Reset", comboReset ? "PRESSED" : "");
            telemetry.addData("X+Y Recalibrate", comboRecal ? "PRESSED" : ""); // Fix typo: Recal -> Recalibrate

            telemetry.addData("Cam LB", gamepad2.left_bumper ? "IN" : "--");
            telemetry.addData("Cam RB", gamepad2.right_bumper ? "OUT" : "--");
            telemetry.addData("Cam Position", camPosition);
            telemetry.addData("Sweeper A", gamepad2.a ? "IN" : "--");
            telemetry.addData("Sweeper B", gamepad2.b ? "OUT" : "--");
            telemetry.addData("Sweeper Position", sweeperPosition);
            telemetry.addData("Launcher X", gamepad2.x ? "ACTIVE" : "off");
            telemetry.addData("Intake RT", gamepad2.right_trigger);
            telemetry.addData("Intake LT", gamepad2.left_trigger);
            telemetry.addData("Intake Power", intakePower);

            telemetry.addData("Cam Target", camPosition);
            telemetry.addData("Cam Actual", hw.cam != null ? hw.cam.getPosition() : -1);
            telemetry.addData("Sweeper Target", sweeperPosition);
            telemetry.addData("Sweeper Actual", sweeperServo != null ? sweeperServo.getPosition() : -1);

            if (hw.depositMotorL != null && hw.depositMotorR != null) {
                double ticksPerRev = 28.0;
                double velocityL = hw.depositMotorL.getVelocity();
                double velocityR = hw.depositMotorR.getVelocity();
                double rpmL = (velocityL / ticksPerRev) * 60.0;
                double rpmR = (velocityR / ticksPerRev) * 60.0;
                telemetry.addData("Launcher L RPM", rpmL);
                telemetry.addData("Launcher R RPM", rpmR);
            } else {
                telemetry.addData("Launcher", "NOT FOUND");
            }

            telemetry.addData("Loop Hz", 1.0/dt);
            telemetry.addData("Heading", pos.getHeadingDeg());
            telemetry.update();

            // Panels publishing (inches)
            double xIn = pos.getXmm()/25.4;
            double yIn = pos.getYmm()/25.4;
            panels.putText("teleop/pinpoint/status", hw.pinpoint!=null? hw.pinpoint.getDeviceStatus().name():"NONE");
            panels.publishPose("teleop/pose", xIn, yIn, pos.getHeadingDeg(), 50);
        }
    }
}
