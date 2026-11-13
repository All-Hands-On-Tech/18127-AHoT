package org.firstinspires.ftc.teamcode.main;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.utils.Angle;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.auxiliary.FlywheelController;
import org.firstinspires.ftc.teamcode.auxiliary.Navigation;
import org.firstinspires.ftc.teamcode.config.RobotConstants;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.pedro.Constants;

@TeleOp(name = "Main: BBB Driver", group = "Main")
public class MainTeleOp extends OpMode {

    // Aim points (field inches): red = 0-1, blue = 2-3.
    private static final double[][] AIM_XY = {
            {57.38, 84.90}, {57.69, 58.00},
            {83.97, 81.48}, {83.81, 57.53}
    };
    private static final double[][] CORNERS = {
            {8.6, 8.2}, {8.3, 133.2}, {133.0, 133.2}, {132.8, 8.9}
    };

    private Follower follower;
    private RobotHardware hardware;
    private FlywheelController flywheel;
    private Navigation.Alliance alliance = null;

    private boolean shooterEnabled = false;
    private boolean blockerClosed = false;
    private double flywheelTarget = RobotConstants.FLYWHEEL_TICKS_PER_SECOND;

    private boolean shooterPrev, blockerPrev, lb2Prev, rsPrev, rb1Prev;
    private final DpadPress dpad1 = new DpadPress();
    private final DpadPress dpad2 = new DpadPress();

    private static class DpadPress {
        boolean up, down, left, right;
    }

    @Override
    public void init() {
        hardware = new RobotHardware(hardwareMap);
        flywheel = new FlywheelController(hardware.shooterMotor);
        follower = Constants.create(hardwareMap);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        setPose(new Pose(0.0, 0.0, Math.toRadians(90.0)));
        hardware.setBlockerClosed(false);
    }

    @Override
    public void init_loop() {
        Navigation.AllianceSelector.update(gamepad1, gamepad2);
        alliance = Navigation.AllianceSelector.get();
        Navigation.AllianceSelector.addTelemetry(telemetry);
        telemetry.addData("Drive mode", "ROBOT CENTRIC");
        telemetry.addData("Driver 1", "LS drive / RS turn / triggers intake / RB hold = gate open / dpad = flywheel");
        telemetry.addData("Driver 2", "Y shooter / A blocker / triggers flower / RB hold = aim / LB = nearest corner / RS click = reset pose");
        telemetry.update();
    }

    @Override
    public void loop() {
        follower.update();
        alliance = Navigation.AllianceSelector.get();

        updateRelocalize();
        updateDrive();

        if (gamepad2.y && !shooterPrev) shooterEnabled = !shooterEnabled;
        shooterPrev = gamepad2.y;
        if (shooterEnabled) {
            flywheel.setTarget(flywheelTarget);
            flywheel.update();
        } else {
            flywheel.stop();
        }

        // Driver 2: A toggles the blocker
        if (gamepad2.a && !blockerPrev) blockerClosed = !blockerClosed;
        blockerPrev = gamepad2.a;

        // Driver 1: hold RB to keep the blocker open, release to close it
        boolean rb1Held = gamepad1.left_bumper;
        if (rb1Held) {
            blockerClosed = false;       // held = open
        } else if (rb1Prev) {
            blockerClosed = true;        // just released = close
        }
        rb1Prev = rb1Held;

        hardware.setBlockerClosed(blockerClosed);

        if (gamepad2.right_trigger > 0.05) hardware.setFlowerPower(1.0);
        else if (gamepad2.left_trigger > 0.05) hardware.setFlowerPower(-1.0);
        else hardware.setFlowerPower(0.0);

        if (gamepad1.right_bumper) {
            hardware.setIntakePower(RobotConstants.INTAKE_POWER);
        } else if (gamepad1.x) {
            hardware.setIntakePower(-1 * RobotConstants.INTAKE_POWER);
        } else {
            hardware.setIntakePower(0.0);
        }

        updateFlywheelTicks(gamepad1, dpad1);
        updateFlywheelTicks(gamepad2, dpad2);

        Pose pose = follower.pose();
        telemetry.addData("Alliance", alliance == null ? "NONE (select during init)" : alliance.getDisplayName());
        telemetry.addData("Pose", String.format("(%.1f, %.1f, %.1f deg)",
                pose.x(), pose.y(), Math.toDegrees(pose.heading())));
        telemetry.addData("Mode", follower.mode().name());
        telemetry.addData("Aim", aimActive() ? "AIMING" : "RB2 = aim");
        telemetry.addData("Shooter", shooterEnabled ? "ON" : "OFF");
        telemetry.addData("Shooter target", flywheelTarget);
        telemetry.addData("Shooter actual", flywheel.getVelocity());
        telemetry.addData("graph/flywheel_target_ticks_per_s", shooterEnabled ? flywheelTarget : 0.0);
        telemetry.addData("graph/flywheel_actual_ticks_per_s", flywheel.getVelocity());
        telemetry.addData("graph/flywheel_controller_power", flywheel.getOutputPower());
        telemetry.addData("Blocker", blockerClosed ? "CLOSED" : "OPEN");
        telemetry.update();
    }

    // ---------------------------------------------------------------- drive

    private void updateDrive() {
        double forward = -gamepad1.left_stick_y * RobotConstants.DRIVE_SPEED_SCALE;
        double strafe = -gamepad1.left_stick_x * RobotConstants.STRAFE_SPEED_SCALE;
        double turn = -gamepad1.right_stick_x * RobotConstants.TURN_SPEED_SCALE;

        // Heading-only aim, driven through Pedro so nothing fights it.
        if (aimActive()) {
            follower.manual(new DrivePowers(0.0, 0.0, aimTurn()));
            return;
        }

        // Robot centric drive.
        follower.manual(new DrivePowers(forward, strafe, turn));
    }

    // ------------------------------------------------------------------ aim

    private boolean aimActive() {
        return gamepad2.right_bumper && alliance != null;
    }

    /** Turn power toward the nearest aim point for our alliance (+ = CCW). */
    private double aimTurn() {
        Pose pose = follower.pose();
        int base = alliance == Navigation.Alliance.RED ? 0 : 2;
        int best = base;
        double bestDist = Double.MAX_VALUE;
        for (int i = base; i < base + 2; i++) {
            double dx = AIM_XY[i][0] - pose.x();
            double dy = AIM_XY[i][1] - pose.y();
            double d = dx * dx + dy * dy;
            if (d < bestDist) {
                bestDist = d;
                best = i;
            }
        }
        double desired = Math.atan2(AIM_XY[best][1] - pose.y(), AIM_XY[best][0] - pose.x())
                + Math.toRadians(RobotConstants.AIM_HEADING_OFFSET_DEG);
        double error = Angle.normalizeSigned(desired - pose.heading());
        if (Math.abs(error) < RobotConstants.AIM_DEADBAND_RADIANS) return 0.0;

        double turn = Math.max(-RobotConstants.AIM_MAX_TURN,
                Math.min(RobotConstants.AIM_MAX_TURN, RobotConstants.AIM_TURN_GAIN * error));
        return turn * RobotConstants.AIM_TURN_SIGN;
    }

    // ----------------------------------------------------------- relocalize

    /** Every pose reset goes through here so the hold target moves with the pose. */
    private void setPose(Pose pose) {
        follower.setPose(pose);
        follower.hold(pose);
    }

    private void updateRelocalize() {
        // LB2: snap to the nearest field corner.
        if (gamepad2.left_bumper && !lb2Prev) {
            Pose pose = follower.pose();
            int best = 0;
            double bestDist = Double.MAX_VALUE;
            for (int i = 0; i < CORNERS.length; i++) {
                double dx = CORNERS[i][0] - pose.x();
                double dy = CORNERS[i][1] - pose.y();
                double d = dx * dx + dy * dy;
                if (d < bestDist) {
                    bestDist = d;
                    best = i;
                }
            }
            setPose(new Pose(CORNERS[best][0], CORNERS[best][1], Math.toRadians(90.0)));
        }
        lb2Prev = gamepad2.left_bumper;

        // RS click on gamepad2: reset to the fixed corner reference.
        if (gamepad2.right_stick_button && !rsPrev) {
            setPose(new Pose(9.1, 9.1, Math.toRadians(90.0)));
        }
        rsPrev = gamepad2.right_stick_button;
    }

    // ------------------------------------------------------------- flywheel

    /** dpad up/down = +/-50, right/left = +/-10 ticks/s. */
    private void updateFlywheelTicks(Gamepad g, DpadPress prev) {
        if (g.dpad_up && !prev.up) flywheelTarget = Math.min(5000.0, flywheelTarget + 50.0);
        if (g.dpad_down && !prev.down) flywheelTarget = Math.max(0.0, flywheelTarget - 50.0);
        if (g.dpad_right && !prev.right) flywheelTarget = Math.min(5000.0, flywheelTarget + 10.0);
        if (g.dpad_left && !prev.left) flywheelTarget = Math.max(0.0, flywheelTarget - 10.0);
        prev.up = g.dpad_up;
        prev.down = g.dpad_down;
        prev.left = g.dpad_left;
        prev.right = g.dpad_right;
    }

    @Override
    public void stop() {
        if (flywheel != null) flywheel.stop();
        if (follower != null) follower.stop();
        if (hardware != null) hardware.stopMechanisms();
    }
}