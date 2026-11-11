package org.firstinspires.ftc.teamcode.main;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.controllers.Controller;
import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.ManualDrive;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.auxiliary.FlywheelController;
import org.firstinspires.ftc.teamcode.auxiliary.Navigation;
import org.firstinspires.ftc.teamcode.config.RobotConstants;
import org.firstinspires.ftc.teamcode.drive.Drivetrain;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.pedro.Constants;

@TeleOp(name = "Main: BBB Driver", group = "Main")
public class MainTeleOp extends OpMode {

    // Aim points (field inches): red = 0-1, blue = 2-3.
    private static final double[][] AIM_XY = {
            {57.38, 84.90}, {57.69, 58.00},
            {83.97, 81.48}, {83.81, 57.53}
    };
    // The four field corners, used by the corner relocalize button.
    private static final double[][] CORNERS = {
            {8.6, 8.2}, {8.3, 133.2}, {133.0, 133.2}, {132.8, 8.9}
    };

    private Follower follower;
    private RobotHardware hardware;
    private Drivetrain drivetrain;
    private FlywheelController flywheelController;

    private Navigation.Alliance alliance = null;
    private boolean poseValid = false;

    // Turn controller used by the aim heading lock.
    private static final Controller AIM_CONTROLLER = Controller.proportional(RobotConstants.AIM_TURN_GAIN);

    private boolean shooterEnabled = false;
    private boolean shooterButtonPrev = false;
    private boolean blockerClosed = false;
    private boolean blockerButtonPrev = false;
    private boolean lb2Prev = false;
    private boolean rightStickButtonPrev = false;
    private boolean fieldCentric = true;
    private boolean fieldCentricTogglePrev = false;
    private double flywheelTargetTicksPerSecond = RobotConstants.FLYWHEEL_TICKS_PER_SECOND;
    private final DpadPress dpad1Prev = new DpadPress();
    private final DpadPress dpad2Prev = new DpadPress();
    private double flowerPower = 0.0;

    private static class DpadPress {
        boolean up, down, left, right;
    }

    @Override
    public void init() {
        hardware = new RobotHardware(hardwareMap);
        drivetrain = new Drivetrain(hardwareMap);
        flywheelController = new FlywheelController(hardware.shooterMotor, null);
        follower = Constants.create(hardwareMap);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // Always start at the same initialization pose: exact field origin in inches,
        // with the robot facing +90 degrees (CCW from +X, matching the field axes).
        follower.setPose(new Pose(0.0, 0.0, Math.toRadians(90.0)));
        poseValid = true;

        if (hardware.blockerServo != null || hardware.axonBlockerServo != null) {
            hardware.setBlockerClosed(false);
        }
    }

    @Override
    public void init_loop() {
        updateDriveModeToggle();
        Navigation.AllianceSelector.update(gamepad1, gamepad2);
        alliance = Navigation.AllianceSelector.get();
        Navigation.AllianceSelector.addTelemetry(telemetry);
        telemetry.addData("Driver 1", "Left stick drive / Right stick turn / RT/LT intake / D-pad ticks: up/down +/-50, right/left +/-10");
        telemetry.addData("Driver 2", "Y toggles shooter / A toggles blocker / ZR/ZL flower / D-pad ticks / RB2 = heading aim / LB2 = set nearest corner / Gamepad2 RS click = reset to (9.1, 9.1, 90) / Start toggles field-vs-robot drive");
        telemetry.addData("Drive mode", fieldCentric ? "FIELD CENTRIC" : "ROBOT CENTRIC");
        telemetry.update();
    }

    @Override
    public void loop() {
        follower.update();

        updateDriveModeToggle();
        updateResetPose();
        updateDrive();
        updateRelocalize();
        alliance = Navigation.AllianceSelector.get();

        if (gamepad2.y && !shooterButtonPrev) {
            shooterEnabled = !shooterEnabled;
        }
        shooterButtonPrev = gamepad2.y;

        if (shooterEnabled) {
            flywheelController.setTarget(flywheelTargetTicksPerSecond);
            flywheelController.update();
        } else {
            flywheelController.stop();
        }

        if (gamepad2.a && !blockerButtonPrev) {
            blockerClosed = !blockerClosed;
        }
        blockerButtonPrev = gamepad2.a;

        if (hardware.blockerServo != null || hardware.axonBlockerServo != null) {
            hardware.setBlockerClosed(blockerClosed);
        }

        if (hardware.hasContinuousServos) {
            boolean zrPressed = gamepad2.right_trigger > 0.05 || gamepad2.right_trigger_pressed;
            boolean zlPressed = gamepad2.left_trigger > 0.05 || gamepad2.left_trigger_pressed;

            if (zrPressed) {
                hardware.continuousServoLeft.setPower(1.0);
                hardware.continuousServoRight.setPower(1.0);
                flowerPower = 1.0;
            } else if (zlPressed) {
                hardware.continuousServoLeft.setPower(-1.0);
                hardware.continuousServoRight.setPower(-1.0);
                flowerPower = -1.0;
            } else {
                hardware.continuousServoLeft.setPower(0.0);
                hardware.continuousServoRight.setPower(0.0);
                flowerPower = 0.0;
            }
        }

        drivetrain.updateAuxiliaries(gamepad1, gamepad2);

        updateFlywheelTicks(gamepad1, dpad1Prev);
        updateFlywheelTicks(gamepad2, dpad2Prev);

        double shooterVelocity = hardware.shooterMotor != null ? hardware.shooterMotor.getVelocity() : 0.0;

        Pose pose = follower.pose();
        telemetry.addData("Alliance", alliance == null ? "NONE (select during init)" : alliance.getDisplayName());
        telemetry.addData("Pose", String.format("(%.1f in, %.1f in, %.1f deg)",
                pose.x(), pose.y(), Math.toDegrees(pose.heading())));
        telemetry.addData("Mode", follower.mode().name());
        telemetry.addData("Aim", poseValid ? (aimActive() ? "heading-only aim" : "RB2 = aim") : "pose invalid (LB2 or RS click set pose)");
        telemetry.addData("Shooter", shooterEnabled ? "ON" : "OFF");
        telemetry.addData("Shooter target", String.format("%.1f ticks/s", flywheelTargetTicksPerSecond));
        telemetry.addData("Shooter actual", shooterVelocity);
        telemetry.addData("graph/flywheel_target_ticks_per_s", shooterEnabled ? flywheelTargetTicksPerSecond : 0.0);
        telemetry.addData("graph/flywheel_actual_ticks_per_s", shooterVelocity);
        telemetry.addData("graph/flywheel_controller_power", flywheelController.getOutputPower());
        telemetry.addData("graph/flower_power", flowerPower);
        telemetry.addData("Blocker", blockerClosed ? "CLOSED" : "OPEN");
        telemetry.update();
    }

    /** Field-centric vs robot-centric driving through Pedro; toggled at init and on Start/driver toggle. */
    private void updateDrive() {
        double forward = -gamepad1.left_stick_y * RobotConstants.DRIVE_SPEED_SCALE;
        double strafe = -gamepad1.left_stick_x * RobotConstants.STRAFE_SPEED_SCALE;
        double turn = -gamepad1.right_stick_x * RobotConstants.TURN_SPEED_SCALE;

        DrivePowers powers;
        if (fieldCentric) {
            powers = ManualDrive.fieldCentric(
                    forward,
                    strafe,
                    turn,
                    follower.pose().heading());
        } else {
            powers = new DrivePowers(forward, strafe, turn);
        }

        if (aimActive()) {
            // Auto-aim is heading-only: no translation while RB is held.
            double desiredHeading = aimHeading();
            double turnPower = computeHeadingTurn(desiredHeading, follower.pose().heading());
            double[] wheelPowers = mecanumMix(0.0, 0.0, turnPower);
            hardware.setDrivePowers(wheelPowers[0], wheelPowers[1], wheelPowers[2], wheelPowers[3]);
            return;
        }

        if (fieldCentric) {
            ManualDrive.driveOrHold(follower, powers);
        } else {
            follower.manual(powers);
        }
    }

    private void updateDriveModeToggle() {
        boolean togglePressed = gamepad1 != null && gamepad1.start && !fieldCentricTogglePrev;
        if (togglePressed) {
            fieldCentric = !fieldCentric;
        }
        fieldCentricTogglePrev = gamepad1 != null && gamepad1.start;
    }

    private double computeHeadingTurn(double desiredHeading, double currentHeading) {
        double headingError = angleWrap(desiredHeading - currentHeading);
        if (Math.abs(headingError) < RobotConstants.AIM_DEADBAND_RADIANS) {
            return 0.0;
        }
        double turn = Math.max(-RobotConstants.AIM_MAX_TURN,
                Math.min(RobotConstants.AIM_MAX_TURN, RobotConstants.AIM_TURN_GAIN * headingError));
        return turn * RobotConstants.AIM_TURN_SIGN;
    }

    private double angleWrap(double angle) {
        while (angle > Math.PI) {
            angle -= 2.0 * Math.PI;
        }
        while (angle < -Math.PI) {
            angle += 2.0 * Math.PI;
        }
        return angle;
    }

    private double[] mecanumMix(double forward, double strafe, double turn) {
        double frontLeft = forward + strafe + turn;
        double backLeft = forward - strafe + turn;
        double frontRight = forward - strafe - turn;
        double backRight = forward + strafe - turn;

        double max = Math.max(Math.abs(frontLeft), Math.max(Math.abs(backLeft),
                Math.max(Math.abs(frontRight), Math.abs(backRight))));
        if (max > 1.0) {
            frontLeft /= max;
            backLeft /= max;
            frontRight /= max;
            backRight /= max;
        }

        return new double[] {frontLeft, backLeft, frontRight, backRight};
    }

    /** RB2 held and the pose is usable. */
    private boolean aimActive() {
        return gamepad2.right_bumper && poseValid && alliance != null;
    }

    /** Heading that faces the nearest aim point for our alliance (re-picked every call). */
    private double aimHeading() {
        Pose pose = follower.pose();
        int base = alliance == Navigation.Alliance.RED ? 0 : 2;
        int best = base;
        double bestDistance = Double.MAX_VALUE;
        for (int i = base; i < base + 2; i++) {
            double dx = AIM_XY[i][0] - pose.x();
            double dy = AIM_XY[i][1] - pose.y();
            double d = dx * dx + dy * dy;
            if (d < bestDistance) {
                bestDistance = d;
                best = i;
            }
        }
        return Math.atan2(AIM_XY[best][1] - pose.y(), AIM_XY[best][0] - pose.x())
                + Math.toRadians(RobotConstants.AIM_HEADING_OFFSET_DEG);
    }

    /** LB2 pressed: set the pose to whichever field corner we're closest to without moving. */
    private void updateRelocalize() {
        if (gamepad2.left_bumper && !lb2Prev) {
            relocalizeCorner();
        }
        lb2Prev = gamepad2.left_bumper;
    }

    private void relocalizeCorner() {
        Pose pose = follower.pose();
        int best = 0;
        double bestDistance = Double.MAX_VALUE;
        for (int i = 0; i < CORNERS.length; i++) {
            double dx = CORNERS[i][0] - pose.x();
            double dy = CORNERS[i][1] - pose.y();
            double d = dx * dx + dy * dy;
            if (d < bestDistance) {
                bestDistance = d;
                best = i;
            }
        }
        Pose newPose = new Pose(CORNERS[best][0], CORNERS[best][1], Math.toRadians(90.0));
        follower.setPose(newPose);
        poseValid = true;
    }

    /** RS click on gamepad2 resets the pose to the fixed corner reference without moving. */
    private void updateResetPose() {
        boolean pressed = gamepad2 != null && gamepad2.right_stick_button && !rightStickButtonPrev;
        if (pressed) {
            Pose resetPose = new Pose(9.1, 9.1, Math.toRadians(90.0));
            follower.setPose(resetPose);
            poseValid = true;
        }
        rightStickButtonPrev = gamepad2 != null && gamepad2.right_stick_button;
    }

    /** Steps the flywheel target once per dpad press: up/down +/-50, right/left +/-10. */    private void updateFlywheelTicks(Gamepad gamepad, DpadPress prev) {
        if (gamepad.dpad_up && !prev.up) {
            flywheelTargetTicksPerSecond = Math.min(5000.0, flywheelTargetTicksPerSecond + 50.0);
        }
        if (gamepad.dpad_down && !prev.down) {
            flywheelTargetTicksPerSecond = Math.max(0.0, flywheelTargetTicksPerSecond - 50.0);
        }
        if (gamepad.dpad_right && !prev.right) {
            flywheelTargetTicksPerSecond = Math.min(5000.0, flywheelTargetTicksPerSecond + 10.0);
        }
        if (gamepad.dpad_left && !prev.left) {
            flywheelTargetTicksPerSecond = Math.max(0.0, flywheelTargetTicksPerSecond - 10.0);
        }
        prev.up = gamepad.dpad_up;
        prev.down = gamepad.dpad_down;
        prev.left = gamepad.dpad_left;
        prev.right = gamepad.dpad_right;
    }

    @Override
    public void stop() {
        if (flywheelController != null) {
            flywheelController.stop();
        }
        if (drivetrain != null) {
            drivetrain.stop();
        }
        if (follower != null) {
            follower.stop();
        }
        if (hardware != null) {
            hardware.stopMechanisms();
        }
    }
}
