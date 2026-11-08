package org.firstinspires.ftc.teamcode.main;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.auxiliary.FlywheelController;
import org.firstinspires.ftc.teamcode.auxiliary.Navigation;
import org.firstinspires.ftc.teamcode.config.RobotConstants;
import org.firstinspires.ftc.teamcode.drive.Drivetrain;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;

@TeleOp(name = "Main: BBB Driver", group = "Main")
public class MainTeleOp extends OpMode {
    private Drivetrain drivetrain;
    private RobotHardware hardware;
    private Navigation.Pinpoint localizer;
    private FlywheelController flywheelController;
    private boolean shooterEnabled = false;
    private boolean shooterButtonPrev = false;
    private boolean blockerClosed = false;
    private boolean blockerButtonPrev = false;
    private double flywheelTargetTicksPerSecond = RobotConstants.FLYWHEEL_TICKS_PER_SECOND;
    private double flowerPower = 0.0;

    @Override
    public void init() {
        hardware = new RobotHardware(hardwareMap);
        drivetrain = new Drivetrain(hardwareMap);
        localizer = new Navigation.Pinpoint(hardwareMap);
        flywheelController = new FlywheelController(hardware.shooterMotor, null);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        if (hardware.blockerServo != null || hardware.axonBlockerServo != null) {
            hardware.setBlockerClosed(false);
        }
    }

    @Override
    public void init_loop() {
        localizer.update();
        telemetry.addData("Pinpoint", localizer.getStatusText());
        telemetry.addData("Pose valid", localizer.isPoseValid() ? "yes" : "no");
        telemetry.addData("Driver 1", "Left stick drive / Right stick turn / RT/LT intake");
        telemetry.addData("Driver 2", "Y toggles shooter / A toggles blocker / ZR/ZL = flower spin same direction");
        telemetry.update();
    }

    @Override
    public void loop() {
        localizer.update();
        drivetrain.update(gamepad1, gamepad2, false, 0.0);

        if (gamepad1.dpad_up && !gamepad1.dpad_down) {
            flywheelTargetTicksPerSecond = Math.min(5000.0, flywheelTargetTicksPerSecond + 20.0);
        } else if (gamepad1.dpad_down && !gamepad1.dpad_up) {
            flywheelTargetTicksPerSecond = Math.max(0.0, flywheelTargetTicksPerSecond - 20.0);
        }

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

        if (gamepad1.right_trigger > 0.05) {
            if (hardware.intakeMotor != null) {
                hardware.intakeMotor.setPower(1.0);
            }
        } else if (gamepad1.left_trigger > 0.05) {
            if (hardware.intakeMotor != null) {
                hardware.intakeMotor.setPower(-1.0);
            }
        } else if (hardware.intakeMotor != null) {
            hardware.intakeMotor.setPower(0.0);
        }

        double shooterVelocity = hardware.shooterMotor != null ? hardware.shooterMotor.getVelocity() : 0.0;

        telemetry.addData("Pinpoint", localizer.getStatusText());
        telemetry.addData("Pose", String.format("(%.1f in, %.1f in, %.1f deg)",
                localizer.getXInches(), localizer.getYInches(), Math.toDegrees(localizer.getHeadingRadians())));
        telemetry.addData("Pose valid", localizer.isPoseValid() ? "yes" : "no");
        telemetry.addData("Shooter", shooterEnabled ? "ON" : "OFF");
        telemetry.addData("Shooter target", String.format("%.1f ticks/s", flywheelTargetTicksPerSecond));
        telemetry.addData("Shooter actual", shooterVelocity);
        telemetry.addData("graph/flywheel_target_ticks_per_s", shooterEnabled ? flywheelTargetTicksPerSecond : 0.0);
        telemetry.addData("graph/flywheel_actual_ticks_per_s", shooterVelocity);
        telemetry.addData("graph/flywheel_controller_power", flywheelController.getOutputPower());
        telemetry.addData("graph/flower_power", flowerPower);
        telemetry.addData("Blocker", blockerClosed ? "CLOSED" : "OPEN");
        telemetry.addData("Flower", String.format("%.1f power (ZR = forward, ZL = reverse)", flowerPower));
        telemetry.update();
    }

    @Override
    public void stop() {
        if (flywheelController != null) {
            flywheelController.stop();
        }
        if (drivetrain != null) {
            drivetrain.stop();
        }
        if (hardware != null) {
            hardware.stopMechanisms();
        }
    }
}
