package org.firstinspires.ftc.teamcode.main;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.auxiliary.FlywheelController;
import org.firstinspires.ftc.teamcode.auxiliary.Navigation;
import org.firstinspires.ftc.teamcode.config.RobotConstants;
import org.firstinspires.ftc.teamcode.drive.Drivetrain;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;

@TeleOp(name = "Main: BBB Driver", group = "Main")
public class MainTeleOp extends OpMode {
    private Drivetrain drivetrain;
    private RobotHardware hardware;
    private FlywheelController flywheelController;
    private boolean shooterEnabled = false;
    private boolean shooterButtonPrev = false;
    private boolean blockerClosed = false;
    private boolean blockerButtonPrev = false;
    private double flywheelTargetTicksPerSecond = RobotConstants.FLYWHEEL_TICKS_PER_SECOND;
    private final DpadPress dpad1Prev = new DpadPress();
    private final DpadPress dpad2Prev = new DpadPress();
    private double flowerPower = 0.0;

    /** Edge-detection state for one gamepad's dpad. */
    private static class DpadPress {
        boolean up, down, left, right;
    }

    /**
     * Steps the flywheel target once per dpad press (not while held):
     * up/down = +/-50 ticks/s, right/left = +/-10 ticks/s.
     */
    private void updateFlywheelTicks(Gamepad gamepad, DpadPress prev) {
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
    public void init() {
        hardware = new RobotHardware(hardwareMap);
        drivetrain = new Drivetrain(hardwareMap);
        flywheelController = new FlywheelController(hardware.shooterMotor, null);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        if (hardware.blockerServo != null || hardware.axonBlockerServo != null) {
            hardware.setBlockerClosed(false);
        }
    }

    @Override
    public void init_loop() {
        telemetry.addData("Driver 1", "Left stick drive / Right stick turn / RT/LT intake / D-pad ticks: up/down +/-50, right/left +/-10");
        telemetry.addData("Driver 2", "Y toggles shooter / A toggles blocker / ZR/ZL = flower spin same direction / D-pad ticks same as driver 1");
        telemetry.update();
    }

    @Override
    public void loop() {
        drivetrain.update(gamepad1, gamepad2, false, 0.0);

        // Flywheel tick adjust, once per dpad press, same scheme on both pads.
        updateFlywheelTicks(gamepad1, dpad1Prev);
        updateFlywheelTicks(gamepad2, dpad2Prev);

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

        // Raw analogs: if a stick is off-center while untouched, the controller
        // was plugged in deflected (DS stores that as zero) - unplug, center the
        // sticks, replug. An F310 "Mode" LED lit also swaps dpad/left stick.
        telemetry.addData("GP1", "lx %.2f  ly %.2f  rx %.2f  ry %.2f  lt %.2f  rt %.2f",
                gamepad1.left_stick_x, gamepad1.left_stick_y,
                gamepad1.right_stick_x, gamepad1.right_stick_y,
                gamepad1.left_trigger, gamepad1.right_trigger);
        telemetry.addData("GP2", "lx %.2f  ly %.2f  rx %.2f  ry %.2f  lt %.2f  rt %.2f",
                gamepad2.left_stick_x, gamepad2.left_stick_y,
                gamepad2.right_stick_x, gamepad2.right_stick_y,
                gamepad2.left_trigger, gamepad2.right_trigger);

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
