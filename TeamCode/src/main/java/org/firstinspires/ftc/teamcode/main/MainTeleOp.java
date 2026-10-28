package org.firstinspires.ftc.teamcode.main;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.config.RobotConstants;
import org.firstinspires.ftc.teamcode.drive.Alliance;
import org.firstinspires.ftc.teamcode.drive.Drivetrain;
import org.firstinspires.ftc.teamcode.drive.OdometryAimAssist;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;

@TeleOp(name = "Main: BBB Driver", group = "Main")
public class MainTeleOp extends OpMode {
    private Drivetrain drivetrain;
    private RobotHardware hardware;
    private Alliance selectedAlliance = Alliance.BLUE;
    private boolean shooterEnabled = false;
    private boolean shooterButtonPrev = false;
    private boolean blockerClosed = false;
    private boolean blockerButtonPrev = false;
    private boolean relocateButtonPrev = false;
    private boolean dpadUpPrev = false;
    private boolean dpadDownPrev = false;
    private double flywheelTargetTicksPerSecond = RobotConstants.FLYWHEEL_TICKS_PER_SECOND;

    @Override
    public void init() {
        hardware = new RobotHardware(hardwareMap);
        drivetrain = new Drivetrain(hardwareMap);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        drivetrain.setAlliance(selectedAlliance);
        if (hardware.blockerServo != null || hardware.axonBlockerServo != null) {
            hardware.setBlockerClosed(false);
        }
        if (drivetrain.getAimAssist() != null) {
            drivetrain.getAimAssist().setTargetPoint(8.356902356902351, 9.111672278338956);
            drivetrain.getAimAssist().setPointTargetEnabled(true);
        }
    }

    @Override
    public void init_loop() {
        if (gamepad1.dpad_left || gamepad1.x) {
            selectedAlliance = Alliance.RED;
        } else if (gamepad1.dpad_right || gamepad1.b) {
            selectedAlliance = Alliance.BLUE;
        }

        drivetrain.setAlliance(selectedAlliance);

        telemetry.addData("Alliance", selectedAlliance.getDisplayName());
        telemetry.addData("Select", "D-pad left/X = red, D-pad right/B = blue");
        telemetry.addData("Driver 1", "Left stick drive / Right stick turn / RT/LT intake / LB aim assist");
        telemetry.addData("Driver 2", "Y toggles shooter / A toggles blocker / X relocalize target / LT/RT flower control");
        telemetry.update();
    }

    @Override
    public void loop() {
        drivetrain.update(gamepad1, gamepad2, gamepad1.left_bumper);

        if (gamepad1.dpad_up && !dpadUpPrev) {
            flywheelTargetTicksPerSecond += 10.0;
        }
        if (gamepad1.dpad_down && !dpadDownPrev) {
            flywheelTargetTicksPerSecond = Math.max(0.0, flywheelTargetTicksPerSecond - 10.0);
        }
        dpadUpPrev = gamepad1.dpad_up;
        dpadDownPrev = gamepad1.dpad_down;

        if (gamepad2.y && !shooterButtonPrev) {
            shooterEnabled = !shooterEnabled;
        }
        shooterButtonPrev = gamepad2.y;

        if (hardware.leftFlywheel != null && hardware.rightFlywheel != null) {
            if (shooterEnabled) {
                hardware.leftFlywheel.setVelocity(flywheelTargetTicksPerSecond);
                hardware.rightFlywheel.setVelocity(flywheelTargetTicksPerSecond);
            } else {
                hardware.leftFlywheel.setPower(0.0);
                hardware.rightFlywheel.setPower(0.0);
            }
        }

        if (gamepad2.a && !blockerButtonPrev) {
            blockerClosed = !blockerClosed;
        }
        blockerButtonPrev = gamepad2.a;

        if (hardware.blockerServo != null || hardware.axonBlockerServo != null) {
            hardware.setBlockerClosed(blockerClosed);
        }

        if (gamepad2.x && !relocateButtonPrev) {
            if (drivetrain.getAimAssist() != null) {
                drivetrain.getAimAssist().setPose(8.356902356902351, 9.111672278338956, 0.0);
                drivetrain.getAimAssist().setTargetPoint(8.356902356902351, 9.111672278338956);
                drivetrain.getAimAssist().setPointTargetEnabled(true);
            }
        }
        relocateButtonPrev = gamepad2.x;

        if (hardware.flowerLeftServo != null || hardware.flowerRightServo != null) {
            double leftPos = RobotConstants.FLOWER_DEFAULT_POSITION;
            double rightPos = RobotConstants.FLOWER_DEFAULT_POSITION;

            if (gamepad2.right_trigger > 0.05) {
                leftPos = RobotConstants.FLOWER_ACTIVE_POSITION;
                rightPos = RobotConstants.FLOWER_DEFAULT_POSITION;
            } else if (gamepad2.left_trigger > 0.05) {
                leftPos = RobotConstants.FLOWER_DEFAULT_POSITION;
                rightPos = RobotConstants.FLOWER_ACTIVE_POSITION;
            }

            hardware.setFlowerPositions(leftPos, rightPos);
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

        double leftVelocity = hardware.leftFlywheel != null ? hardware.leftFlywheel.getVelocity() : 0.0;
        double rightVelocity = hardware.rightFlywheel != null ? hardware.rightFlywheel.getVelocity() : 0.0;
        double averageVelocity = (leftVelocity + rightVelocity) * 0.5;
        double velocityError = flywheelTargetTicksPerSecond - averageVelocity;

        // Numeric values for live Panels graphing.
        telemetry.addData("graph/flywheel_target_ticks_per_s", flywheelTargetTicksPerSecond);
        telemetry.addData("graph/flywheel_left_ticks_per_s", leftVelocity);
        telemetry.addData("graph/flywheel_right_ticks_per_s", rightVelocity);
        telemetry.addData("graph/flywheel_avg_ticks_per_s", averageVelocity);
        telemetry.addData("graph/flywheel_error_ticks_per_s", velocityError);

        telemetry.addData("Alliance", selectedAlliance.getDisplayName());
        telemetry.addData("Drive enabled", drivetrain.hasDrive());
        telemetry.addData("Odometry enabled", drivetrain.hasAimAssist());
        telemetry.addData("Driver 1", "LS drive / RS turn / RT/LT intake / D-pad up/down flywheel target +/-10");
        telemetry.addData("Driver 2", "Y toggles shooter / A toggles blocker / LT/RT flower");
        telemetry.addData("Shooter", shooterEnabled ? "ON" : "OFF");
        telemetry.addData("Shooter target", String.format("%.1f ticks/s", flywheelTargetTicksPerSecond));
        if (hardware.leftFlywheel != null) {
            telemetry.addData("Shooter actual left", leftVelocity);
        }
        if (hardware.rightFlywheel != null) {
            telemetry.addData("Shooter actual right", rightVelocity);
        }
        telemetry.addData("Blocker", blockerClosed ? "CLOSED" : "OPEN");
        telemetry.addData("Aim", gamepad1.left_bumper ? "holding" : "off");
        if (drivetrain.getAimAssist() != null) {
            OdometryAimAssist aimAssist = drivetrain.getAimAssist();
            telemetry.addData("Pose", aimAssist.getPoseText());
            telemetry.addData("Pinpoint", aimAssist.getOdometryStatusText());
            telemetry.addData("Aim error (deg)", aimAssist.getHeadingErrorDegrees());
        } else {
            telemetry.addData("Pose", "disabled");
            telemetry.addData("Aim error (deg)", "disabled");
        }
        telemetry.update();
    }

    @Override
    public void stop() {
        if (drivetrain != null) {
            drivetrain.stop();
        }
        if (hardware != null) {
            hardware.stopMechanisms();
        }
    }
}
