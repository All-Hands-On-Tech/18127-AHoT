package org.firstinspires.ftc.teamcode.main;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.auxiliary.FlywheelAutoTuner;
import org.firstinspires.ftc.teamcode.config.RobotConstants;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;

@TeleOp(name = "Tuner: Flywheel PID", group = "Tuning")
public class FlywheelPidTunerTeleOp extends OpMode {
    private RobotHardware hardware;
    private FlywheelAutoTuner tuner;

    private double targetVelocity = RobotConstants.FLYWHEEL_TICKS_PER_SECOND;
    private boolean tuneButtonPrev = false;
    private boolean dpadUpPrev = false;
    private boolean dpadDownPrev = false;
    private boolean xButtonPrev = false;

    @Override
    public void init() {
        hardware = new RobotHardware(hardwareMap);
        tuner = new FlywheelAutoTuner(hardware);
        tuner.setTargetVelocityTicksPerSecond(targetVelocity);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    @Override
    public void loop() {
        if (gamepad1.a && !tuneButtonPrev) {
            tuner.setTuningEnabled(!tuner.isTuning());
        }
        tuneButtonPrev = gamepad1.a;

        if (gamepad1.dpad_up && !dpadUpPrev) {
            targetVelocity += 50.0;
        }
        if (gamepad1.dpad_down && !dpadDownPrev) {
            targetVelocity = Math.max(0.0, targetVelocity - 50.0);
        }
        dpadUpPrev = gamepad1.dpad_up;
        dpadDownPrev = gamepad1.dpad_down;

        if (gamepad1.x && !xButtonPrev) {
            targetVelocity = RobotConstants.FLYWHEEL_TICKS_PER_SECOND;
            tuner.setTargetVelocityTicksPerSecond(targetVelocity);
            tuner.setTuningEnabled(false);
            RobotConstants.FLYWHEEL_KP = 0.025;
            RobotConstants.FLYWHEEL_KI = 0.003;
            RobotConstants.FLYWHEEL_KD = 0.002;
            RobotConstants.FLYWHEEL_KF = 0.0;
            tuner.applyCurrentCoefficients();
        }
        xButtonPrev = gamepad1.x;

        tuner.setTargetVelocityTicksPerSecond(targetVelocity);
        if (tuner.isTuning()) {
            tuner.update();
        } else {
            if (hardware.leftFlywheel != null) {
                hardware.leftFlywheel.setVelocity(targetVelocity);
            }
            if (hardware.rightFlywheel != null) {
                hardware.rightFlywheel.setVelocity(targetVelocity);
            }
        }

        double leftVelocity = hardware.leftFlywheel != null ? hardware.leftFlywheel.getVelocity() : 0.0;
        double rightVelocity = hardware.rightFlywheel != null ? hardware.rightFlywheel.getVelocity() : 0.0;
        double averageVelocity = tuner.getAverageVelocity();
        double velocityError = targetVelocity - averageVelocity;

        // Keep these numeric so Panels can plot them directly on a live graph.
        telemetry.addData("graph/flywheel_target_ticks_per_s", targetVelocity);
        telemetry.addData("graph/flywheel_left_ticks_per_s", leftVelocity);
        telemetry.addData("graph/flywheel_right_ticks_per_s", rightVelocity);
        telemetry.addData("graph/flywheel_avg_ticks_per_s", averageVelocity);
        telemetry.addData("graph/flywheel_error_ticks_per_s", velocityError);

        telemetry.addData("Mode", tuner.isTuning() ? "AUTO-PID TUNE" : "HOLD TARGET");
        telemetry.addData("Target vel", String.format("%.1f ticks/s", targetVelocity));
        telemetry.addData("Actual vel", String.format("%.1f ticks/s", averageVelocity));
        telemetry.addData("Error", String.format("%.1f ticks/s", velocityError));
        telemetry.addData("KP", String.format("%.5f", RobotConstants.FLYWHEEL_KP));
        telemetry.addData("KI", String.format("%.5f", RobotConstants.FLYWHEEL_KI));
        telemetry.addData("KD", String.format("%.5f", RobotConstants.FLYWHEEL_KD));
        telemetry.addData("KF", String.format("%.5f", RobotConstants.FLYWHEEL_KF));
        telemetry.addData("Best error", String.format("%.1f ticks/s", tuner.getBestError()));
        telemetry.addData("Gamepad", "A = run auto tune / D-pad +/- target / X = reset gains");
        telemetry.update();
    }

    @Override
    public void stop() {
        if (hardware != null) {
            hardware.stopMechanisms();
        }
    }
}
