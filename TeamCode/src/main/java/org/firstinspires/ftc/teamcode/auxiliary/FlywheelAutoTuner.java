package org.firstinspires.ftc.teamcode.auxiliary;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.config.RobotConstants;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;

public class FlywheelAutoTuner {
    private final DcMotorEx leftFlywheel;
    private final DcMotorEx rightFlywheel;
    private final ElapsedTime loopTimer = new ElapsedTime();

    private double targetVelocityTicksPerSecond = RobotConstants.FLYWHEEL_TICKS_PER_SECOND;
    private double kP = RobotConstants.FLYWHEEL_KP;
    private double kI = RobotConstants.FLYWHEEL_KI;
    private double kD = RobotConstants.FLYWHEEL_KD;
    private double kF = RobotConstants.FLYWHEEL_KF;

    private double lastError = 0.0;
    private double integral = 0.0;
    private int stableSampleCount = 0;
    private boolean tuning = false;
    private double bestError = Double.POSITIVE_INFINITY;

    public FlywheelAutoTuner(RobotHardware hardware) {
        this.leftFlywheel = hardware.leftFlywheel;
        this.rightFlywheel = hardware.rightFlywheel;
        if (leftFlywheel != null || rightFlywheel != null) {
            applyCurrentCoefficients();
        }
    }

    public void setTargetVelocityTicksPerSecond(double targetVelocityTicksPerSecond) {
        this.targetVelocityTicksPerSecond = targetVelocityTicksPerSecond;
    }

    public double getTargetVelocityTicksPerSecond() {
        return targetVelocityTicksPerSecond;
    }

    public void setTuningEnabled(boolean enabled) {
        tuning = enabled;
        if (tuning) {
            lastError = 0.0;
            integral = 0.0;
            stableSampleCount = 0;
            bestError = Double.POSITIVE_INFINITY;
            loopTimer.reset();
        }
    }

    public boolean isTuning() {
        return tuning;
    }

    public void update() {
        if (!tuning || (leftFlywheel == null && rightFlywheel == null)) {
            return;
        }

        double averageVelocity = getAverageVelocity();
        double dt = Math.max(loopTimer.seconds(), 0.01);
        loopTimer.reset();

        double error = targetVelocityTicksPerSecond - averageVelocity;
        integral = clamp(integral + error * dt, -20000.0, 20000.0);
        double derivative = (error - lastError) / dt;
        lastError = error;
        double absoluteError = Math.abs(error);
        bestError = Math.min(bestError, absoluteError);

        if (absoluteError < 15.0) {
            stableSampleCount++;
        } else {
            stableSampleCount = 0;
        }

        if (absoluteError > 40.0) {
            if (error > 0.0) {
                kP = clamp(kP + 0.0009 * (absoluteError / Math.max(targetVelocityTicksPerSecond, 1.0)), 0.003, 0.30);
                kI = clamp(kI + 0.00005 * (absoluteError / Math.max(targetVelocityTicksPerSecond, 1.0)), 0.0, 0.02);
            } else {
                kP = clamp(kP - 0.0006 * (absoluteError / Math.max(targetVelocityTicksPerSecond, 1.0)), 0.003, 0.30);
                kD = clamp(kD + 0.0008, 0.0, 0.04);
            }
        } else {
            kP = clamp(kP * 0.997, 0.003, 0.30);
            kI = clamp(kI * 0.995, 0.0, 0.02);
            kD = clamp(kD * 0.999, 0.0, 0.04);
        }

        if (Math.abs(derivative) > 25.0) {
            kD = clamp(kD + 0.0005, 0.0, 0.04);
            kP = clamp(kP - 0.0002, 0.003, 0.30);
        }

        if (Math.abs(error) < 8.0 && Math.abs(derivative) < 8.0 && stableSampleCount > 8) {
            tuning = false;
            kP = clamp(kP + 0.001, 0.003, 0.30);
            kI = clamp(kI + 0.00015, 0.0, 0.02);
            kD = clamp(kD + 0.0002, 0.0, 0.04);
            applyCurrentCoefficients();
            RobotConstants.FLYWHEEL_KP = kP;
            RobotConstants.FLYWHEEL_KI = kI;
            RobotConstants.FLYWHEEL_KD = kD;
            RobotConstants.FLYWHEEL_KF = kF;
            return;
        }

        applyCurrentCoefficients();
        leftFlywheel.setVelocity(targetVelocityTicksPerSecond);
        rightFlywheel.setVelocity(targetVelocityTicksPerSecond);
    }

    public void applyCurrentCoefficients() {
        PIDFCoefficients coefficients = new PIDFCoefficients(kP, kI, kD, kF);
        if (leftFlywheel != null) {
            leftFlywheel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, coefficients);
        }
        if (rightFlywheel != null) {
            rightFlywheel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, coefficients);
        }

        RobotConstants.FLYWHEEL_KP = kP;
        RobotConstants.FLYWHEEL_KI = kI;
        RobotConstants.FLYWHEEL_KD = kD;
        RobotConstants.FLYWHEEL_KF = kF;
    }

    public double getAverageVelocity() {
        double leftVelocity = leftFlywheel != null ? leftFlywheel.getVelocity() : 0.0;
        double rightVelocity = rightFlywheel != null ? rightFlywheel.getVelocity() : 0.0;
        if (leftFlywheel != null && rightFlywheel != null) {
            return (leftVelocity + rightVelocity) * 0.5;
        }
        return Math.max(leftVelocity, rightVelocity);
    }

    public double getCurrentP() {
        return kP;
    }

    public double getCurrentI() {
        return kI;
    }

    public double getCurrentD() {
        return kD;
    }

    public double getCurrentF() {
        return kF;
    }

    public double getLatestError() {
        return lastError;
    }

    public double getBestError() {
        return bestError;
    }

    private static double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }
}
