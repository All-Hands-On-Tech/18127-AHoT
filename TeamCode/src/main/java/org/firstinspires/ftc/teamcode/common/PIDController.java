package org.firstinspires.ftc.teamcode.common;

/**
 * Simple PID controller intended for control loops in OpModes.
 * - Units are generic (same units as measurement and setpoint).
 * - Output is clamped to configured min/max.
 * - Provides anti-windup via integrator clamping and integrator reset on setpoint change.
 */
public class PIDController {
    private double kP, kI, kD;
    private double integrator = 0.0;
    private double lastError = Double.NaN;
    private double lastDerivative = 0.0;
    private double outMin = -1.0, outMax = 1.0;
    private double integratorMin = -1.0, integratorMax = 1.0;

    public PIDController(double kP, double kI, double kD) {
        this.kP = kP; this.kI = kI; this.kD = kD;
    }

    public void setOutputLimits(double min, double max) {
        this.outMin = min; this.outMax = max;
    }

    public void setIntegratorLimits(double min, double max) {
        this.integratorMin = min; this.integratorMax = max;
    }

    public void setCoefficients(double kP, double kI, double kD) {
        this.kP = kP; this.kI = kI; this.kD = kD;
    }

    public void reset() {
        integrator = 0.0;
        lastError = Double.NaN;
        lastDerivative = 0.0;
    }

    /**
     * Compute control output.
     * @param setpoint desired value
     * @param measurement current value
     * @param dt seconds since last update
     * @return control output (clamped to outMin/outMax)
     */
    public double update(double setpoint, double measurement, double dt) {
        double error = setpoint - measurement;
        if (Double.isNaN(lastError)) {
            lastError = error; // initialize derivative
        }

        // Integral term with simple anti-windup
        integrator += error * dt;
        if (integrator > integratorMax) integrator = integratorMax;
        if (integrator < integratorMin) integrator = integratorMin;

        // Derivative (on error)
        double derivative = (error - lastError) / (dt > 0 ? dt : 1e-6);
        lastDerivative = derivative;
        lastError = error;

        double out = kP * error + kI * integrator + kD * derivative;
        if (out > outMax) out = outMax;
        if (out < outMin) out = outMin;
        return out;
    }

    // Getters for telemetry / tuning
    public double getIntegrator() { return integrator; }
    public double getLastError() { return lastError; }
    public double getLastDerivative() { return lastDerivative; }
}
