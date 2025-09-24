package org.firstinspires.ftc.teamcode.pedroPathing.control;

/**
 * This is the KalmanFilter class. This creates a Kalman filter that is used to smooth out data.
 *
 * @author Anyi Lin - 10158 Scott's Bots
 * @version 1.0, 7/17/2024
 */
public class KalmanFilter implements NoiseFilter {
    private KalmanFilterParameters parameters;
    private double state;
    private double variance;
    private double kalmanGain;
    private double previousState;
    private double previousVariance;

    /**
     * This creates a new KalmanFilter from a set of KalmanFilterParameters.
     * @param parameters the parameters to use.
     */
    public KalmanFilter(KalmanFilterParameters parameters) {
        this.parameters = parameters;
        reset();
        previousState = state;
        previousVariance = variance;
    }

    /**
     * This method updates the Kalman filter with a new measurement.
     * @param updateProjection the new measurement to incorporate into the filter.
     */
    public void update(double updateProjection) {
        kalmanGain = previousVariance / (previousVariance + parameters.getMeasurementNoise());
        state = previousState + kalmanGain * (updateProjection - previousState);
        variance = previousVariance * (1.0 - kalmanGain);
        previousState = state;
        previousVariance = variance;
    }

    public double getState() {
        return state;
    }

    /**
     * This method outputs the current state, variance, and Kalman gain of the filter as a string array.
     * @return A string array containing the current state, variance, and Kalman gain.
     */
    public String[] output() {
        return new String[]{
                "State: " + state,
                "Variance: " + variance,
                "Kalman Gain: " + kalmanGain
        };
    }

    /**
     * This method resets the Kalman filter's state, variance, kalmanGain, previousState, and previousVariance
     * to their default values.
     */
    public void reset() {
        state = 0.0;
        variance = 1.0;
        kalmanGain = 0.0;
        previousState = 0.0;
        previousVariance = 1.0;
    }

    @Override
    public void reset(double startState, double startVariance, double startGain) {
        state = startState;
        variance = startVariance;
        kalmanGain = startGain;
        previousState = startState;
        previousVariance = startVariance;
    }

    @Override
    public void update(double updateData, double updateProjection) {
        // Use updateProjection for the filter update, ignore updateData for compatibility
        update(updateProjection);
    }
}
