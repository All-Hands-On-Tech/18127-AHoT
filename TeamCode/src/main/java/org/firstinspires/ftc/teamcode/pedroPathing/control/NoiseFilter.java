package org.firstinspires.ftc.teamcode.pedroPathing.control;

public interface NoiseFilter {
    void update(double updateData, double updateProjection);
    double getState();
    default void reset() {
        reset(0, 1, 1);
    }
    void reset(double startState, double startVariance, double startGain);
}

