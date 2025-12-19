package org.firstinspires.ftc.teamcode.Trowel.common;

import org.firstinspires.ftc.teamcode.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.GoBildaPinpointDriver.DeviceStatus;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Trowel.Configs.TrowelHardware;

/**
 * Trowel-specific odometry wrapper.
 * Call update() each loop, then getPosition().
 * Units: X/Y millimeters, heading radians (also degrees accessor).
 */
public class Odometry {
    private final TrowelHardware hw;
    private GoBildaPinpointDriver pinpoint;
    private double xMm, yMm, hRad;

    public Odometry(TrowelHardware hw, GoBildaPinpointDriver pinpoint) {
        this.hw = hw;
        this.pinpoint = pinpoint;
        this.xMm = 0.0;
        this.yMm = 0.0;
        this.hRad = 0.0;
    }

    public void setPinpoint(GoBildaPinpointDriver p) { this.pinpoint = p; }

    public void update() {
        if (pinpoint != null && pinpoint.getDeviceStatus() == DeviceStatus.READY) {
            xMm = pinpoint.getPosX(DistanceUnit.MM);
            yMm = pinpoint.getPosY(DistanceUnit.MM);
            hRad = pinpoint.getHeading(AngleUnit.RADIANS);
        } else if (hw != null) {
            try {
                if (hw.imu != null) {
                    hRad = hw.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
                }
            } catch (Exception ignored) {
                // IMU access failed or not present; keep previous heading
            }
        }
    }

    public Position getPosition() { return new Position(xMm, yMm, hRad); }

    public static class Position {
        public final double xMm, yMm, headingRad;
        public Position(double xMm, double yMm, double headingRad) { this.xMm = xMm; this.yMm = yMm; this.headingRad = headingRad; }
        public double getXmm() { return xMm; }
        public double getYmm() { return yMm; }
        public double getHeadingRad() { return headingRad; }
        public double getHeadingDeg() { return Math.toDegrees(headingRad); }
        @Override public String toString() { return String.format(java.util.Locale.US, "X%.1f Y%.1f H%.1f°", xMm, yMm, getHeadingDeg()); }
    }
}
