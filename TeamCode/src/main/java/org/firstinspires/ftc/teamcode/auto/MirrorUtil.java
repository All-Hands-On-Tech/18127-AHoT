package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;

public final class MirrorUtil {
    private MirrorUtil() {
    }

    public static Pose mirrorPose(Pose pose) {
        return PoseFactory.degrees().of(
                -pose.x(),
                -pose.y(),
                mirrorDegrees(Math.toDegrees(pose.heading())));
    }

    public static double mirrorDegrees(double headingDegrees) {
        double mirrored = -headingDegrees;
        while (mirrored > 180.0) {
            mirrored -= 360.0;
        }
        while (mirrored <= -180.0) {
            mirrored += 360.0;
        }
        return mirrored;
    }

    public static Pose mirrorPose(double x, double y, double headingDegrees) {
        return PoseFactory.degrees().of(
                -x,
                -y,
                mirrorDegrees(headingDegrees));
    }
}
