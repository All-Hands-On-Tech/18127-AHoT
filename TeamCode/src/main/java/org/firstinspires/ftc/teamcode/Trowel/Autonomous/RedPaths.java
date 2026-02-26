package org.firstinspires.ftc.teamcode.Trowel.Autonomous;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

// ═══════════════════════════════════════════════════════════════
// Copy the ENTIRE visualizer output and paste it below,
// replacing everything between the PASTE markers.
// DO NOT MODIFY ANYTHING. Just paste and save.
// ═══════════════════════════════════════════════════════════════

public class RedPaths {

    // PASTE BELOW THIS LINE ════════════════════════════════════

    public static class Paths {
        public PathChain Depo1;
        public PathChain IntakeStart1;
        public PathChain IntakeEnd1;
        public PathChain Depo2;
        public PathChain IntakeStart2;
        public PathChain IntakeEnd2;
        public PathChain Depo3;
        public PathChain IntakeStart3;
        public PathChain IntakeEnd3;
        public PathChain Depo4;
        public PathChain Gate;

        public Paths(Follower follower) {
            Depo1 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(120.741, 127.624),
                                    new Pose(96.796, 105.152)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(35), Math.toRadians(44))
                    .build();

            IntakeStart1 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(96.796, 105.152),
                                    new Pose(97.608, 88.818)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(180))
                    .build();

            IntakeEnd1 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(97.608, 88.818),
                                    new Pose(130.990, 88.574)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            Depo2 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(130.990, 88.574),
                                    new Pose(96.619, 105.455)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(44))
                    .build();

            IntakeStart2 = follower.pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(96.619, 105.455),
                                    new Pose(84.724, 83.552),
                                    new Pose(95.258, 65.750)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            IntakeEnd2 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(95.258, 65.750),
                                    new Pose(138.497, 64.524)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            Depo3 = follower.pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(138.497, 64.524),
                                    new Pose(101.498, 69.205),
                                    new Pose(96.803, 105.178)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(44))
                    .build();

            IntakeStart3 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(96.803, 105.178),
                                    new Pose(101.621, 43.661)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(180))
                    .build();

            IntakeEnd3 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(101.621, 43.661),
                                    new Pose(137.762, 44.439)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            Depo4 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(137.762, 44.439),
                                    new Pose(96.416, 105.052)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(44))
                    .build();

            Gate = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(96.416, 105.052),
                                    new Pose(117.860, 72.035)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(90))
                    .build();
        }
    }
}