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

public class BluePaths {

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
                                    new Pose(23.259, 127.624),
                                    new Pose(47.204, 105.152)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(145), Math.toRadians(136))
                    .build();

            IntakeStart1 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(47.204, 105.152),
                                    new Pose(46.392, 88.818)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(0))
                    .build();

            IntakeEnd1 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(46.392, 88.818),
                                    new Pose(13.010, 88.574)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            Depo2 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(13.010, 88.574),
                                    new Pose(47.381, 105.455)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(136))
                    .build();

            IntakeStart2 = follower.pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(47.381, 105.455),
                                    new Pose(59.276, 83.552),
                                    new Pose(48.742, 65.750)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            IntakeEnd2 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(48.742, 65.750),
                                    new Pose(5.503, 64.524)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            Depo3 = follower.pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(5.503, 64.524),
                                    new Pose(42.502, 69.205),
                                    new Pose(47.197, 105.178)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(136))
                    .build();

            IntakeStart3 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(47.197, 105.178),
                                    new Pose(42.379, 43.661)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(0))
                    .build();

            IntakeEnd3 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(42.379, 43.661),
                                    new Pose(6.238, 44.439)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            Depo4 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(6.238, 44.439),
                                    new Pose(47.584, 105.052)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(136))
                    .build();

            Gate = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(47.584, 105.052),
                                    new Pose(26.140, 72.035)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(90))
                    .build();
        }
    }

    // PASTE ABOVE THIS LINE ════════════════════════════════════

}