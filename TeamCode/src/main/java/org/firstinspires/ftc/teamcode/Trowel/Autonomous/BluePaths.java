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
            Depo1 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(22.903, 128.515),

                                    new Pose(43.622, 103.643)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(145), Math.toRadians(136))

                    .build();

            IntakeStart1 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(43.622, 103.643),

                                    new Pose(46.571, 89.352)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(0))

                    .build();

            IntakeEnd1 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(46.571, 89.352),

                                    new Pose(12.653, 89.287)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))

                    .build();

            Depo2 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(12.653, 89.287),

                                    new Pose(43.909, 104.097)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(136))

                    .build();

            IntakeStart2 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(43.909, 104.097),
                                    new Pose(59.276, 83.552),
                                    new Pose(42.402, 61.524)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))

                    .build();

            IntakeEnd2 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(42.402, 61.524),

                                    new Pose(6.189, 60.874)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))

                    .build();

            Depo3 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(6.189, 60.874),
                                    new Pose(42.502, 69.205),
                                    new Pose(43.725, 103.970)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(136))

                    .build();

            IntakeStart3 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(43.725, 103.970),

                                    new Pose(42.379, 40.944)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(0))

                    .build();

            IntakeEnd3 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(42.379, 40.944),

                                    new Pose(6.567, 41.323)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))

                    .build();

            Depo4 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(6.567, 41.323),

                                    new Pose(43.961, 103.844)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(136))

                    .build();

            Gate = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(43.961, 103.844),

                                    new Pose(26.140, 72.035)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(90))

                    .build();
        }
    }

    // PASTE ABOVE THIS LINE ════════════════════════════════════

}