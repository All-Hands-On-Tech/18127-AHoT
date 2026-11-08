package org.firstinspires.ftc.teamcode.auto.paths;

import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.pedro.PedroCommands;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.interpolator.Interpolator;

import org.firstinspires.ftc.teamcode.auto.Auxiliaries;
import org.firstinspires.ftc.teamcode.auto.PedroAuto;

import static com.pedropathing.ivy.groups.Groups.sequential;

/**
 * Red alliance, up side auto.
 */
public final class RedUp implements PedroAuto.AutoPath {

    public static RedUp mirrored() {
        return new RedUp();
    }

    // =====================================================================
    // VISUALIZER OUTPUT
    // =====================================================================
    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(56, 8, 90);
    private final Pose path1 = poseFactory.of(56.393, 14.1854, 90);
    private final Pose point2 = poseFactory.of(17.394, 45.7667, 0);
    private final Pose point2Segment2Start = poseFactory.of(17.394, 45.7667, -39.0005);
    private final Pose point2Segment2End = poseFactory.of(17.394, 45.7667, 0);
    private final Pose point3 = poseFactory.of(10.416, 45.8892, 0);
    private final Pose point3Segment1Start = poseFactory.of(10.416, 45.8892, 0);
    private final Pose point3Segment1End = poseFactory.of(10.416, 45.8892, 90);
    private final Pose point3Segment2Start = poseFactory.of(10.416, 45.8892, 0);
    private final Pose point3Segment2End = poseFactory.of(10.416, 45.8892, 0);
    private final Pose point4Start = poseFactory.of(10.416, 45.8892, 0);
    private final Pose point4 = poseFactory.of(56.8948, 118.1623, 270);
    private final Pose point4Control1 = poseFactory.of(30.1256, 44.2578, 0);
    private final Pose point4Control2 = poseFactory.of(0.3983, 103.0905, 0);
    private final Pose point5Start = poseFactory.of(56.8948, 118.1623, 270);
    private final Pose point5 = poseFactory.of(46.184, 128.6766, 270);
    private final Pose point5Control1 = poseFactory.of(45.7802, 105.0261, 0);
    private final Pose point6Start = poseFactory.of(46.184, 128.6766, 270);
    private final Pose point6 = poseFactory.of(56.5509, 118.3097, 270);
    private final Pose point6Control1 = poseFactory.of(45.4268, 105.775, 0);
    private final Pose point7 = poseFactory.of(6.0924, 117.9167, -179.5537);

    public Path path1() {
        return Paths.line(start, path1).linear(start, path1);
    }

    public Path path2() {
        return Paths.line(path1, point2).heading(Interpolator.piecewise().until(0.8005, Interpolator.tangent.reverse()).until(1, Interpolator.linear(point2Segment2Start, point2Segment2End)));
    }

    public Path path3() {
        return Paths.line(point2, point3).heading(Interpolator.piecewise().until(0.0001, Interpolator.linear(point3Segment1Start, point3Segment1End).reverse()).until(1, Interpolator.linear(point3Segment2Start, point3Segment2End)));
    }

    public Path path4() {
        return Paths.curve(point4Start, point4Control1, point4Control2, point4).linear(point4Start, point4);
    }

    public Path path5() {
        return Paths.curve(point5Start, point5Control1, point5).linear(point5Start, point5);
    }

    public Path path6() {
        return Paths.curve(point6Start, point6Control1, point6).linear(point6Start, point6);
    }

    public Path path7() {
        return Paths.line(point6, point7).tangent();
    }
    @Override
    public Pose startPose() {
        return start;
    }

    @Override
    public Command plan(Follower follower) {
        return sequential(
                //move and shoot for the first time
                Auxiliaries.shooterOn(0.0, 0.5, 0.0, true),
                PedroCommands.follow(follower, path1()),
                Auxiliaries.shoot(0.0, 3.0),

                //grab from flower
                Auxiliaries.flowerRampDown(0.0, 1.0, 0.0, true),
                PedroCommands.follow(follower, path2()),
                PedroCommands.follow(follower, path3()),
                Auxiliaries.intakeOn(0.0, 3.5, 0.0, true),
                Auxiliaries.intakeOff(0.0, 1.0, 0.0, true),
                Auxiliaries.flowerRampUp(0.4, 1.0, 0.0, true),

                //go shoot on other side of field
                PedroCommands.follow(follower, path4()),
                Auxiliaries.shoot(0.0, 1.0),

                //grab from flower again
                Auxiliaries.flowerRampDown(0.0, 1.0, 0.0, true),
                PedroCommands.follow(follower, path5()),
                Auxiliaries.intakeOn(0.0, 3.0, 0.0, false),
                Auxiliaries.intakeOff(0.0, 1.0, 0.0, true),
                Auxiliaries.flowerRampUp(0.4, 1.0, 0.0, true),

                //shoot for the last time
                PedroCommands.follow(follower, path6()),
                Auxiliaries.shoot(0.0, 1.0),

                //park and be ready for teleop
                Auxiliaries.shooterOff(0.0),
                PedroCommands.follow(follower, path7())
        );
    }
}
