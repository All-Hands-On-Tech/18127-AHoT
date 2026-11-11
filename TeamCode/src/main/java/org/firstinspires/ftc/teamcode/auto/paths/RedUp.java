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

    private final Pose start = poseFactory.of(62.2353, 8.3258, 90);
    private final Pose path1 = poseFactory.of(59.9321, 18.437, 90);
    private final Pose point2 = poseFactory.of(20.75, 46.75, 0);
    private final Pose point2Segment2Start = poseFactory.of(20.75, 46.75, -35.8519);
    private final Pose point2Segment2End = poseFactory.of(20.75, 46.75, 0);
    private final Pose point3 = poseFactory.of(17.22, 46.846, 0);
    private final Pose point3Segment1Start = poseFactory.of(17.22, 46.846, 0);
    private final Pose point3Segment1End = poseFactory.of(17.22, 46.846, 90);
    private final Pose point3Segment2Start = poseFactory.of(17.22, 46.846, 0);
    private final Pose point3Segment2End = poseFactory.of(17.22, 46.846, 0);
    private final Pose point4 = poseFactory.of(56.75, 118.75, -87.9569);
    private final Pose point4Control1 = poseFactory.of(30.1256, 44.2578, 0);
    private final Pose point4Control2 = poseFactory.of(0.3983, 103.0905, 0);
    private final Pose point4Segment2Target = poseFactory.of(57.99, 83.99, 0);
    private final Pose point5Start = poseFactory.of(56.75, 118.75, 270);
    private final Pose point5 = poseFactory.of(46.75, 121.0325, 270);
    private final Pose point5Control1 = poseFactory.of(45.7802, 105.0261, 0);
    private final Pose point6Start = poseFactory.of(46.75, 121.0325, 270);
    private final Pose point6 = poseFactory.of(56.75, 118.75, 270);
    private final Pose point6Control1 = poseFactory.of(49.3613, 119.1142, 0);
    private final Pose point7 = poseFactory.of(10.4108, 118.7803, 179.9625);
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
        return Paths.curve(point3, point4Control1, point4Control2, point4).heading(Interpolator.piecewise().until(0.8469, Interpolator.tangent).until(1, Interpolator.facingPoint(point4Segment2Target)));
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
                // move and shoot
                Auxiliaries.shooterOn(0.0, 0.1, true),
                PedroCommands.follow(follower, path1()),
                Auxiliaries.shoot(0.0, 3.0),

                // grab from flower
                Auxiliaries.flowerRampDown(0.0, 1.3, true),
                PedroCommands.follow(follower, path2()),
                PedroCommands.follow(follower, path3()),
                Auxiliaries.intakeOn(0.0, 3.5, true, false),
                Auxiliaries.intakeOff(0.0, true),
                Auxiliaries.flowerRampUp(0.0, 1.0, true),

                // shoot on other side
                PedroCommands.follow(follower, path4()),
                Auxiliaries.shoot(0.0, 3.0),

                // grab again
                Auxiliaries.flowerRampDown(0.0, 1.0, true),
                PedroCommands.follow(follower, path5()),
                Auxiliaries.intakeOn(0.0, 3.0, false, true),
                Auxiliaries.intakeOff(0.0, true),
                Auxiliaries.flowerRampUp(0.0, 1.0, true),

                // last shot, park
                PedroCommands.follow(follower, path6()),
                Auxiliaries.shoot(0.0, 3.0),
                Auxiliaries.shooterOff(0.0),
                PedroCommands.follow(follower, path7())
        );
    }
}
