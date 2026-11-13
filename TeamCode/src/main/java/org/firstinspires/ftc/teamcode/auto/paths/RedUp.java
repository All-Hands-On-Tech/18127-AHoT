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

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(55.1339, 7.9419, 90);
    private final Pose path1 = poseFactory.of(56.5733, 14.4065, 90);
    private final Pose point2 = poseFactory.of(25.4523, 46.75, 0);
    private final Pose point2Segment2Start = poseFactory.of(25.4523, 46.75, -46.1035);
    private final Pose point2Segment2End = poseFactory.of(25.4523, 46.75, 0);
    private final Pose point3Start = poseFactory.of(25.4523, 46.75, 0);
    private final Pose point3 = poseFactory.of(17.8918, 46.846, 0);
    private final Pose point4 = poseFactory.of(59.9168, 122.9724, -90);
    private final Pose point4Control1 = poseFactory.of(30.1256, 44.2578, 0);
    private final Pose point4Control2 = poseFactory.of(0.3983, 103.0905, 0);
    private final Pose point4Segment2Start = poseFactory.of(59.9168, 122.9724, 29.8085);
    private final Pose point4Segment2End = poseFactory.of(59.9168, 122.9724, 270);
    private final Pose point5Start = poseFactory.of(59.9168, 122.9724, 270);
    private final Pose point5 = poseFactory.of(48.4774, 115.1786, 270);
    private final Pose point5Control1 = poseFactory.of(45.7802, 105.0261, 0);
    private final Pose point6Start = poseFactory.of(48.4774, 115.1786, 270);
    private final Pose point6 = poseFactory.of(48.1132, 123.5482, 270);
    private final Pose point7Start = poseFactory.of(48.1132, 123.5482, 270);
    private final Pose point7 = poseFactory.of(60.0979, 122.9068, 270);
    private final Pose point7Control1 = poseFactory.of(46.9518, 115.1029, 0);
    private final Pose point8 = poseFactory.of(12.618, 117.9167, 154.2501);
    private final Pose point8Control1 = poseFactory.of(39.5986, 104.9028, 0);

    public Path path1() {
        return Paths.line(start, path1).linear(start, path1);
    }

    public Path path2() {
        return Paths.line(path1, point2).heading(Interpolator.piecewise().until(0.8005, Interpolator.tangent.reverse()).until(1, Interpolator.linear(point2Segment2Start, point2Segment2End)));
    }

    public Path path3() {
        return Paths.line(point3Start, point3).linear(point3Start, point3);
    }

    public Path path4() {
        return Paths.curve(point3, point4Control1, point4Control2, point4).heading(Interpolator.piecewise().until(0.8469, Interpolator.tangent).until(1, Interpolator.linear(point4Segment2Start, point4Segment2End)));
    }

    public Path path5() {
        return Paths.curve(point5Start, point5Control1, point5).linear(point5Start, point5);
    }

    public Path path6() {
        return Paths.line(point6Start, point6).linear(point6Start, point6);
    }

    public Path path7() {
        return Paths.curve(point7Start, point7Control1, point7).linear(point7Start, point7);
    }

    public Path path8() {
        return Paths.curve(point7, point8Control1, point8).tangent();
    }

    @Override
    public Pose startPose() {
        return start;
    }

    @Override
    public Command plan(Follower follower) {
        return sequential(
                // spin the flywheel up and move to point 1
                Auxiliaries.shooterOn(0.0, 2, true),
                PedroCommands.follow(follower, path1()),
                Auxiliaries.shoot(0.0, 2.5, 0.35),

                // point 2, put the flower ramp down, point 3
                PedroCommands.follow(follower, path2()),
                Auxiliaries.flowerRampDown(0.0, 1.0, true),
                PedroCommands.follow(follower, path3()),
                Auxiliaries.intakeOn(0.0, 2.5, true, false, 0.0),
                Auxiliaries.intakeCarry(),                       // keep balls held while driving
                Auxiliaries.flowerRampUp(1.2, 1.0, true),

                // point 4, shoot (gate opens only now)
                PedroCommands.follow(follower, path4()),
                Auxiliaries.shoot(0.0, 2.5, 0.0),

                // points 5 and 6, intake
                PedroCommands.follow(follower, path5()),
                PedroCommands.follow(follower, path6()),
                Auxiliaries.intakeOn(0.0, 2.5, false, false, 0.0),
                Auxiliaries.intakeCarry(),                       // keep balls held while driving

                // point 7, shoot
                PedroCommands.follow(follower, path7()),
                Auxiliaries.shoot(0.0, 2.5, 0.0),

                // turn everything off, then park at point 8
                Auxiliaries.shooterOff(0.0),
                Auxiliaries.intakeOff(0.0, true),
                PedroCommands.follow(follower, path8())
        );
    }
}