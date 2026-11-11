package org.firstinspires.ftc.teamcode.auto.paths;

import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.pedro.PedroCommands;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

import org.firstinspires.ftc.teamcode.auto.Auxiliaries;
import org.firstinspires.ftc.teamcode.auto.PedroAuto;

import static com.pedropathing.ivy.groups.Groups.sequential;

/**
 * Red alliance, down side auto.
 */
public final class RedDown implements PedroAuto.AutoPath {

    public static RedDown mirrored() {
        return new RedDown();
    }

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(59.461, 133.1576, -90);
    private final Pose path1 = poseFactory.of(58.8689, 114.4093, -90);
    private final Pose point2Start = poseFactory.of(58.8689, 114.4093, -90);
    private final Pose point2 = poseFactory.of(46.9998, 131.3633, -90);
    private final Pose point2Control1 = poseFactory.of(45.0086, 115.6437, 0);
    private final Pose point3Start = poseFactory.of(46.9998, 131.3633, -90);
    private final Pose point3 = poseFactory.of(58.6511, 115.0465, -90);
    private final Pose point4 = poseFactory.of(9.3409, 114.8209, 0.2621);

    public Path path1() {
        return Paths.line(start, path1).linear(start, path1);
    }

    public Path path2() {
        return Paths.curve(point2Start, point2Control1, point2).linear(point2Start, point2);
    }

    public Path path3() {
        return Paths.line(point3Start, point3).linear(point3Start, point3);
    }

    public Path path4() {
        return Paths.line(point3, point4).reverseTangent();
    }

    @Override
    public Pose startPose() {
        return start;
    }

    @Override
    public Command plan(Follower follower) {
        return sequential(
                Auxiliaries.shooterOn(0.0, 0.0, false),
                PedroCommands.follow(follower, path1()),
                Auxiliaries.shoot(0.0, 3.0),

                Auxiliaries.flowerRampDown(0.0, 1.0, true),
                PedroCommands.follow(follower, path2()),
                Auxiliaries.intakeOn(0.0, 3.5, true, false),
                Auxiliaries.intakeOff(0.0, true),

                Auxiliaries.flowerRampUp(0.4, 1.0, true),
                PedroCommands.follow(follower, path3()),
                Auxiliaries.shoot(0.0, 3.0),

                Auxiliaries.shooterOff(0.0),
                PedroCommands.follow(follower, path4())
        );
    }
}
