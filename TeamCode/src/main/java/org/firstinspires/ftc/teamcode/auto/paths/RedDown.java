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

    private final Pose start = poseFactory.of(58.687, 133.042, 270);
    private final Pose path1 = poseFactory.of(59.0709, 125.3432, 270);
    private final Pose point2Start = poseFactory.of(59.0709, 125.3432, 270);
    private final Pose point2 = poseFactory.of(46.4229, 125.306, 270);
    private final Pose point2Control1 = poseFactory.of(45.9289, 101.8552, 0);
    private final Pose point3Start = poseFactory.of(46.4229, 125.306, 270);
    private final Pose point3 = poseFactory.of(58.0347, 124.8261, 270);
    private final Pose point3Control1 = poseFactory.of(48.7296, 109.4164, 0);
    private final Pose point4 = poseFactory.of(11.0071, 120.3111, 172.5992);
    private final Pose point4Control1 = poseFactory.of(45.93, 115.7749, 0);

    public Path path1() {
        return Paths.line(start, path1).linear(start, path1);
    }

    public Path path2() {
        return Paths.curve(point2Start, point2Control1, point2).linear(point2Start, point2);
    }

    public Path path3() {
        return Paths.curve(point3Start, point3Control1, point3).linear(point3Start, point3);
    }

    public Path path4() {
        return Paths.curve(point3, point4Control1, point4).tangent();
    }

    @Override
    public Pose startPose() {
        return start;
    }

    @Override
    public Command plan(Follower follower) {
        return sequential(
                // wait, spin up, move to the first shot
                Auxiliaries.wait(5.0),
                Auxiliaries.shooterOn(0.0, 5.0, false),
                PedroCommands.follow(follower, path1()),
                Auxiliaries.shoot(0.0, 3.0, 0.2),

                // flower down, drive to the flower, intake
                Auxiliaries.flowerRampDown(0.0, 1.0, true),
                PedroCommands.follow(follower, path2()),
                Auxiliaries.intakeOn(0.0, 10.5, true, false, 0.0),
                Auxiliaries.intakeCarry(),                       // keep balls held while driving

                // drive to the shooting spot, gate opens only now
                PedroCommands.follow(follower, path3()),
                Auxiliaries.shoot(0.0, 3.0, 0.0),

                // turn everything off, then park
                Auxiliaries.intakeOff(0.0, true),
                Auxiliaries.shooterOff(0.0),
                PedroCommands.follow(follower, path4())
        );
    }
}
