package org.firstinspires.ftc.teamcode.common;

import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.robotcore.hardware.HardwareMap;

/** External Pedro Pathing configuration (replaces internal pedroPathing.Constants). */
public class PedroConfig {
    public static final FollowerConstants FOLLOWER_CONSTANTS = new FollowerConstants();
    // Tunable default constraints (match old internal values): maxVel, maxAccel, maxAngVel, maxAngAccel
    public static final PathConstraints DEFAULT_CONSTRAINTS = new PathConstraints(0.99, 100, 1, 1);

    public static Follower createFollower(HardwareMap hw) {
        return new FollowerBuilder(FOLLOWER_CONSTANTS, hw)
                .pathConstraints(DEFAULT_CONSTRAINTS)
                .build();
    }
}

