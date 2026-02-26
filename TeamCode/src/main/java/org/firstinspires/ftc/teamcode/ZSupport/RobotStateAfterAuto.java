package org.firstinspires.ftc.teamcode.ZSupport;

import com.pedropathing.geometry.Pose;


public class RobotStateAfterAuto {
    public static Pose postAutoPose;
    public static int postAutoYawAngle;
    public static boolean wasAuto = false;
    private static void setPostAutoPose(Pose p){
        postAutoPose = p;
        wasAuto = true;
    }
    private static void setPostAutoYawTicks(int t){
        postAutoYawAngle = t;
        wasAuto = true;
    }
    public static void setPostAutoState(Pose p, int t){
        setPostAutoPose(p);
        setPostAutoYawTicks(t);
        wasAuto = true;
    }
}
