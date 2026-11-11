package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.auxiliary.Navigation;
import org.firstinspires.ftc.teamcode.auto.paths.RedDown;
import org.firstinspires.ftc.teamcode.auto.paths.RedUp;
import org.firstinspires.ftc.teamcode.pedro.Constants;

/**
 * Main autonomous OpMode.
 */
@Autonomous(name = "Auto: Pedro Main", group = "Pedro")
public class PedroAuto extends LinearOpMode {

    /** A single path plan with a start pose and command list. */
    public interface AutoPath {
        Pose startPose();

        Command plan(Follower follower);
    }

    private enum Side { UP, DOWN }

    private Follower follower;
    private AutoPath plan;
    private final ElapsedTime matchTimer = new ElapsedTime();
    private Side side = Side.UP;
    private boolean dpadUpPrev = false;
    private boolean dpadDownPrev = false;

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        follower = Constants.create(hardwareMap);
        Auxiliaries.init(hardwareMap);

        while (!isStarted() && !isStopRequested()) {
            Navigation.AllianceSelector.update(gamepad1, gamepad2);
            Navigation.AllianceSelector.addTelemetry(telemetry);

            if (gamepad1.dpad_up && !dpadUpPrev) {
                side = Side.UP;
            }
            if (gamepad1.dpad_down && !dpadDownPrev) {
                side = Side.DOWN;
            }
            dpadUpPrev = gamepad1.dpad_up;
            dpadDownPrev = gamepad1.dpad_down;

            telemetry.addData("Auto", planName(Navigation.AllianceSelector.get(), side));
            telemetry.addData("Setup", "X/B = alliance, D-pad up/down = side");
            telemetry.update();
            sleep(20);
        }

        if (isStopRequested()) {
            return;
        }

        // Stamp the start pose and schedule the whole plan.
        plan = pickPlan(Navigation.AllianceSelector.get(), side);
        follower.setPose(plan.startPose());
        Scheduler.schedule(plan.plan(follower));
        matchTimer.reset();

        while (opModeIsActive()) {
            follower.update();
            Scheduler.execute();

            Pose pose = follower.pose();
            telemetry.addData("Match time", "%.1f s", matchTimer.seconds());
            telemetry.addData("Pose", String.format("(%.1f in, %.1f in, %.1f deg)",
                    pose.x(), pose.y(), Math.toDegrees(pose.heading())));
            telemetry.addData("Busy", follower.isBusy() ? "yes" : "no");
            telemetry.update();
        }

        Scheduler.reset();
        Auxiliaries.shutdown();
        follower.stop();
    }

    private AutoPath pickPlan(Navigation.Alliance alliance, Side side) {
        if (alliance == Navigation.Alliance.RED) {
            return side == Side.UP ? new RedUp() : new RedDown();
        }
        return side == Side.UP ? RedUp.mirrored() : RedDown.mirrored();
    }

    private String planName(Navigation.Alliance alliance, Side side) {
        Navigation.Alliance a = alliance == null ? Navigation.Alliance.RED : alliance;
        return a.getDisplayName() + (side == Side.UP ? " Up" : " Down");
    }
}
