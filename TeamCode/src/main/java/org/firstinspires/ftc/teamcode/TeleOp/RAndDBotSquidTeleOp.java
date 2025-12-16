package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import org.firstinspires.ftc.teamcode.ZSupport.RAndDBotUtilities;

import java.util.Locale;

@Disabled
@TeleOp(name="RAndDBotSquidTeleOp", group="Z")
public class RAndDBotSquidTeleOp extends LinearOpMode {
    RAndDBotUtilities bot = new RAndDBotUtilities(this);

    private double prevElapsedTime = 0.0;


    private final Pose2D START_POSE = new Pose2D(DistanceUnit.INCH, 36, -63.667, AngleUnit.DEGREES,0);
    private final Pose2D BLUE_BACKBOARD_POSE = new Pose2D(DistanceUnit.INCH, -65, 70, AngleUnit.DEGREES, 0);
    private final Pose2D BLUE_PARKING_POSE = new Pose2D(DistanceUnit.INCH, 40, -40, AngleUnit.DEGREES, 0);

    private Pose2D targetPose = START_POSE;

    private final double MAX_IPS = 1.0;
    private final double MAX_DPS = 1.0;
    @Override
    public void runOpMode()
    {
        bot.initialize(this);


        if (isStopRequested()) return;
        waitForStart();

        // initialize target heading to current at start
        bot.setPoseEstimate(START_POSE);
        targetPose = new Pose2D(DistanceUnit.INCH, bot.odo.getPosX(DistanceUnit.INCH), bot.odo.getPosY(DistanceUnit.INCH), AngleUnit.DEGREES, bot.odo.getHeading(UnnormalizedAngleUnit.DEGREES) + 90);

        while(opModeIsActive())
        {
            double deltaRuntime = getRuntime() - prevElapsedTime;
            prevElapsedTime = getRuntime();
            if (deltaRuntime <= 0) deltaRuntime = 0.02; // guard

            bot.updateLocalization();

            double y  = Math.abs(gamepad1.left_stick_y)  > 0.05 ? -gamepad1.left_stick_y  : 0; // Forward/backward strafe
            double x  = Math.abs(gamepad1.left_stick_x)  > 0.05 ? gamepad1.left_stick_x  : 0; // Left/right
            double rx = Math.abs(gamepad1.right_stick_x) > 0.05 ? -gamepad1.right_stick_x : 0; // Rotation


            y *= MAX_IPS;
            x *= MAX_IPS;
            rx *= MAX_DPS;

//            targetPose = new Pose2D(DistanceUnit.INCH, targetPose.getX(DistanceUnit.INCH) + x*deltaRuntime, targetPose.getY(DistanceUnit.INCH) + y*deltaRuntime, AngleUnit.DEGREES, (targetPose.getHeading(AngleUnit.DEGREES))+rx*deltaRuntime);
            targetPose = new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0);

            telemetry.addData("XPow: ", bot.squidToX(targetPose.getX(DistanceUnit.MM)));
            telemetry.addData("YPow: ", bot.squidToY(targetPose.getY(DistanceUnit.MM)));

            if(gamepad1.a){
                bot.squidToPose(targetPose);

            }else if(gamepad1.b)
            {
                bot.squidToPose(BLUE_PARKING_POSE);
            }else if(gamepad1.x) {
                double xPow = bot.squidToX(targetPose.getX(DistanceUnit.MM));
                bot.moveFieldOriented(0, -xPow,0);
            }else if(gamepad1.y) {
                double yPow = bot.squidToY(targetPose.getY(DistanceUnit.MM));
                bot.moveFieldOriented(yPow, 0, 0);
            }else {
                bot.move(0,0,0);
            }

            bot.applyDrivePower();

            String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", targetPose.getX(DistanceUnit.INCH), targetPose.getY(DistanceUnit.INCH), targetPose.getHeading(AngleUnit.DEGREES));
            telemetry.addData("Target Position", data);

            bot.logDriveData(bot.odo.getPosition());
            telemetry.addLine();telemetry.addLine();telemetry.addLine();
            bot.logPinpointFrequency();
            bot.logREVHubFrequency();
            telemetry.addData("Joystick", "Y: %.2f X: %.2f RX: %.2f", y, x, rx);
            telemetry.update();

        }
    }



    private double getHeading()
    {
        return bot.odo.getHeading(UnnormalizedAngleUnit.DEGREES) + 90;
    }

}
