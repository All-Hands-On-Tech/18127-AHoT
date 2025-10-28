package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit; // added

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import org.firstinspires.ftc.teamcode.ZSupport.RAndDBotUtilities;

@TeleOp(name="RAndDBotTeleOp", group="Z")
public class RAndDBotTeleOp extends LinearOpMode {
    RAndDBotUtilities bot = new RAndDBotUtilities(this);

    private double prevY = 0.0;
    private double prevX = 0.0;

    private double prevElapsedTime = 0.0;

    private double targetHeadingDeg = 0.0;


    boolean traditionalDrivetrain = true;
    boolean prevBack = false;

    private final Pose2D START_POSE = new Pose2D(DistanceUnit.INCH, 36, -63.667, AngleUnit.DEGREES,0);
    private final Pose2D BLUE_BACKBOARD_POSE = new Pose2D(DistanceUnit.INCH, -65, 70, AngleUnit.DEGREES, 0);
    private final Pose2D BLUE_PARKING_POSE = new Pose2D(DistanceUnit.INCH, 40, -40, AngleUnit.DEGREES, 0);


    @Override
    public void runOpMode()
    {
        bot.initialize(this);


        if (isStopRequested()) return;
        waitForStart();

        // initialize target heading to current at start
        bot.setPoseEstimate(START_POSE);
        targetHeadingDeg = getHeading();

        while(opModeIsActive())
        {
            double deltaRuntime = getRuntime() - prevElapsedTime;
            if (deltaRuntime <= 0) deltaRuntime = 0.02; // guard

            bot.updateLocalization();

            double y  = Math.abs(gamepad1.left_stick_y)  > 0.05 ? -gamepad1.left_stick_y  : 0; // Forward/backward strafe
            double x  = Math.abs(gamepad1.left_stick_x)  > 0.05 ? gamepad1.left_stick_x  : 0; // Left/right
            double rx = Math.abs(gamepad1.right_stick_x) > 0.05 ? -gamepad1.right_stick_x : 0; // Rotation

            double speedMultiplier = gamepad1.left_bumper ? 0.5 : 1.0;
            y  *= speedMultiplier;
            x  *= speedMultiplier;
            rx *= speedMultiplier;


            /*//////////////////////////


            THIS IS DRIVETRAIN CODE


             //////////////////////////*/
            if (prevBack && !gamepad1.back) {
                traditionalDrivetrain = !traditionalDrivetrain; // toggle
                targetHeadingDeg = getHeading();
            }
            prevBack = gamepad1.back;

            if(traditionalDrivetrain) {
                y *= 1.1;
                bot.move(x, -y, rx);
            }else {
                double goalAngle = headingToPoint(BLUE_BACKBOARD_POSE.getX(DistanceUnit.INCH), BLUE_BACKBOARD_POSE.getY(DistanceUnit.INCH));
                telemetry.addData("Goal Angle: ", goalAngle);

                // === Heading hold using squidToHeading when driver isn't rotating ===

                final double rotateDeadband = 0.02;
                if(!gamepad1.a){

                    final double maxDegPerSec  = 360.0;        // full-stick = 180°/s target change

                    if (Math.abs(rx) > rotateDeadband) {
                        targetHeadingDeg = targetHeadingDeg + rx * maxDegPerSec * deltaRuntime;
                    }

                    rx = bot.squidToHeading(targetHeadingDeg);


                }else{
                    rx = bot.squidToHeading(goalAngle);
                }

                // Slew limit translation AFTER all scaling/heading-hold logic (unchanged)
                if (y < 0.8) {
                    y = slew(y, prevY, 4 * deltaRuntime);
                }
                if (x < 0.8) {
                    x = slew(x, prevX, 4 * deltaRuntime);
                }
                // rx slew already handled conditionally above

                prevY = y;
                prevX = x;
                prevElapsedTime = getRuntime();

                bot.moveFieldOriented(x, y, rx);





            }

            bot.logDriveData(bot.odo.getPosition());
            telemetry.addLine();telemetry.addLine();telemetry.addLine();
            bot.logPinpointFrequency();
            bot.logREVHubFrequency();
            telemetry.addData("Joystick", "Y: %.2f X: %.2f RX: %.2f", y, x, rx);
            telemetry.addData("Heading Hold", "target=%.1f°, cur=%.1f°", targetHeadingDeg, getHeading());
            telemetry.update();

        }
    }

    // Limits how fast a value can change per loop
    double slew(double target, double prev, double maxDelta) {
        double delta = target - prev;
        if(Math.abs(prev) > Math.abs(delta)) return target;
        if (delta > maxDelta)  return prev + maxDelta;
        if (delta < -maxDelta) return prev - maxDelta;
        return target;
    }


    private double headingToPoint(double x, double y)
    {
        Pose2D currentPose = bot.odo.getPosition();
        double currentX = currentPose.getX(DistanceUnit.INCH);
        double currentY = currentPose.getY(DistanceUnit.INCH);

        double dX = x - currentX;
        double dY = y - currentY;

        double deg = Math.toDegrees(customArctan(dY, dX));
        double band = (Math.floor(getHeading() / 360.0)) * 360.0;

        return deg + band;
    }

    //same as atan2 but domain is (-90, 270)
    private double customArctan(double y, double x)
    {
        double angle = Math.atan2(y,x);
        if(angle < -90 && angle >= -180)
        {
            angle = 360+angle;
        }
        return angle;
    }

    private double getHeading()
    {
        return bot.odo.getHeading(UnnormalizedAngleUnit.DEGREES);
    }

}
