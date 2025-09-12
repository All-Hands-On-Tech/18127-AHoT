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
    private double prevHeading = 0.0;
    private double currentCumulativeHeading = 0.0;


    boolean traditionalDrivetrain = true;
    boolean prevBack = false;


    @Override
    public void runOpMode()
    {
        bot.initialize(this);

        if (isStopRequested()) return;
        waitForStart();

        // initialize target heading to current at start
        targetHeadingDeg = bot.odo.getHeading(AngleUnit.DEGREES);

        while(opModeIsActive())
        {
            double deltaRuntime = getRuntime() - prevElapsedTime;
            if (deltaRuntime <= 0) deltaRuntime = 0.02; // guard

            bot.updateLocalization();
            double headingDeg = bot.odo.getHeading(AngleUnit.DEGREES);
            currentCumulativeHeading += headingDeg - prevHeading;


            double y  = Math.abs(gamepad1.left_stick_y)  > 0.05 ? gamepad1.left_stick_y  : 0; // Forward/backward strafe
            double x  = Math.abs(gamepad1.left_stick_x)  > 0.05 ? -gamepad1.left_stick_x  : 0; // Left/right
            double rx = Math.abs(gamepad1.right_stick_x) > 0.05 ? -gamepad1.right_stick_x : 0; // Rotation

            double speedMultiplier = gamepad1.left_bumper ? 0.5 : 1.0;
            y  *= speedMultiplier;
            x  *= speedMultiplier;
            rx *= speedMultiplier;

            if (prevBack && !gamepad1.back) {
                traditionalDrivetrain = !traditionalDrivetrain; // toggle
            }
            prevBack = gamepad1.back;

            if(traditionalDrivetrain) {
                y *= 1.1;
                bot.move(x, -y, -rx);
            }else {

                // === Heading hold using squidToHeading when driver isn't rotating ===

                final double rotateDeadband = 0.02;
                if(!gamepad1.a){

                    final double maxDegPerSec  = 360.0;        // full-stick = 180°/s target change

                    if (Math.abs(rx) > rotateDeadband) {
                        targetHeadingDeg = targetHeadingDeg + rx * maxDegPerSec * deltaRuntime;
                    }

                    rx = bot.squidToHeading(targetHeadingDeg);


                }else{
                    rx = bot.squidToHeading(headingToPoint(32, 32));
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
            telemetry.addData("Heading Hold", "target=%.1f°, cur=%.1f°", targetHeadingDeg, headingDeg);
            telemetry.update();

            prevHeading = headingDeg;
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

        double deg = Math.toDegrees(Math.atan2(dX, dY));
        double band = Math.floor(bot.odo.getHeading(UnnormalizedAngleUnit.DEGREES) / 360.0) * 360.0;

        return deg + band;
    }

    double wrap0to360(double a) {
        double w = a % 360.0;
        if (w < 0) w += 360.0;
        return w;
    }

}
