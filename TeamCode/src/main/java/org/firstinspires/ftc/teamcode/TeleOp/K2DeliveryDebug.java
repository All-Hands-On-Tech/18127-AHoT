package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import org.firstinspires.ftc.teamcode.ZSupport.K2Utilities;

@Disabled
@TeleOp(name="K2 Delivery Debug", group="Z")
public class K2DeliveryDebug extends LinearOpMode {
    K2Utilities bot = new K2Utilities(this);

    ElapsedTime timer = new ElapsedTime();
    double startTime = 0.0;
    double endTime = 0.0;

    boolean toSpeed = false;

    double targetRPM = 0;
    double prevTargetRPM = 0;

    boolean dpadUp, prevDpadUp;
    boolean dpadDown, prevDpadDown;

    @Override
    public void runOpMode()
    {
        bot.initialize(this);


        if (isStopRequested()) return;
        waitForStart();

        startTime = timer.seconds();

        while(opModeIsActive())
        {
            dpadUp = gamepad1.dpad_up;
            dpadDown = gamepad1.dpad_down;

            if(prevDpadUp && !dpadUp){
                targetRPM += 100;
            }
            if(prevDpadDown && !dpadDown){
                targetRPM -= 100;
            }

            if(prevTargetRPM != targetRPM){
                toSpeed = false;
                startTime = timer.seconds();
            }

            if(gamepad1.a) {
                bot.setDeliverVel(targetRPM);
            }else{
                bot.setDeliverVel(0);
            }


            if(Math.abs(bot.getDeliverLVel() - targetRPM) < 5 && !toSpeed){
                endTime = timer.seconds();
                toSpeed = true;
            }

            if(toSpeed) telemetry.addData("Ramp-up time:", endTime-startTime);

            prevTargetRPM = targetRPM;
            prevDpadDown = dpadDown;
            prevDpadUp = dpadUp;

            telemetry.addData("Hold 'a' to apply rpm of: ", targetRPM);
            telemetry.addLine();telemetry.addLine();

            bot.logDriveData(bot.odo.getPosition());
            telemetry.addLine();telemetry.addLine();telemetry.addLine();
            bot.logPinpointFrequency();
            bot.logREVHubFrequency();
            telemetry.addData("DeliverL rpm", bot.getDeliverLVel());
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


    private double pointToHeading(double x, double y)
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
