package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.ZSupport.Utilities000;

@TeleOp(name="HUYKHAI 0-0-0 TeleOp", group="Z")
public class HUYKHAITeleOp000 extends TeleOp000 {
    private ElapsedTime loopTime = new ElapsedTime();

    @Override
    public void runOpMode() {
        bot.initialize(this);
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        setTelemetryMode(TelemetryMode.DEBUG);

        waitForStart();
        loopTime.reset();

        while (opModeIsActive()) {

            handleDrivetrain(gamepad1);

            handleDelivery(gamepad2);

            handleIntake(gamepad1);

            handleTelemetry();
        }
    }
    void aimAtPoint(int x, int y) {
        Pose2D currentPose = bot.odo.getPosition();
        double currentX = currentPose.getX(DistanceUnit.INCH);
        double currentY = currentPose.getY(DistanceUnit.INCH);

        double dX = x - currentX;
        double dY = y - currentY;

        double deg = Math.toDegrees(Math.atan2(dY, dX));

        bot.setHoodYawAngleDegrees(deg);
    }

    double getAngleRelativeToPoint(int x, int y) {
        Pose2D currentPose = bot.odo.getPosition();
        double currentX = currentPose.getX(DistanceUnit.INCH);
        double currentY = currentPose.getY(DistanceUnit.INCH);

        double dX = x - currentX;
        double dY = y - currentY;

        double deg = Math.toDegrees(Math.atan2(dY, dX));

        return deg;
    }
}
