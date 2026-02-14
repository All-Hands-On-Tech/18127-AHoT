package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

@TeleOp(name="Single Player 0-0-0 TeleOp", group="A")
public class SinglePlayerTeleOp000 extends TeleOp000 {
    private ElapsedTime loopTime = new ElapsedTime();

    @Override
    public void runOpMode() {
        bot.initialize(this);
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        setTelemetryMode(TelemetryMode.ODOMETRY);

        waitForStart();
        loopTime.reset();

        while (opModeIsActive()) {

            handleDrivetrain(gamepad1);

            handleDelivery(gamepad1);

            handleIntake(gamepad1);

            handleTelemetry();
        }
    }
}
