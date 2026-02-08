package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.ZSupport.Utilities000;

@TeleOp(name="0-0-0 TeleOp Manual Control", group="A")
public class FeedForwardTuner extends LinearOpMode {
    Utilities000 bot = new Utilities000(this);

    int volts = 0;

    @Override
    public void runOpMode() {
        bot.initialize(this);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

           if(gamepad1.rightBumperWasPressed()) {
                volts += 1;
            } else if (gamepad1.leftBumperWasPressed()) {
                volts -= 1;
            }
            bot.setFlywheelVolts(volts);

            telemetry.addData("Flywheel volts: ", volts);
            telemetry.addData("Flywheel speed: ", bot.getFlywheelSpeed());
            telemetry.update();
        }
    }
}
