package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.ZSupport.RAndDBotUtilities;

@TeleOp(name="RAndDBotTeleOp", group="Z")
public class RAndDBotTeleOp extends LinearOpMode {
    RAndDBotUtilities bot;

    @Override
    public void runOpMode()
    {
        bot.initialize(this);

        if (isStopRequested()) return;
        waitForStart();
        while(opModeIsActive())
        {
            bot.updateLocalization();

            double y = Math.abs(gamepad1.left_stick_y) > 0.05 ? gamepad1.left_stick_y : 0;    // Forward/backward
            double x = Math.abs(gamepad1.left_stick_x) > 0.05 ? -gamepad1.left_stick_x : 0;   // Left/right strafe
            double rx = Math.abs(gamepad1.right_stick_x) > 0.05 ? -gamepad1.right_stick_x : 0; // Rotation

            double speedMultiplier = gamepad1.left_bumper ? 0.5 : 1.0;
            y *= speedMultiplier;
            x *= speedMultiplier;
            rx *= speedMultiplier;

            x *= 1.1;

            bot.move(x, y, rx);

            bot.logDriveData(bot.odo.getPosition());
            telemetry.addLine();telemetry.addLine();telemetry.addLine();
            bot.logPinpointFrequency();
            bot.logREVHubFrequency();
            telemetry.addData("Joystick", "Y: %.2f X: %.2f RX: %.2f", y, x, rx);
            telemetry.update();
        }
    }
}
