package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.ZSupport.K2Utilities;
import org.firstinspires.ftc.teamcode.ZSupport.RAndDBotUtilities;

@Disabled
@TeleOp(name="K2 Drive Debug", group="A")
public class K2DriveDebug extends LinearOpMode {
    K2Utilities bot = new K2Utilities(this);

    @Override
    public void runOpMode()
    {

        bot.initialize(this);


        if (isStopRequested()) return;
        waitForStart();

        while(opModeIsActive())
        {
            double x = gamepad1.left_stick_x;
            double y = -gamepad1.left_stick_y;
            double rx = -gamepad1.right_stick_x;

            if(gamepad1.a) {
                bot.blPower = 0.5;
            }else{bot.blPower = 0.0;}
            if(gamepad1.b) {
                bot.brPower = 0.5;
            }else{bot.brPower = 0.0;}
            if(gamepad1.x) {
                bot.flPower = 0.5;
            }else{bot.flPower = 0.0;}
            if(gamepad1.y) {
                bot.frPower = 0.5;
            }else{bot.frPower = 0.0;}

            bot.applyDrivePower();



            bot.logDriveData(bot.odo.getPosition());
            telemetry.addLine();telemetry.addLine();telemetry.addLine();
            bot.logPinpointFrequency();
            bot.logREVHubFrequency();
            telemetry.addData("Joystick", "Y: %.2f X: %.2f RX: %.2f", y, x, rx);
            telemetry.update();

        }
    }

}
