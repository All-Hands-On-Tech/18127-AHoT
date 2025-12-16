package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.ZSupport.RAndDBotUtilities;

@Disabled
@TeleOp(name="RAndDBotDebug", group="Z")
public class RAndDBotDebug extends LinearOpMode {
    RAndDBotUtilities bot = new RAndDBotUtilities(this);

    @Override
    public void runOpMode()
    {
        bot.initialize(this);

        if (isStopRequested()) return;
        waitForStart();
        while(opModeIsActive())
        {
            if(gamepad1.a)
            {
                bot.frPower = 1;
            }else{
                bot.frPower = 0;
            }
            if(gamepad1.b)
            {
                bot.brPower = 1;
            }else{
                bot.brPower = 0;
            }
            if(gamepad1.x)
            {
                bot.blPower = 1;
            }else{
                bot.blPower = 0;
            }
            if(gamepad1.y){
                bot.flPower = 1;
            }else{
                bot.flPower = 0;
            }
            bot.applyDrivePower();

            telemetry.addLine("=== Motor Debug Menu ===");
            telemetry.addData("A → Front Right",  gamepad1.a ? "ON" : "off");
            telemetry.addData("B → Back Right",   gamepad1.b ? "ON" : "off");
            telemetry.addData("X → Back Left",    gamepad1.x ? "ON" : "off");
            telemetry.addData("Y → Front Left",   gamepad1.y ? "ON" : "off");

            telemetry.addLine("--- Current Powers ---");
            telemetry.addData("Front Right", bot.frPower);
            telemetry.addData("Back Right",  bot.brPower);
            telemetry.addData("Back Left",   bot.blPower);
            telemetry.addData("Front Left",  bot.flPower);

            telemetry.update();

        }
    }
}
