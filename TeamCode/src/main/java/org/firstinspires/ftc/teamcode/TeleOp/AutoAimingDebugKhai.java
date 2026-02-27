package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.ZSupport.Utilities000;

@TeleOp(name="Auto Aiming Debug Khai", group="Z")
public class AutoAimingDebugKhai extends TeleOp000 {
    private ElapsedTime loopTime = new ElapsedTime();

    double targetDeg = 0.0;

    @Override
    public void init() {
        super.init();

        setTelemetryMode(TelemetryMode.DELIVERY);

        loopTime.reset();
    }

    @Override
    public void init_loop() {
        telemetry.addLine("Please choose alliance color");
        telemetry.addLine("x - blue");
        telemetry.addLine("b - red");
        if (gamepad1.bWasPressed()) {
            bot.setAllianceColor(Utilities000.AllianceColor.RED);
        } else if (gamepad1.xWasPressed()) {
            bot.setAllianceColor(Utilities000.AllianceColor.BLUE);
        }
        telemetry.addData("Your alliance color is: ", bot.getAllianceColor().toString());

    }

    @Override
    public void start() {
        loopTime.reset();
    }

    @Override
    public void loop() {
        if(gamepad1.rightBumperWasPressed()){
            targetDeg += 5;
        }
        if(gamepad1.leftBumperWasPressed()){
            targetDeg -= 5;
        }
        bot.setHoodYawPower(1);
        bot.setHoodYawAngleDegrees(targetDeg);



        handleTelemetry();
    }
}
