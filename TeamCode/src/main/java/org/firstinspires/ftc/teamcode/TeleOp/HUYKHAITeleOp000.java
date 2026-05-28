package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.ZSupport.Utilities000;

@TeleOp(name="HUY KHAI 0-0-0 TeleOp", group="A")
public class HUYKHAITeleOp000 extends TeleOp000 {
    private ElapsedTime loopTime = new ElapsedTime();

    @Override
    public void init() {
        super.init();

        bot.setTransferBlock();
        loopTime.reset();
    }

    @Override
    public void init_loop() {
        telemetry.addLine("Please choose alliance color");
        telemetry.addLine("x - blue");
        telemetry.addLine("b - red");
        if(gamepad1.startWasPressed()){
            setTelemetryMode(TelemetryMode.DELIVERY);
        }else if(gamepad1.backWasPressed()){
            setTelemetryMode(TelemetryMode.ODOMETRY);
        }else{
            setTelemetryMode(TelemetryMode.NONE);
        }
        setTelemetryMode(TelemetryMode.DEBUG);
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
//        super.loop();

        handleDrivetrain(gamepad1);

        handleTilt(gamepad1);

        handleDelivery(gamepad1);

        handleAimAssist(gamepad2);

        handleIntake(gamepad1);

        toggleFlywheelPower(gamepad1);//b to toggle
        toggleFlywheelPower(gamepad2);//b to toggle


        handleTelemetry();
    }
}
