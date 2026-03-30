package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.ZSupport.Utilities000;

@TeleOp(name="Shoot At Goal (No Drivetrain)", group="DEMO")
public class ShootAtGoal extends TeleOp000 {
    private ElapsedTime loopTime = new ElapsedTime();

    @Override
    public void init() {
        super.init();

        bot.setTransferBlock();
        bot.updateOdo();
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

        handleDelivery(gamepad1, true);

        handleAimAssist(gamepad2);

        handleIntake(gamepad1);

        handleTelemetry();
    }
}
