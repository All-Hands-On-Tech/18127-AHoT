package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.ZSupport.Utilities000;

@TeleOp(name="Single Player 0-0-0 TeleOp", group="A")
public class SinglePlayerTeleOp000 extends TeleOp000 {
    private ElapsedTime loopTime = new ElapsedTime();

    @Override
    public void init() {
        bot.initialize(this);
        telemetry.addData("Status", "Initialized");
        telemetry.update();

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

            handleDrivetrain(gamepad1);

            handleDelivery(gamepad1);

            handleIntake(gamepad1);

            handleTelemetry();
        }
}
