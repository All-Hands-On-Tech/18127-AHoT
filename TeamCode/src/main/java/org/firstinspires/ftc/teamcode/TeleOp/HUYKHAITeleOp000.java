package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.ZSupport.Utilities000;

@TeleOp(name="HUYKHAI 0-0-0 TeleOp", group="Z")
public class HUYKHAITeleOp000 extends TeleOp000 {
    private ElapsedTime loopTime = new ElapsedTime();

    @Override
    public void init() {
        bot.initialize(this);
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        setTelemetryMode(TelemetryMode.DEBUG);

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

        handleDelivery(gamepad2);

        handleIntake(gamepad1);

        handleTelemetry();
    }
}
