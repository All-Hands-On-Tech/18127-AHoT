package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.ZSupport.Utilities000;

@TeleOp(name="0-0-0 TeleOp Manual Control", group="A")
public class TeleOp000TESTING extends LinearOpMode {
    Utilities000 bot = new Utilities000(this);
    private ElapsedTime loopTime = new ElapsedTime();

    int shotPower = 0;
    double yaw = 0;
    double pitch = 0.6;

    @Override
    public void runOpMode() {
        bot.initialize(this);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        loopTime.reset();

        while (opModeIsActive()) {

            bot.move(-gamepad1.left_stick_y/3, gamepad1.left_stick_x/3, -gamepad1.right_stick_x/3);

            if(gamepad1.rightBumperWasPressed()) {
                shotPower += 100;
            } else if (gamepad1.leftBumperWasPressed()) {
                shotPower -= 100;
            }
            bot.setFlywheelSpeed_DO_NOT_USE(shotPower);

            if(gamepad1.dpad_right) {
                yaw += 1;
            } else if (gamepad1.dpad_left) {
                yaw -= 1;
            }
            bot.setHoodYawAngle(yaw);

            if(gamepad1.dpad_up) {
                pitch += 0.003;
            } else if (gamepad1.dpad_down) {
                pitch -= 0.003;
            }
            bot.setHoodPitchAngle(pitch);
            //0.61 -> 0.84

            if(gamepad1.right_trigger > 0.01){
                bot.intakePower(gamepad1.right_trigger);
            }

            if(gamepad1.left_trigger > 0.01){
                bot.intakePower(gamepad1.left_trigger);
            }


            telemetry.addData("Turret Yaw Deg: ", "%.1", bot.getHoodYawAngleDegrees());
            telemetry.addData("Flywheel ticks/s: ", shotPower);
            telemetry.addData("Flywheel speed: ", bot.getFlywheelSpeed());
            telemetry.addData("Flywheel yaw:     ", yaw);
            telemetry.addData("Flywheel pitch:   ", pitch);
            telemetry.addData("loop time: ", (int)loopTime.milliseconds());
            loopTime.reset();
            telemetry.update();
        }
    }
}
