package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.ZSupport.K2Utilities;
import org.firstinspires.ftc.teamcode.ZSupport.Utilities000;

@TeleOp(name="No tune 0-0-0 TeleOp", group="A")
@Disabled
public class TeleOp000 extends LinearOpMode {
    Utilities000 bot = new Utilities000(this);
    private ElapsedTime loopTime = new ElapsedTime();

    int shotPower = 0;
    double yaw = 0;
    double pitch = 0;

    @Override
    public void runOpMode() {
        bot.initialize(this);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        loopTime.reset();

        while (opModeIsActive()) {

            double drive = -gamepad1.left_stick_y;
            double turn  =  gamepad1.right_stick_x;
            bot.move(-gamepad1.left_stick_y, -gamepad1.left_stick_x, gamepad1.right_stick_x, 0.6);

            if(gamepad1.right_bumper) {
                shotPower += 100;
            } else if (gamepad1.left_bumper) {
                shotPower -= 100;
            }
            bot.setFlywheelSpeed_DO_NOT_USE(shotPower);

            if(gamepad1.dpad_right) {
                yaw += 100;
            } else if (gamepad1.dpad_left) {
                yaw -= 100;
            }
            bot.setHoodYawAngle(yaw);

            if(gamepad1.dpad_up) {
                pitch += 0.2;
            } else if (gamepad1.dpad_down) {
                pitch -= 0.2;
            }
            bot.setHoodPitchAngle(pitch);



            telemetry.addData("Flywheel ticks/s: ", shotPower);
            telemetry.addData("Flywheel yaw:     ", yaw);
            telemetry.addData("Flywheel pitch:   ", pitch);
            telemetry.addData("loop time: ", (int)loopTime.milliseconds());
            loopTime.reset();
            telemetry.update();
        }
    }
}
