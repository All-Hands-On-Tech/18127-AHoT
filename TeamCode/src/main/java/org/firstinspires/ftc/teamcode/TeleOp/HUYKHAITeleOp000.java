package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.ZSupport.Utilities000;

@TeleOp(name="HUYKHAI 0-0-0 TeleOp", group="Z")
public class HUYKHAITeleOp000 extends LinearOpMode {
    Utilities000 bot = new Utilities000(this);
    private ElapsedTime loopTime = new ElapsedTime();

    int shotPower = 0;
    double yaw = 0;
    double pitch = 0.6;

    double speedFactor = 0.8;

    @Override
    public void runOpMode() {
        bot.initialize(this);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        loopTime.reset();

        while (opModeIsActive()) {

            if(gamepad1.right_bumper){
                speedFactor = 0.6;
            }else{
                speedFactor = 0.8;
            }

            bot.move(-gamepad1.left_stick_y*speedFactor, gamepad1.left_stick_x*speedFactor, -gamepad1.right_stick_x*speedFactor);

            if(gamepad2.rightBumperWasPressed()) {
                shotPower += 100;
            } else if (gamepad2.leftBumperWasPressed()) {
                shotPower -= 100;
            }
            bot.setFlywheelSpeed_DO_NOT_USE(shotPower);

            if(gamepad2.dpad_right) {
                yaw += 1;
            } else if (gamepad2.dpad_left) {
                yaw -= 1;
            }
            bot.setHoodYawAngle(yaw);

            if(gamepad2.dpad_up) {
                pitch += 0.003;
            } else if (gamepad2.dpad_down) {
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


            telemetry.addData("Turret Yaw Deg: ", bot.getHoodYawAngleDegrees());
            telemetry.addData("Flywheel ticks/s: ", shotPower);
            telemetry.addData("Flywheel speed: ", bot.getFlywheelSpeed());
            telemetry.addData("Flywheel yaw:     ", yaw);
            telemetry.addData("Flywheel pitch:   ", pitch);
            telemetry.addData("loop time: ", (int)loopTime.milliseconds());
            telemetry.addData("Goal angle (deg): ", getAngleRelativeToPoint(58, 58));
            loopTime.reset();
            telemetry.update();
        }
    }
    void aimAtPoint(int x, int y) {
        Pose2D currentPose = bot.odo.getPosition();
        double currentX = currentPose.getX(DistanceUnit.INCH);
        double currentY = currentPose.getY(DistanceUnit.INCH);

        double dX = x - currentX;
        double dY = y - currentY;

        double deg = Math.toDegrees(Math.atan2(dY, dX));

        bot.setHoodYawAngleDegrees(deg);
    }

    double getAngleRelativeToPoint(int x, int y) {
        Pose2D currentPose = bot.odo.getPosition();
        double currentX = currentPose.getX(DistanceUnit.INCH);
        double currentY = currentPose.getY(DistanceUnit.INCH);

        double dX = x - currentX;
        double dY = y - currentY;

        double deg = Math.toDegrees(Math.atan2(dY, dX));

        return deg;
    }
}
