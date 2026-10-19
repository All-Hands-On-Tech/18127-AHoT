package org.firstinspires.ftc.teamcode.main;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.Alliance;
import org.firstinspires.ftc.teamcode.drive.Drivetrain;

@TeleOp(name = "Main: BBB Driver", group = "Main")
public class MainTeleOp extends OpMode {
    private Drivetrain drivetrain;
    private Alliance selectedAlliance = Alliance.BLUE;

    @Override
    public void init() {
        drivetrain = new Drivetrain(hardwareMap);
        drivetrain.setAlliance(selectedAlliance);
    }

    @Override
    public void init_loop() {
        if (gamepad1.dpad_left || gamepad1.x) {
            selectedAlliance = Alliance.RED;
        } else if (gamepad1.dpad_right || gamepad1.b) {
            selectedAlliance = Alliance.BLUE;
        }

        drivetrain.setAlliance(selectedAlliance);

        telemetry.addData("Alliance", selectedAlliance.getDisplayName());
        telemetry.addData("Select", "D-pad left/X = red, D-pad right/B = blue");
        telemetry.update();
    }

    @Override
    public void loop() {
        drivetrain.update(gamepad1, gamepad1.left_bumper);

        telemetry.addData("Alliance", selectedAlliance.getDisplayName());
        telemetry.addData("Aim", gamepad1.left_bumper ? "holding" : "off");
        telemetry.addData("Pose", drivetrain.getAimAssist().getPose());
        telemetry.addData("Aim error (deg)",
                Math.toDegrees(drivetrain.getAimAssist().getHeadingError()));
        telemetry.update();
    }

    @Override
    public void stop() {
        if (drivetrain != null) {
            drivetrain.stop();
        }
    }
}
