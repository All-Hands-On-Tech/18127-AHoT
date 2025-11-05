package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Servo Position Debug", group="ZTesting")
public class ServoPositionDebug extends LinearOpMode {

    Servo hood, leftTransfer, rightTransfer, door;
    Servo[] servos;
    String[] names = {"hood", "leftTransfer", "rightTransfer", "door"};
    int index = 0;

    double position = 0.5;
    final double step = 0.01;
    final double inputDelay = 0.2; // seconds debounce time

    ServoController controller;

    // Timers for input debounce
    ElapsedTime dpadTimer = new ElapsedTime();
    ElapsedTime bumperTimer = new ElapsedTime();

    @Override
    public void runOpMode() {

        hood = hardwareMap.get(Servo.class, "hood");
        leftTransfer = hardwareMap.get(Servo.class, "leftTransfer");
        rightTransfer = hardwareMap.get(Servo.class, "rightTransfer");
//        door = hardwareMap.get(Servo.class, "door");

        servos = new Servo[]{hood, leftTransfer, rightTransfer, door};

        controller = hood.getController(); // Get hub

        telemetry.addLine("Initialized — Press Start");
        telemetry.update();

        waitForStart();

        disableAll();

        while (opModeIsActive()) {

            // === Servo Selection (D-pad) ===
            if (dpadTimer.seconds() > inputDelay) {
                if (gamepad1.dpad_up) {
                    index = (index + 1) % servos.length;
                    disableAll();
                    dpadTimer.reset();
                } else if (gamepad1.dpad_down) {
                    index = (index - 1 + servos.length) % servos.length;
                    disableAll();
                    dpadTimer.reset();
                }
            }

            // === Position Adjust (Bumpers) ===
            if (bumperTimer.seconds() > inputDelay) {
                if (gamepad1.right_bumper) {
                    position = Math.min(1.0, position + step);
                    bumperTimer.reset();
                } else if (gamepad1.left_bumper) {
                    position = Math.max(0.0, position - step);
                    bumperTimer.reset();
                }
            }

            // === Drive only selected servo ===
            int port = servos[index].getPortNumber();
            controller.pwmEnable();
            servos[index].setPosition(position);

            // === Telemetry ===
            telemetry.addLine("=== Servo Debug ===");
            telemetry.addData("Active Servo", names[index]);
            telemetry.addData("Servo Port", port);
            telemetry.addData("Position", "%.3f", position);
            telemetry.addLine("Controls:");
            telemetry.addLine("  D-Pad Up/Down = select servo");
            telemetry.addLine("  RB = +0.01 | LB = -0.01");
            telemetry.update();
        }
    }

    private void disableAll() {
        controller.pwmDisable();
    }
}
