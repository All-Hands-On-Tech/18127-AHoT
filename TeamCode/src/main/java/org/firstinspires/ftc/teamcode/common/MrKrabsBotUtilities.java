package org.firstinspires.ftc.teamcode.common;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class MrKrabsBotUtilities {
    private DcMotor front_left, front_right, back_left, back_right;
    private DcMotorEx deliverySlide;
    private LinearOpMode opMode;
    private static final int LIFT_GROUND_POSITION = 100;
    private static final int LIFT_STARTUP_ACTIVATION = 150; // Small activation movement on start

    public void initialize(LinearOpMode opMode) {
        this.opMode = opMode;
        front_left   = opMode.hardwareMap.get(DcMotor.class, "LF");
        front_right  = opMode.hardwareMap.get(DcMotor.class, "RBRE");
        back_left    = opMode.hardwareMap.get(DcMotor.class, "RFBE");
        back_right   = opMode.hardwareMap.get(DcMotor.class, "LBLE");
        front_right.setDirection(DcMotor.Direction.REVERSE);
        back_right.setDirection(DcMotor.Direction.REVERSE);

        deliverySlide = opMode.hardwareMap.get(DcMotorEx.class, "deliverySlide");
        deliverySlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // First move to safe ground position
        deliverySlide.setTargetPosition(LIFT_GROUND_POSITION);
        deliverySlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        deliverySlide.setPower(0.8);

        // Wait briefly and then do a small activation movement
        try {
            Thread.sleep(500); // Wait for initial positioning
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }

        // Small activation movement to show the slide is working
        deliverySlide.setTargetPosition(LIFT_STARTUP_ACTIVATION);
        deliverySlide.setPower(0.6);

        opMode.telemetry.addLine("Robot initialized - Delivery slide activated and ready");
        opMode.telemetry.update();
    }

    public void move(double x, double y, double rx) {
        double[] speeds = {
            (y + x + rx),  // front_left
            (y - x - rx),  // front_right
            (y - x + rx),  // back_left
            (y + x - rx)   // back_right
        };
        double max = Math.abs(speeds[0]);
        for (int i = 0; i < speeds.length; i++) {
            if (max < Math.abs(speeds[i])) max = Math.abs(speeds[i]);
        }
        if (max > 1) {
            for (int i = 0; i < speeds.length; i++) speeds[i] /= max;
        }
        front_left.setPower(speeds[0]);
        front_right.setPower(speeds[1]);
        back_left.setPower(speeds[2]);
        back_right.setPower(speeds[3]);
    }

    public void handleLift(Gamepad gamepad2) {
        // Always enforce minimum position before handling controls
        enforceLiftSafety();
        if (gamepad2.dpad_up) {
            setLiftPosition(deliverySlide.getCurrentPosition() + 50);
        } else if (gamepad2.dpad_down) {
            setLiftPosition(deliverySlide.getCurrentPosition() - 50);
        }
        if (gamepad2.a) {
            setLiftPosition(LIFT_GROUND_POSITION);
        } else if (gamepad2.y) {
            setLiftPosition(LIFT_GROUND_POSITION + 500);
        }
    }

    private void enforceLiftSafety() {
        // If the lift is below the safe position, move it up
        if (deliverySlide.getCurrentPosition() < LIFT_GROUND_POSITION) {
            setLiftPosition(LIFT_GROUND_POSITION);
        }
    }

    private void setLiftPosition(int position) {
        int targetPosition = Math.max(position, LIFT_GROUND_POSITION);
        deliverySlide.setTargetPosition(targetPosition);
        if (deliverySlide.getMode() != DcMotor.RunMode.RUN_TO_POSITION) {
            deliverySlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        deliverySlide.setPower(0.8);
    }

    public void logDriveData() {
        // Safety check every loop
        enforceLiftSafety();
        opMode.telemetry.addLine("=== LIFT STATUS ===");
        opMode.telemetry.addData("Position", deliverySlide.getCurrentPosition());
        opMode.telemetry.addData("Target", deliverySlide.getTargetPosition());
        opMode.telemetry.addData("At Target", Math.abs(deliverySlide.getCurrentPosition() - deliverySlide.getTargetPosition()) < 10);
        if (deliverySlide.getCurrentPosition() < LIFT_GROUND_POSITION) {
            opMode.telemetry.addLine("WARNING: Lift below safe position! Moving up...");
        }
    }
}
