package org.firstinspires.ftc.teamcode.Unused;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "SmallKrabsTele", group = "TeleOp")
public class SmallKrabsTele extends OpMode {

    // Local hardware names (change if your config differs)
    private static final String LF_NAME = "frontLeft";
    private static final String RF_NAME = "frontRight";
    private static final String LR_NAME = "backLeft";
    private static final String RR_NAME = "backRight";
    private static final String CLAW_SERVO_NAME = "clawServo";
    private static final String SLIDER_MOTOR_NAME = "sliderMotor";

    // Local motor power correction multipliers (start at 1.0)
    private static final double LF_MULT = 1.0;
    private static final double RF_MULT = 1.0;
    private static final double LR_MULT = 0.8;
    private static final double RR_MULT = 0.8;

    // Slow mode scale (50% when left bumper held)
    private static final double SLOW_MODE_SCALE = 0.5;

    // Claw and slider controls
    private static final double CLAW_DELTA = 0.02; // servo step per loop when bumper held
    private static final double CLAW_MIN = 0.0;
    private static final double CLAW_MAX = 1.0;
    private double clawPosition = 0.5; // start half-open

    private static final double SLIDER_POWER = 0.6; // base power for slider motor
    private static final double TRIGGER_THRESHOLD = 0.05; // minimal trigger value to move

    // Motors
    private DcMotor leftFront;
    private DcMotor rightFront;
    private DcMotor leftRear;
    private DcMotor rightRear;
    private DcMotor sliderMotor;
    private Servo clawServo;

    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void init() {
        telemetry.addData("Status", "Initializing SmallKrabsTele");

        HardwareMap hw = hardwareMap;
        try {
            leftFront = hw.get(DcMotor.class, LF_NAME);
            rightFront = hw.get(DcMotor.class, RF_NAME);
            leftRear = hw.get(DcMotor.class, LR_NAME);
            rightRear = hw.get(DcMotor.class, RR_NAME);
            // optional hardware that may not exist on all bots
            try {
                clawServo = hw.get(Servo.class, CLAW_SERVO_NAME);
            } catch (Exception ignored) { clawServo = null; }
            try {
                sliderMotor = hw.get(DcMotor.class, SLIDER_MOTOR_NAME);
            } catch (Exception ignored) { sliderMotor = null; }
        } catch (Exception e) {
            telemetry.addData("Hardware", "Failed to map one or more motors: " + e.getMessage());
        }

        // Set motor directions to common mecanum layout
        if (leftFront != null) leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        if (leftRear != null) leftRear.setDirection(DcMotorSimple.Direction.REVERSE);
        if (rightFront != null) rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        if (rightRear != null) rightRear.setDirection(DcMotorSimple.Direction.FORWARD);
        if (sliderMotor != null) sliderMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        // Set run modes
        if (leftFront != null) leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        if (leftRear != null) leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        if (rightFront != null) rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        if (rightRear != null) rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        if (sliderMotor != null) sliderMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        if (clawServo != null) clawServo.setPosition(clawPosition);

        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void start() {
        runtime.reset();
    }

    @Override
    public void loop() {
        // Read the joysticks
        double forward = -0.5*gamepad1.left_stick_y; // forward is negative on gamepad
        double strafe = 0.5*gamepad1.left_stick_x;
        double rotate = 0.35*gamepad1.right_stick_x;

        // Mecanum drive algorithm (standard)
        double lf = forward + strafe + rotate;
        double rf = forward - strafe - rotate;
        double lr = forward - strafe + rotate;
        double rr = forward + strafe - rotate;

        // Normalize so no value exceeds 1.0
        double max = Math.max(1.0, Math.max(Math.abs(lf), Math.max(Math.abs(rf), Math.max(Math.abs(lr), Math.abs(rr)))));
        lf /= max;
        rf /= max;
        lr /= max;
        rr /= max;

        // Apply slow mode scale and motor multipliers
        lf = lf * LF_MULT;
        rf = rf * RF_MULT;
        lr = lr * LR_MULT;
        rr = rr * RR_MULT;

        // Set motor powers
        if (leftFront != null) leftFront.setPower(lf);
        if (rightFront != null) rightFront.setPower(rf);
        if (leftRear != null) leftRear.setPower(lr);
        if (rightRear != null) rightRear.setPower(rr);

        // ---------- Claw servo control (LB / RB) ----------
        // Left bumper closes/lowers claw (decrease position), right bumper opens/raises (increase)
        if (gamepad2.left_bumper) {
            clawPosition -= CLAW_DELTA;
        }
        if (gamepad2.right_bumper) {
            clawPosition += CLAW_DELTA;
        }
        // Clamp and write
        if (clawPosition < CLAW_MIN) clawPosition = CLAW_MIN;
        if (clawPosition > CLAW_MAX) clawPosition = CLAW_MAX;
        if (clawServo != null) clawServo.setPosition(clawPosition);

        // ---------- Slider motor control (ZL / ZR mapped to left_trigger / right_trigger) ----------
        double sliderPower = 0.0;
        if (gamepad2.left_trigger > TRIGGER_THRESHOLD) {
            // left trigger moves slider in positive direction
            sliderPower = SLIDER_POWER * gamepad2.left_trigger;
        } else if (gamepad2.right_trigger > TRIGGER_THRESHOLD) {
            // right trigger moves slider in negative direction
            sliderPower = -SLIDER_POWER * gamepad2.right_trigger;
        }
        if (sliderMotor != null) sliderMotor.setPower(sliderPower);

        // Telemetry
        telemetry.addData("lf", String.format("%.2f", lf));
        telemetry.addData("rf", String.format("%.2f", rf));
        telemetry.addData("lr", String.format("%.2f", lr));
        telemetry.addData("rr", String.format("%.2f", rr));
        telemetry.addData("clawPos", String.format("%.2f", clawPosition));
        telemetry.addData("sliderPwr", String.format("%.2f", sliderMotor != null ? sliderMotor.getPower() : 0.0));
        telemetry.update();
    }

    @Override
    public void stop() {
        if (leftFront != null) leftFront.setPower(0);
        if (rightFront != null) rightFront.setPower(0);
        if (leftRear != null) leftRear.setPower(0);
        if (rightRear != null) rightRear.setPower(0);
        if (sliderMotor != null) sliderMotor.setPower(0);
        if (clawServo != null) clawServo.setPosition(clawPosition);
    }
}
