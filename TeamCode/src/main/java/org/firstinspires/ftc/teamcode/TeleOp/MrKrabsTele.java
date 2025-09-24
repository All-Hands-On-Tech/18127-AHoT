package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="MrKrabsTele", group="MrKrabs")
public class MrKrabsTele extends LinearOpMode {

    // Drive motors
    private DcMotor front_left, front_right, back_left, back_right;

    // Delivery slide
    private DcMotorEx deliverySlide;

    // Claw servo
    private Servo clawServo;
    private static final double CLAW_OPEN_POS = 0.7;
    private static final double CLAW_CLOSED_POS = 0.3;
    private static final double CLAW_STEP = 0.01; // Amount to move per loop for smoothness
    private double clawPosition;

    // Slide constants
    private static final int LIFT_MIN_POSITION = -500;        // Minimum safe position
    private static final int LIFT_MAX_POSITION = 2000;     // Maximum safe position
    private static final int LIFT_INCREMENT = 100;         // How much to move per button press
    private static final double LIFT_POWER = 0.8;          // Power for slide movement

    // Slide control variables

    // Telemetry tracking variables (to show previous values)
    private double lastY = 0, lastX = 0, lastRx = 0;
    private double lastFrontLeftPower = 0, lastFrontRightPower = 0;
    private double lastBackLeftPower = 0, lastBackRightPower = 0;
    private boolean lastSlowMode = false;
    private int lastSlideTarget = 0;

    @Override
    public void runOpMode() {
        // Initialize hardware
        initializeHardware();

        // Wait for start
        telemetry.addLine("Robot Ready - Press START");
        telemetry.addLine("LEFT STICK: Drive forward/back/strafe");
        telemetry.addLine("RIGHT STICK: Turn left/right");
        telemetry.addLine("LEFT BUMPER: Slow mode");
        telemetry.addLine("A BUTTON: Slide UP");
        telemetry.addLine("B BUTTON: Slide DOWN");
        telemetry.addLine("*** SINGLE CONTROLLER OPERATION ***");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Telemetry: Check gamepad connection and raw stick values
            telemetry.addData("Gamepad1 Connected", gamepad1.id >= 0 ? "YES" : "NO");
            telemetry.addData("Raw Left Stick Y", gamepad1.left_stick_y);
            telemetry.addData("Raw Left Stick X", gamepad1.left_stick_x);
            telemetry.addData("Raw Right Stick X", gamepad1.right_stick_x);
            if (gamepad1.id < 0) {
                telemetry.addLine("WARNING: Gamepad1 not detected! Check connection.");
            }
            telemetry.update();

            // Handle ALL controls on gamepad1
            handleDriveControls();
            handleSlideControls();
            handleClawControls();
            updateTelemetry();
        }
    }

    private void initializeHardware() {
        // Initialize drive motors
        front_left = hardwareMap.get(DcMotor.class, "LF");     // Left Front
        front_right = hardwareMap.get(DcMotor.class, "RBRE");  // Right Front
        back_left = hardwareMap.get(DcMotor.class, "RFBE");    // Back Left
        back_right = hardwareMap.get(DcMotor.class, "LBLE");   // Back Right

        front_left.setDirection(DcMotor.Direction.REVERSE);
        front_right.setDirection(DcMotor.Direction.REVERSE);
        back_left.setDirection(DcMotor.Direction.REVERSE);
        back_right.setDirection(DcMotor.Direction.FORWARD);

        // Set brake behavior for better control
        front_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        front_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        back_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        back_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Initialize delivery slide - FIXED ORDER
        deliverySlide = hardwareMap.get(DcMotorEx.class, "deliverySlide");
        deliverySlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        deliverySlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // FIXED: Set slide direction to go UP instead of into floor
        deliverySlide.setDirection(DcMotor.Direction.REVERSE);  // Reverse slide direction

        // FIXED: Set target position BEFORE switching to RUN_TO_POSITION mode
        deliverySlide.setTargetPosition(LIFT_MIN_POSITION);
        deliverySlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);  // Now set mode after target
        deliverySlide.setPower(LIFT_POWER);

        // Initialize claw servo
        clawServo = hardwareMap.get(Servo.class, "clawServo");
        clawPosition = CLAW_OPEN_POS;
        clawServo.setPosition(clawPosition); // Start open

        telemetry.addLine("Hardware Initialized Successfully");
        telemetry.addLine("RIGHT motors REVERSED as requested");
        telemetry.addLine("Slide direction FIXED to go UP");
        telemetry.addLine("Slide initialization order FIXED");
        telemetry.addLine("Single controller mode ENABLED");
        telemetry.update();
    }

    private void handleDriveControls() {
        // Get raw joystick inputs from GAMEPAD1 ONLY
        double rawY = -gamepad1.left_stick_y;   // Forward/backward (negated for correct direction)
        double rawX = -gamepad1.left_stick_x;   // Left/right strafe (NEGATED to swap directions)
        double rawRx = gamepad1.right_stick_x;  // Rotation

        // Apply deadzone
        double y = Math.abs(rawY) > 0.05 ? rawY : 0;
        double x = Math.abs(rawX) > 0.05 ? rawX : 0;
        double rx = Math.abs(rawRx) > 0.05 ? rawRx : 0;

        // Speed multiplier using left bumper
        boolean slowMode = gamepad1.left_bumper;
        double speedMultiplier = slowMode ? 0.3 : 0.7;  // Slower speeds for better control
        y *= speedMultiplier;
        x *= speedMultiplier;
        rx *= speedMultiplier;

        double frontLeftPower = rx + y + x;
        double frontRightPower = rx - y - x;
        double backLeftPower = rx - y + x;
        double backRightPower = rx + y - x;

        // Normalize powers to prevent any from exceeding 1.0
        double maxPower = Math.max(Math.abs(frontLeftPower),
                         Math.max(Math.abs(frontRightPower),
                         Math.max(Math.abs(backLeftPower), Math.abs(backRightPower))));

        if (maxPower > 1.0) {
            frontLeftPower /= maxPower;
            frontRightPower /= maxPower;
            backLeftPower /= maxPower;
            backRightPower /= maxPower;
        }

        // Set motor powers
        front_left.setPower(frontLeftPower);
        front_right.setPower(frontRightPower);
        back_left.setPower(backLeftPower);
        back_right.setPower(backRightPower);

        // Update tracking variables for telemetry
        lastY = y;
        lastX = x;
        lastRx = rx;
        lastFrontLeftPower = frontLeftPower;
        lastFrontRightPower = frontRightPower;
        lastBackLeftPower = backLeftPower;
        lastBackRightPower = backRightPower;
        lastSlowMode = slowMode;
    }

    private void handleSlideControls() {
        // Move slide UP while holding A
        if (gamepad1.a) {
            int newPosition = deliverySlide.getCurrentPosition() + LIFT_INCREMENT;
            newPosition = Math.min(newPosition, LIFT_MAX_POSITION);
            deliverySlide.setTargetPosition(newPosition);
            deliverySlide.setPower(LIFT_POWER);
            lastSlideTarget = newPosition;
        }
        // Move slide DOWN while holding B
        else if (gamepad1.b) {
            int newPosition = deliverySlide.getCurrentPosition() - LIFT_INCREMENT;
            newPosition = Math.max(newPosition, LIFT_MIN_POSITION);
            deliverySlide.setTargetPosition(newPosition);
            deliverySlide.setPower(LIFT_POWER);
            lastSlideTarget = newPosition;
        }
        // Stop power if neither button is pressed
        else {
            deliverySlide.setPower(0);
        }

        // Emergency stop
        if (gamepad1.left_trigger > 0.5 && gamepad1.right_trigger > 0.5) {
            deliverySlide.setPower(0);
            deliverySlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            sleep(100);
            deliverySlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
    }

    private void handleClawControls() {
        // Debug: manual override for servo position

        if (gamepad1.x) {
            clawPosition = 0.0;
        } else if (gamepad1.y) {
            clawPosition = 0.5;
        }
        clawServo.setPosition(clawPosition);
        telemetry.addData("Claw Position", clawPosition);
        try {
            telemetry.addData("Servo getPosition()", clawServo.getPosition());
        } catch (Exception e) {
            telemetry.addLine("Servo getPosition() not supported");
        }
    }

    private void updateTelemetry() {
        // ===== CONTROLLER STATUS =====
        telemetry.addLine("=== SINGLE CONTROLLER STATUS ===");
        telemetry.addData("Gamepad1 Connected", gamepad1.id >= 0 ? "YES" : "NO");
        telemetry.addLine("*** Using GAMEPAD1 for EVERYTHING ***");

        // ===== RAW JOYSTICK INPUTS =====
        telemetry.addLine();
        telemetry.addLine("=== RAW INPUTS (GAMEPAD1) ===");
        telemetry.addData("Left Stick Y (Raw)", "%.3f", -gamepad1.left_stick_y);
        telemetry.addData("Left Stick X (Raw)", "%.3f", gamepad1.left_stick_x);
        telemetry.addData("Right Stick X (Raw)", "%.3f", gamepad1.right_stick_x);
        telemetry.addData("Left Bumper", gamepad1.left_bumper ? "PRESSED" : "Released");
        telemetry.addData("A Button", gamepad1.a ? "PRESSED" : "Released");
        telemetry.addData("B Button", gamepad1.b ? "PRESSED" : "Released");

        // ===== PROCESSED DRIVE VALUES =====
        telemetry.addLine();
        telemetry.addLine("=== PROCESSED DRIVE VALUES ===");
        telemetry.addData("Speed Mode", lastSlowMode ? "SLOW (30%)" : "NORMAL (70%)");
        telemetry.addData("Y (Forward/Back)", "%.3f", lastY);
        telemetry.addData("X (Strafe)", "%.3f", lastX);
        telemetry.addData("RX (Rotation)", "%.3f", lastRx);

        // ===== MOTOR POWERS =====
        telemetry.addLine();
        telemetry.addLine("=== MOTOR POWERS ===");
        telemetry.addData("Front Left", "%.3f", lastFrontLeftPower);
        telemetry.addData("Front Right", "%.3f", lastFrontRightPower);
        telemetry.addData("Back Left", "%.3f", lastBackLeftPower);
        telemetry.addData("Back Right", "%.3f", lastBackRightPower);

        // ===== SLIDE STATUS =====
        telemetry.addLine();
        telemetry.addLine("=== SLIDE STATUS ===");
        telemetry.addData("Current Position", deliverySlide.getCurrentPosition());
        telemetry.addData("Target Position", deliverySlide.getTargetPosition());
        telemetry.addData("Last Target Set", lastSlideTarget);
        telemetry.addData("At Target", Math.abs(deliverySlide.getCurrentPosition() - deliverySlide.getTargetPosition()) < 10 ? "YES" : "NO");
        telemetry.addData("Power", "%.2f", deliverySlide.getPower());
        telemetry.addData("Direction", "REVERSED (UP is positive)");

        // ===== MOTOR MAPPING INFO =====
        telemetry.addLine();
        telemetry.addLine("=== MOTOR DIRECTIONS (FIXED) ===");
        telemetry.addData("LF -> Front Left", "Direction: FORWARD");
        telemetry.addData("RBRE -> Front Right", "Direction: REVERSE");
        telemetry.addData("RFBE -> Back Left", "Direction: FORWARD");
        telemetry.addData("LBLE -> Back Right", "Direction: REVERSE");
        telemetry.addLine("*** RIGHT MOTORS REVERSED ***");

        // ===== CONTROLS REMINDER =====
        telemetry.addLine();
        telemetry.addLine("=== SINGLE CONTROLLER CONTROLS ===");
        telemetry.addLine("Left stick: Move forward/back/strafe");
        telemetry.addLine("Right stick: Turn left/right");
        telemetry.addLine("Left bumper: Slow mode");
        telemetry.addLine("A button: Slide UP");
        telemetry.addLine("B button: Slide DOWN");
        telemetry.addLine("Both triggers: Emergency slide stop");

        telemetry.update();
    }
}
