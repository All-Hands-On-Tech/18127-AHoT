package org.firstinspires.ftc.teamcode.Testing;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.teamcode.common.PIDController;

/**
 * Deposit PID Tuner - Allows real-time tuning of PID parameters for the deposit motors
 *
 * CONTROLLER 1 (Gamepad1) - Motor Control:
 *   D-Pad Up/Down: Increase/Decrease Target Speed (+/- 10 ticks)
 *   D-Pad Left: RESET (set speed to 0, stop motors, reset PID)
 *   D-Pad Right: START MOTORS
 *   A Button: STOP MOTORS (without reset)
 *
 * CONTROLLER 2 (Gamepad2) - PID Tuning:
 *   D-Pad Up/Down: Adjust KP (+/- increment)
 *   D-Pad Left/Right: Adjust KI (+/- increment)
 *   Left Trigger (LT): Adjust KD (increase)
 *   Right Trigger (RT): Adjust KD (decrease)
 *   X Button: Adjust KFF (+/- based on D-Pad Up/Down)
 *   B Button: Toggle increment size (COARSE 0.0001 / FINE 0.00001)
 *   Y Button: Print current PID values
 *
 * QUICK START:
 *   1. Connect 2 controllers
 *   2. Controller 1 D-Pad Up to set target speed
 *   3. Controller 1 D-Pad Right to START
 *   4. Controller 2 to adjust PID values in real-time
 *   5. Controller 1 A to STOP
 */

@TeleOp(name = "Deposit PID Tuner", group = "Testing")
@Configurable
public class DepositPIDTuner extends OpMode {

    private TelemetryManager panelsTelemetry;
    private DcMotorEx depositMotorL;
    private DcMotorEx depositMotorR;
    private PIDController depositPID;

    // PID Tuning Parameters (180 ticks/sec = 400 RPM, conservative anti-overshoot values)
    private double targetSpeed = 0.0; // encoder ticks per second
    private double kp = 0.00008;      // Very low - gentle, no overshoot
    private double ki = 0.000003;     // Tiny - fine adjustments only
    private double kd = 0.0002;       // High - smooth damping
    private double kff = 0.00018;     // Low - gradual power increase

    // Motor control
    private boolean motorsRunning = false;
    private long lastUpdateTime = 0;

    // UI/UX variables
    private double incrementSize = 0.00001; // Fine tuning by default
    private double speedIncrementSize = 10.0; // Speed increment
    private boolean coarseMode = false;
    private boolean previousYPressed = false;
    private boolean previousBPressed = false;
    private boolean previousAPressed = false;
    private boolean previousDpadUpPressed = false;
    private boolean previousDpadDownPressed = false;
    private boolean previousDpadLeftPressed = false;
    private boolean previousDpadRightPressed = false;
    private boolean previousLeftStickPressed = false;
    private boolean previousRightStickPressed = false;
    private boolean previousLeftTriggerPressed = false;
    private boolean previousRightTriggerPressed = false;
    private boolean previousXPressed = false;

    private double maxSpeed = 2000.0; // Max allowable target speed

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        try {
            depositMotorL = hardwareMap.get(DcMotorEx.class, "DepositMotorL");
            if (depositMotorL != null) {
                depositMotorL.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
                depositMotorL.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
                depositMotorL.setPower(0);
            }
        } catch (Exception e) {
            depositMotorL = null;
            telemetry.addLine("ERROR: Could not initialize DepositMotorL");
        }

        try {
            depositMotorR = hardwareMap.get(DcMotorEx.class, "DepositMotorR");
            if (depositMotorR != null) {
                depositMotorR.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
                depositMotorR.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
                depositMotorR.setPower(0);
            }
        } catch (Exception e) {
            depositMotorR = null;
            telemetry.addLine("ERROR: Could not initialize DepositMotorR");
        }

        depositPID = new PIDController(kp, ki, kd);
        depositPID.setOutputLimits(-1.0, 1.0);
        depositPID.setIntegratorLimits(-2000, 2000);

        lastUpdateTime = System.currentTimeMillis();

        printWelcomeMessage();
    }

    @Override
    public void loop() {
        handleInput();
        updatePIDController();
        runMotors();
        updateTelemetry();
    }

    @Override
    public void stop() {
        if (depositMotorL != null) depositMotorL.setPower(0);
        if (depositMotorR != null) depositMotorR.setPower(0);
        motorsRunning = false;
    }

    private void handleInput() {
        // ========== GAMEPAD 1: MOTOR CONTROL ==========

        // Target Speed Control (D-Pad Up/Down)
        if (gamepad1.dpad_up && !previousDpadUpPressed) {
            targetSpeed = Math.min(targetSpeed + speedIncrementSize, maxSpeed);
            previousDpadUpPressed = true;
        }
        previousDpadUpPressed = gamepad1.dpad_up;

        if (gamepad1.dpad_down && !previousDpadDownPressed) {
            targetSpeed = Math.max(targetSpeed - speedIncrementSize, 0.0);
            previousDpadDownPressed = true;
        }
        previousDpadDownPressed = gamepad1.dpad_down;

        // Reset Everything (D-Pad Left)
        if (gamepad1.dpad_left && !previousDpadLeftPressed) {
            targetSpeed = 0.0;
            motorsRunning = false;
            if (depositMotorL != null) depositMotorL.setPower(0);
            if (depositMotorR != null) depositMotorR.setPower(0);
            depositPID.reset();
            previousDpadLeftPressed = true;
        }
        previousDpadLeftPressed = gamepad1.dpad_left;

        // Start Motors (D-Pad Right)
        if (gamepad1.dpad_right && !previousDpadRightPressed) {
            motorsRunning = true;
            depositPID.reset();
            previousDpadRightPressed = true;
        }
        previousDpadRightPressed = gamepad1.dpad_right;

        // Stop Motors (A Button)
        if (gamepad1.a && !previousAPressed) {
            motorsRunning = false;
            if (depositMotorL != null) depositMotorL.setPower(0);
            if (depositMotorR != null) depositMotorR.setPower(0);
            previousAPressed = true;
        }
        previousAPressed = gamepad1.a;

        // ========== GAMEPAD 2: PID TUNING ==========

        // KP Control (D-Pad Up/Down)
        if (gamepad2.dpad_up && !previousDpadUpPressed) {
            kp = Math.max(kp + incrementSize, 0.0);
            updatePIDGains();
            previousDpadUpPressed = true;
        }
        if (gamepad2.dpad_down && !previousDpadUpPressed) {
            kp = Math.max(kp - incrementSize, 0.0);
            updatePIDGains();
            previousDpadUpPressed = true;
        }
        previousDpadUpPressed = (gamepad2.dpad_up || gamepad2.dpad_down);

        // KI Control (D-Pad Left/Right)
        if (gamepad2.dpad_left && !previousDpadLeftPressed) {
            ki = Math.max(ki + incrementSize, 0.0);
            updatePIDGains();
            previousDpadLeftPressed = true;
        }
        if (gamepad2.dpad_right && !previousDpadLeftPressed) {
            ki = Math.max(ki - incrementSize, 0.0);
            updatePIDGains();
            previousDpadLeftPressed = true;
        }
        previousDpadLeftPressed = (gamepad2.dpad_left || gamepad2.dpad_right);

        // KD Control (Left/Right Triggers)
        if (gamepad2.left_trigger > 0.1 && !previousLeftTriggerPressed) {
            kd = Math.max(kd + incrementSize, 0.0);
            updatePIDGains();
            previousLeftTriggerPressed = true;
        } else if (gamepad2.right_trigger > 0.1 && !previousRightTriggerPressed) {
            kd = Math.max(kd - incrementSize, 0.0);
            updatePIDGains();
            previousRightTriggerPressed = true;
        }
        previousLeftTriggerPressed = (gamepad2.left_trigger > 0.1);
        previousRightTriggerPressed = (gamepad2.right_trigger > 0.1);

        // KFF Control (X Button - use gamepad2 D-Pad Up/Down to adjust)
        if (gamepad2.x && !previousXPressed) {
            if (gamepad2.dpad_up) {
                kff = Math.max(kff + incrementSize, 0.0);
            } else if (gamepad2.dpad_down) {
                kff = Math.max(kff - incrementSize, 0.0);
            }
            previousXPressed = true;
        }
        previousXPressed = gamepad2.x;

        // Increment Mode Toggle (B Button)
        if (gamepad2.b && !previousBPressed) {
            coarseMode = !coarseMode;
            incrementSize = coarseMode ? 0.0001 : 0.00001;
            previousBPressed = true;
        }
        previousBPressed = gamepad2.b;

        // Print Values (Y Button)
        if (gamepad2.y && !previousYPressed) {
            printPIDValues();
            previousYPressed = true;
        }
        previousYPressed = gamepad2.y;
    }

    private void updatePIDGains() {
        depositPID = new PIDController(kp, ki, kd);
        depositPID.setOutputLimits(-1.0, 1.0);
        depositPID.setIntegratorLimits(-2000, 2000);
    }

    private void updatePIDController() {
        long currentTime = System.currentTimeMillis();
        double dt = (currentTime - lastUpdateTime) / 1000.0;
        lastUpdateTime = currentTime;

        if (dt > 0.1) dt = 0.016; // Cap dt if there's a lag spike

        if (motorsRunning && depositMotorL != null && depositMotorR != null) {
            double currentVelocity = (depositMotorL.getVelocity() + depositMotorR.getVelocity()) / 2.0;
            double pidOutput = depositPID.update(targetSpeed, currentVelocity, dt);
            double feedForward = targetSpeed * kff;
            double totalPower = pidOutput + feedForward;

            depositMotorL.setPower(-totalPower);
            depositMotorR.setPower(totalPower);
        }
    }

    private void runMotors() {
        if (!motorsRunning) {
            if (depositMotorL != null) depositMotorL.setPower(0);
            if (depositMotorR != null) depositMotorR.setPower(0);
        }
    }

    private void updateTelemetry() {
        panelsTelemetry.debug("╔══════════════════════════════════╗", "");
        panelsTelemetry.debug("║   DEPOSIT PID TUNER               ║", "");
        panelsTelemetry.debug("╚══════════════════════════════════╝", "");
        panelsTelemetry.debug("", "");

        // Status
        String statusStr = motorsRunning ? "🟢 RUNNING" : "🔴 STOPPED";
        panelsTelemetry.debug("Status", statusStr);
        panelsTelemetry.debug("Increment Mode", coarseMode ? "COARSE (0.0001)" : "FINE (0.00001)");
        panelsTelemetry.debug("", "");

        // Target vs Actual
        panelsTelemetry.debug("─── TARGET & ACTUAL ───", "");
        panelsTelemetry.debug("Target Speed", String.format("%.1f ticks/sec", targetSpeed));

        if (depositMotorL != null && depositMotorR != null) {
            double velL = depositMotorL.getVelocity();
            double velR = depositMotorR.getVelocity();
            double velAvg = (velL + velR) / 2.0;
            double error = targetSpeed - velAvg;

            panelsTelemetry.debug("Motor L Velocity", String.format("%.1f", velL));
            panelsTelemetry.debug("Motor R Velocity", String.format("%.1f", velR));
            panelsTelemetry.debug("Average Velocity", String.format("%.1f", velAvg));
            panelsTelemetry.debug("Error", String.format("%.1f (%.1f%%)", error, (error/Math.max(targetSpeed, 1.0)*100)));
        }

        panelsTelemetry.debug("", "");

        // PID Parameters
        panelsTelemetry.debug("─── PID PARAMETERS ───", "");
        panelsTelemetry.debug("KP", String.format("%.8f", kp));
        panelsTelemetry.debug("KI", String.format("%.8f", ki));
        panelsTelemetry.debug("KD", String.format("%.8f", kd));
        panelsTelemetry.debug("KFF", String.format("%.8f", kff));

        panelsTelemetry.debug("", "");

        // Controller 1 Instructions
        panelsTelemetry.debug("╔ GAMEPAD 1 - MOTOR CONTROL ═══╗", "");
        panelsTelemetry.debug("D-Pad ↑/↓", "Target Speed (±10)");
        panelsTelemetry.debug("D-Pad ←", "RESET & STOP");
        panelsTelemetry.debug("D-Pad →", "START MOTORS");
        panelsTelemetry.debug("A Button", "STOP (no reset)");
        panelsTelemetry.debug("", "");

        // Controller 2 Instructions
        panelsTelemetry.debug("╔ GAMEPAD 2 - PID TUNING ══════╗", "");
        panelsTelemetry.debug("D-Pad ↑/↓", "Adjust KP");
        panelsTelemetry.debug("D-Pad ←/→", "Adjust KI");
        panelsTelemetry.debug("LT / RT", "Adjust KD (inc/dec)");
        panelsTelemetry.debug("X + D-Pad", "Adjust KFF");
        panelsTelemetry.debug("B Button", "Toggle Increment");
        panelsTelemetry.debug("Y Button", "Print Values");

        panelsTelemetry.update(telemetry);
    }

    private void printWelcomeMessage() {
        telemetry.addLine("╔════════════════════════════════╗");
        telemetry.addLine("║    DEPOSIT PID TUNER READY!    ║");
        telemetry.addLine("╚════════════════════════════════╝");
        telemetry.addLine("");
        telemetry.addLine("SETUP: Connect 2 controllers");
        telemetry.addLine("");
        telemetry.addLine("QUICK START:");
        telemetry.addLine("  1. GAMEPAD 1: D-Pad Up (set speed)");
        telemetry.addLine("  2. GAMEPAD 1: D-Pad Right (START)");
        telemetry.addLine("  3. GAMEPAD 2: D-Pad to adjust PID");
        telemetry.addLine("  4. GAMEPAD 1: A Button (STOP)");
        telemetry.addLine("");
        telemetry.addLine("See Panels for full control list");
        telemetry.update();
    }

    private void printPIDValues() {
        telemetry.addLine("");
        telemetry.addLine("=== CURRENT PID VALUES ===");
        telemetry.addLine("Target Speed: " + String.format("%.1f", targetSpeed));
        telemetry.addLine("KP: " + String.format("%.8f", kp));
        telemetry.addLine("KI: " + String.format("%.8f", ki));
        telemetry.addLine("KD: " + String.format("%.8f", kd));
        telemetry.addLine("KFF: " + String.format("%.8f", kff));
        telemetry.addLine("");
        telemetry.addLine("Copy these values into PedroAutonomous.java:");
        telemetry.addLine("private static final double DEPOSIT_SPEED_TARGET = " + String.format("%.1f", targetSpeed) + ";");
        telemetry.addLine("private static final double DEPOSIT_KP = " + String.format("%.8f", kp) + ";");
        telemetry.addLine("private static final double DEPOSIT_KI = " + String.format("%.8f", ki) + ";");
        telemetry.addLine("private static final double DEPOSIT_KD = " + String.format("%.8f", kd) + ";");
        telemetry.addLine("private static final double DEPOSIT_KFF = " + String.format("%.8f", kff) + ";");
        telemetry.addLine("");
        telemetry.update();
    }
}

