package org.firstinspires.ftc.teamcode.Trowel.Autonomous;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.Trowel.Configs.TrowelHardware;

/**
 * Simple autonomous that turns 90 degrees left and shoots.
 * Uses traditional tank-style turning with direct motor control.
 * All timing values are configurable through Panels.
 */
@Autonomous(name = "DO not USE", group = "Autonomous")
@Configurable
public class Auto extends OpMode {

    // ══════════════════════════════════════════════════════════════
    // CONFIGURABLE PARAMETERS (editable via Panels)
    // ══════════════════════════════════════════════════════════════

    // Timing parameters
    public static long WAIT_BEFORE_SHOOT_MS = 2000;
    public static long INTAKE_RUN_TIME_MS = 3000;
    public static long TURN_TIME_MS = 1500;

    // Turn power (positive = left turn)
    public static double TURN_POWER = 0.4;

    // Deposit target velocity (same as TeleOp default)
    public static double DEPOSIT_TARGET_VELOCITY = 510.0;

    // PIDF values (same as TeleOp tuned values)
    public static double PIDF_P = 240.0;
    public static double PIDF_I = 0.0;
    public static double PIDF_D = 0.0;
    public static double PIDF_F = 19.0;

    // Servo positions
    public static double SERVO_SHOOT_POSITION = 0.7;
    public static double SERVO_IDLE_POSITION = 0.3;

    // Intake powers
    public static double INTAKE1_POWER = 1.0;
    public static double INTAKE2_POWER = 1.0;

    // ══════════════════════════════════════════════════════════════
    // STATE MACHINE
    // ══════════════════════════════════════════════════════════════

    private enum AutoState {
        INIT,
        SPIN_UP_DEPOSIT,
        TURNING,
        WAITING,
        SHOOTING,
        DONE
    }

    private AutoState currentState = AutoState.INIT;

    // ══════════════════════════════════════════════════════════════
    // HARDWARE & UTILITIES
    // ══════════════════════════════════════════════════════════════

    private TelemetryManager panelsTelemetry;
    private TrowelHardware robot;
    private ElapsedTime timer;
    private ElapsedTime totalTimer;

    // Deposit velocity tracking
    private double deposit1Vel = 0;
    private double deposit2Vel = 0;
    private double avgDepositVel = 0;

    // State tracking
    private long stateStartTime = 0;

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        timer = new ElapsedTime();
        totalTimer = new ElapsedTime();

        telemetry.addLine("═══════════════════════════════");
        telemetry.addLine("   SIMPLE TURN & SHOOT AUTO");
        telemetry.addLine("═══════════════════════════════");
        telemetry.addLine("");
        telemetry.addLine("CONFIGURABLE VALUES:");
        telemetry.addData("  Turn time", TURN_TIME_MS + " ms");
        telemetry.addData("  Turn power", TURN_POWER);
        telemetry.addData("  Wait before shoot", WAIT_BEFORE_SHOOT_MS + " ms");
        telemetry.addData("  Intake run time", INTAKE_RUN_TIME_MS + " ms");
        telemetry.addData("  Deposit velocity", DEPOSIT_TARGET_VELOCITY);
        telemetry.addLine("");
        telemetry.addLine("Press START when ready");
        telemetry.update();

        panelsTelemetry.debug("Status", "Initialized - Ready to start");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void init_loop() {
        // Show current config values during init
        telemetry.addLine("═══════════════════════════════");
        telemetry.addLine("   SIMPLE TURN & SHOOT AUTO");
        telemetry.addLine("═══════════════════════════════");
        telemetry.addLine("");
        telemetry.addLine("CURRENT CONFIG (change in Panels):");
        telemetry.addData("  Turn time", TURN_TIME_MS + " ms");
        telemetry.addData("  Turn power", TURN_POWER);
        telemetry.addData("  Wait before shoot", WAIT_BEFORE_SHOOT_MS + " ms");
        telemetry.addData("  Intake run time", INTAKE_RUN_TIME_MS + " ms");
        telemetry.addData("  Deposit velocity", DEPOSIT_TARGET_VELOCITY);
        telemetry.addData("  Intake1 power", INTAKE1_POWER);
        telemetry.addData("  Intake2 power", INTAKE2_POWER);
        telemetry.addLine("");
        telemetry.addLine("Press START when ready");
        telemetry.update();
    }

    @Override
    public void start() {
        // Initialize hardware
        robot = new TrowelHardware(hardwareMap);
        timer.reset();
        totalTimer.reset();

        // Initialize drive motors
        initDriveMotors();

        // Initialize deposit motors with PIDF
        initDepositMotors();

        // Initialize servo - OPEN for shooting
        if (robot.transferServo != null) {
            robot.transferServo.setPosition(SERVO_SHOOT_POSITION);
        }

        // Intakes off initially
        stopIntakes();

        // Start spinning up deposit
        currentState = AutoState.SPIN_UP_DEPOSIT;
        stateStartTime = System.currentTimeMillis();
        setDepositVelocity(DEPOSIT_TARGET_VELOCITY);

        panelsTelemetry.debug("Status", "Started - Spinning up deposit");
    }

    @Override
    public void loop() {
        // Always read deposit velocities for telemetry
        readDepositVelocities();

        // State machine
        switch (currentState) {
            case SPIN_UP_DEPOSIT:
                // Wait for deposit to spin up (1 second or when velocity is close)
                if (timer.milliseconds() >= 1000 || avgDepositVel >= DEPOSIT_TARGET_VELOCITY * 0.9) {
                    currentState = AutoState.TURNING;
                    timer.reset();
                    stateStartTime = System.currentTimeMillis();
                    startTurning();
                    panelsTelemetry.debug("Status", "Deposit ready - Turning");
                }
                break;

            case TURNING:
                // Turn for configured time
                if (timer.milliseconds() >= TURN_TIME_MS) {
                    currentState = AutoState.WAITING;
                    timer.reset();
                    stateStartTime = System.currentTimeMillis();
                    stopDriveMotors();
                    panelsTelemetry.debug("Status", "Turn complete - Waiting " + WAIT_BEFORE_SHOOT_MS + "ms");
                }
                break;

            case WAITING:
                // Wait configured time before shooting
                if (timer.milliseconds() >= WAIT_BEFORE_SHOOT_MS) {
                    currentState = AutoState.SHOOTING;
                    timer.reset();
                    stateStartTime = System.currentTimeMillis();
                    startIntakes();
                    panelsTelemetry.debug("Status", "Shooting - Intakes ON");
                }
                break;

            case SHOOTING:
                // Run intakes for configured time
                if (timer.milliseconds() >= INTAKE_RUN_TIME_MS) {
                    currentState = AutoState.DONE;
                    stateStartTime = System.currentTimeMillis();
                    stopIntakes();
                    panelsTelemetry.debug("Status", "DONE");
                }
                break;

            case DONE:
                // Auto complete - deposit keeps running
                break;

            default:
                break;
        }

        // Update telemetry
        updateTelemetry();
    }

    @Override
    public void stop() {
        if (robot != null) {
            stopIntakes();
            stopDriveMotors();
            stopDeposit();
            robot.stop();
        }
    }

    // ══════════════════════════════════════════════════════════════
    // DRIVE MOTOR CONTROL
    // ══════════════════════════════════════════════════════════════

    private void initDriveMotors() {
        DcMotor[] driveMotors = {robot.frontLeft, robot.frontRight, robot.backLeft, robot.backRight};
        for (DcMotor m : driveMotors) {
            if (m != null) {
                m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                m.setPower(0);
            }
        }
    }

    private void startTurning() {
        // Turn left: left side backward, right side forward
        double leftPower = -TURN_POWER;
        double rightPower = TURN_POWER;

        if (robot.frontLeft != null) robot.frontLeft.setPower(leftPower);
        if (robot.backLeft != null) robot.backLeft.setPower(leftPower);
        if (robot.frontRight != null) robot.frontRight.setPower(rightPower);
        if (robot.backRight != null) robot.backRight.setPower(rightPower);
    }

    private void stopDriveMotors() {
        if (robot.frontLeft != null) robot.frontLeft.setPower(0);
        if (robot.backLeft != null) robot.backLeft.setPower(0);
        if (robot.frontRight != null) robot.frontRight.setPower(0);
        if (robot.backRight != null) robot.backRight.setPower(0);
    }

    // ══════════════════════════════════════════════════════════════
    // DEPOSIT MOTOR CONTROL
    // ══════════════════════════════════════════════════════════════

    private void initDepositMotors() {
        try {
            if (robot.deposit1 instanceof DcMotorEx) {
                DcMotorEx motor = (DcMotorEx) robot.deposit1;
                motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                motor.setVelocityPIDFCoefficients(PIDF_P, PIDF_I, PIDF_D, PIDF_F);
                motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            }
            if (robot.deposit2 instanceof DcMotorEx) {
                DcMotorEx motor = (DcMotorEx) robot.deposit2;
                motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                motor.setVelocityPIDFCoefficients(PIDF_P, PIDF_I, PIDF_D, PIDF_F);
                motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            }
        } catch (Exception e) {
            panelsTelemetry.debug("Deposit Init Error", e.getMessage());
        }
    }

    private void setDepositVelocity(double velocity) {
        try {
            if (robot.deposit1 instanceof DcMotorEx) {
                ((DcMotorEx) robot.deposit1).setVelocity(velocity);
            }
            if (robot.deposit2 instanceof DcMotorEx) {
                ((DcMotorEx) robot.deposit2).setVelocity(velocity);
            }
        } catch (Exception e) {
            panelsTelemetry.debug("Deposit Velocity Error", e.getMessage());
        }
    }

    private void readDepositVelocities() {
        try {
            if (robot.deposit1 instanceof DcMotorEx) {
                deposit1Vel = Math.abs(((DcMotorEx) robot.deposit1).getVelocity());
            }
            if (robot.deposit2 instanceof DcMotorEx) {
                deposit2Vel = Math.abs(((DcMotorEx) robot.deposit2).getVelocity());
            }
            avgDepositVel = (deposit1Vel + deposit2Vel) / 2.0;
        } catch (Exception ignored) {}
    }

    private void stopDeposit() {
        try {
            if (robot.deposit1 != null) robot.deposit1.setPower(0);
            if (robot.deposit2 != null) robot.deposit2.setPower(0);
        } catch (Exception ignored) {}
    }

    // ══════════════════════════════════════════════════════════════
    // INTAKE CONTROL
    // ══════════════════════════════════════════════════════════════

    private void startIntakes() {
        if (robot.intake1 != null) {
            robot.intake1.setPower(INTAKE1_POWER);
        }
        if (robot.intake2 != null) {
            robot.intake2.setPower(INTAKE2_POWER);
        }
    }

    private void stopIntakes() {
        if (robot.intake1 != null) {
            robot.intake1.setPower(0);
        }
        if (robot.intake2 != null) {
            robot.intake2.setPower(0);
        }
    }

    // ══════════════════════════════════════════════════════════════
    // TELEMETRY
    // ══════════════════════════════════════════════════════════════

    private void updateTelemetry() {
        long stateElapsed = System.currentTimeMillis() - stateStartTime;

        // Panels telemetry
        panelsTelemetry.debug("State", currentState.toString());
        panelsTelemetry.debug("State Time", stateElapsed + " ms");
        panelsTelemetry.debug("Total Time", String.format("%.1f s", totalTimer.seconds()));
        panelsTelemetry.debug("Deposit Target", DEPOSIT_TARGET_VELOCITY);
        panelsTelemetry.debug("Deposit Avg Vel", String.format("%.1f", avgDepositVel));
        panelsTelemetry.debug("Deposit1 Vel", String.format("%.1f", deposit1Vel));
        panelsTelemetry.debug("Deposit2 Vel", String.format("%.1f", deposit2Vel));
        panelsTelemetry.debug("Deposit Error", String.format("%.1f", DEPOSIT_TARGET_VELOCITY - avgDepositVel));
        panelsTelemetry.debug("Deposit %", String.format("%.1f%%", (avgDepositVel / DEPOSIT_TARGET_VELOCITY) * 100));
        panelsTelemetry.update(telemetry);

        // Driver station telemetry
        telemetry.addLine("═══════════════════════════════");
        telemetry.addLine("   SIMPLE TURN & SHOOT AUTO");
        telemetry.addLine("═══════════════════════════════");
        telemetry.addLine("");

        // State info
        telemetry.addLine("── STATE ──");
        telemetry.addData("Current", currentState.toString());
        telemetry.addData("State Time", stateElapsed + " ms");
        telemetry.addData("Total Time", String.format("%.1f s", totalTimer.seconds()));

        // Progress bar for current state
        String progressBar = getProgressBar();
        telemetry.addData("Progress", progressBar);

        telemetry.addLine("");

        // Deposit info
        telemetry.addLine("── DEPOSIT ──");
        telemetry.addData("Target", String.format("%.0f ticks/s", DEPOSIT_TARGET_VELOCITY));
        telemetry.addData("Actual", String.format("%.0f ticks/s (%.1f%%)", avgDepositVel, (avgDepositVel / DEPOSIT_TARGET_VELOCITY) * 100));
        telemetry.addData("Motor 1", String.format("%.0f ticks/s", deposit1Vel));
        telemetry.addData("Motor 2", String.format("%.0f ticks/s", deposit2Vel));
        telemetry.addData("Error", String.format("%.1f ticks/s", DEPOSIT_TARGET_VELOCITY - avgDepositVel));

        String depositStatus = avgDepositVel >= DEPOSIT_TARGET_VELOCITY * 0.95 ? "READY" :
                avgDepositVel >= DEPOSIT_TARGET_VELOCITY * 0.8 ? "SPINNING UP" : "LOW";
        telemetry.addData("Status", depositStatus);

        telemetry.addLine("");

        // Intake status
        telemetry.addLine("── INTAKES ──");
        if (currentState == AutoState.SHOOTING) {
            telemetry.addData("Intake1", String.format("%.2f (RUNNING)", INTAKE1_POWER));
            telemetry.addData("Intake2", String.format("%.2f (RUNNING)", INTAKE2_POWER));
        } else {
            telemetry.addData("Intake1", "OFF");
            telemetry.addData("Intake2", "OFF");
        }

        telemetry.addLine("");

        // Servo status
        telemetry.addLine("── SERVO ──");
        telemetry.addData("Position", SERVO_SHOOT_POSITION + " (OPEN)");

        telemetry.update();
    }

    private String getProgressBar() {
        double progress = 0;
        double max = 1;

        switch (currentState) {
            case SPIN_UP_DEPOSIT:
                progress = timer.milliseconds();
                max = 1000;
                break;
            case TURNING:
                progress = timer.milliseconds();
                max = TURN_TIME_MS;
                break;
            case WAITING:
                progress = timer.milliseconds();
                max = WAIT_BEFORE_SHOOT_MS;
                break;
            case SHOOTING:
                progress = timer.milliseconds();
                max = INTAKE_RUN_TIME_MS;
                break;
            case DONE:
                progress = 1;
                max = 1;
                break;
        }

        double percent = Math.min(progress / max, 1.0);
        int filled = (int) (percent * 20);
        int empty = 20 - filled;

        StringBuilder bar = new StringBuilder("[");
        for (int i = 0; i < filled; i++) bar.append("█");
        for (int i = 0; i < empty; i++) bar.append("░");
        bar.append("] ");
        bar.append(String.format("%.0f%%", percent * 100));

        return bar.toString();
    }
}