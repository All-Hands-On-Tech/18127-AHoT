package org.firstinspires.ftc.teamcode.Trowel.Drive;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Trowel.common.Odometry;
import org.firstinspires.ftc.teamcode.Trowel.Configs.TrowelHardware;

@Configurable
@TeleOp(name = "Randy Butter Knubs", group = "Trowel")
public class TrowelTeleOp extends OpMode {

    // ══════════════════════════════════════════════════════════════
    // BROWNOUT PROTECTION — Simple threshold
    // ══════════════════════════════════════════════════════════════

    public static double VOLTAGE_BROWNOUT_THRESHOLD = 8.5;
    public static double VOLTAGE_RECOVERY_THRESHOLD = 10.0;
    public static double BROWNOUT_DRIVE_SCALE = 0.8;
    public static double BROWNOUT_INTAKE_SCALE = 0.33;
    public static int VOLTAGE_SAMPLE_COUNT = 5;

    // ══════════════════════════════════════════════════════════════
    // FLYWHEEL CONFIGURATION
    // ══════════════════════════════════════════════════════════════

    public static double FLYWHEEL_DEFAULT_TARGET = 525.0;
    public static double FLYWHEEL_MIN = 300.0;
    public static double FLYWHEEL_MAX = 800.0;
    public static double FLYWHEEL_STEP = 5.0;
    public static double SHOOT_BOOST = 40.0;
    public static double RAMP_RATE = 600.0;

    public static double PIDF_P = 240.0;
    public static double PIDF_I = 0.0;
    public static double PIDF_D = 0.0;
    public static double PIDF_F = 23.1;

    public static double RECOVERY_P = 38.0;
    public static double RECOVERY_THRESHOLD = 40.0;
    public static double RECOVERY_EXIT = 15.0;

    // ══════════════════════════════════════════════════════════════
    // DRIVE CONFIGURATION
    // ══════════════════════════════════════════════════════════════

    public static double SLOW_MODE_MULTIPLIER = 0.4;
    public static double DRIVE_DEADZONE = 0.05;

    // ══════════════════════════════════════════════════════════════
    // AUTO-AIM HEADING LOCK CONFIGURATION
    // Pedro heading P = 1.5 on radians, no D (causes oscillation)
    // Correction negated to match mecanum mixing direction.
    // ══════════════════════════════════════════════════════════════

    public static double AIM_P = 1.5;
    public static double AIM_MAX_POWER = 0.6;
    public static double AIM_DEADZONE_DEG = 2.0;

    // ══════════════════════════════════════════════════════════════
    // SERVO & INTAKE CONFIGURATION
    // ══════════════════════════════════════════════════════════════

    public static double SERVO_IDLE = 0.3;
    public static double SERVO_SHOOT = 0.95;
    public static double INTAKE_POWER = 1.0;

    public static boolean DEPOSIT1_REVERSED = false;
    public static boolean DEPOSIT2_REVERSED = false;

    // ══════════════════════════════════════════════════════════════
    // STATE VARIABLES
    // ══════════════════════════════════════════════════════════════

    private TrowelHardware robot;
    private Odometry odometry;
    private ElapsedTime loopTimer = new ElapsedTime();

    private boolean odoEnabled = false;
    private boolean flywheelOn = false;
    private boolean inRecoveryMode = false;

    private double flywheelTarget = FLYWHEEL_DEFAULT_TARGET;

    private boolean prevFlywheelToggle = false;
    private boolean prevDpadUp = false;
    private boolean prevDpadDown = false;

    private double currentRampTarget = 0.0;
    private double lastLoopTime = 0.0;
    private double lastP = -1;

    private double vel1 = 0, vel2 = 0, avgVel = 0, commandedTarget = 0;
    private double flPower = 0, frPower = 0, blPower = 0, brPower = 0;
    private double intake1Power = 0, intake2Power = 0;
    private double loopTimeMs = 0;
    private boolean isSlowMode = false;

    // Brownout state
    private VoltageSensor voltageSensor;
    private double[] voltageSamples;
    private int voltageSampleIndex = 0;
    private double smoothedVoltage = 12.0;
    private boolean inBrownout = false;

    // Auto-aim state
    private double savedAimHeadingRad = Double.NaN;
    private boolean aimLockActive = false;
    private double aimCorrectionPower = 0.0;
    private double aimErrorDeg = 0.0;
    private double currentHeadingDeg = 0.0;

    // ══════════════════════════════════════════════════════════════
    // INITIALIZATION
    // ══════════════════════════════════════════════════════════════

    @Override
    public void init() {
        robot = new TrowelHardware(hardwareMap);
        loopTimer.reset();

        initVoltageSensor();
        initDriveMotors();
        initFlywheelMotors(PIDF_P);
        initOdometry();

        if (robot.transferServo != null) {
            robot.transferServo.setPosition(SERVO_IDLE);
        }

        showControls();
    }

    @Override
    public void init_loop() {
    }

    private void initVoltageSensor() {
        voltageSensor = null;
        double maxVoltage = 0;
        for (VoltageSensor sensor : hardwareMap.voltageSensor) {
            double v = sensor.getVoltage();
            if (v > maxVoltage) {
                maxVoltage = v;
                voltageSensor = sensor;
            }
        }

        voltageSamples = new double[VOLTAGE_SAMPLE_COUNT];
        for (int i = 0; i < VOLTAGE_SAMPLE_COUNT; i++) {
            voltageSamples[i] = maxVoltage > 0 ? maxVoltage : 12.0;
        }
        smoothedVoltage = maxVoltage > 0 ? maxVoltage : 12.0;
    }

    private void showControls() {
        telemetry.addLine("══════════════════════════════");
        telemetry.addLine("    RANDY BUTTER KNUBS");
        telemetry.addLine("══════════════════════════════");
        telemetry.addLine("");
        telemetry.addLine("DRIVER 1 (Gamepad 1):");
        telemetry.addLine("  Left Stick: Move");
        telemetry.addLine("  Right Stick X: Turn");
        telemetry.addLine("  RB hold: Slow mode (40%)");
        telemetry.addLine("  LB hold: Auto-aim heading lock");
        telemetry.addLine("  RT: Save current heading for aim lock");
        telemetry.addLine("  LT: Shoot");
        telemetry.addLine("");
        telemetry.addLine("DRIVER 2 (Gamepad 2):");
        telemetry.addLine("  X: Toggle flywheel");
        telemetry.addLine("  Dpad Up/Down: Adjust flywheel speed");
        telemetry.addLine("  LT: Intake in  |  RT: Intake reverse");
        telemetry.addLine("  A: Intake2 in  |  B: Intake2 reverse");
        telemetry.addLine("");
        telemetry.addData("Battery", "%.2fV", smoothedVoltage);
        telemetry.addData("Aim P", "%.2f (Pedro heading P)", AIM_P);
        telemetry.update();
    }

    private void initDriveMotors() {
        for (DcMotor m : new DcMotor[]{robot.frontLeft, robot.frontRight, robot.backLeft, robot.backRight}) {
            if (m != null) {
                m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                m.setPower(0);
            }
        }
    }

    private void initFlywheelMotors(double pValue) {
        try {
            for (DcMotor m : new DcMotor[]{robot.deposit1, robot.deposit2}) {
                if (m instanceof DcMotorEx) {
                    DcMotorEx motor = (DcMotorEx) m;
                    motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    motor.setVelocityPIDFCoefficients(pValue, PIDF_I, PIDF_D, PIDF_F);
                    motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                }
            }
            lastP = pValue;
        } catch (Exception ignored) {}
    }

    private void initOdometry() {
        try {
            robot.initPinpoint();
            if (robot.pinpoint != null) {
                odoEnabled = true;
                robot.updatePinpoint();
                odometry = new Odometry(robot, robot.pinpoint);
            } else {
                odometry = new Odometry(robot, null);
            }
        } catch (Exception e) {
            odometry = new Odometry(robot, null);
        }
    }

    // ══════════════════════════════════════════════════════════════
    // START / STOP
    // ══════════════════════════════════════════════════════════════

    @Override
    public void start() {
        loopTimer.reset();
        lastLoopTime = 0;
        currentRampTarget = 0;
        flywheelTarget = FLYWHEEL_DEFAULT_TARGET;
        savedAimHeadingRad = Double.NaN;
    }

    @Override
    public void stop() {
        for (DcMotor m : new DcMotor[]{robot.frontLeft, robot.frontRight, robot.backLeft, robot.backRight}) {
            if (m != null) m.setPower(0);
        }
        if (robot.deposit1 != null) robot.deposit1.setPower(0);
        if (robot.deposit2 != null) robot.deposit2.setPower(0);
        robot.stop();
    }

    // ══════════════════════════════════════════════════════════════
    // MAIN LOOP
    // ══════════════════════════════════════════════════════════════

    @Override
    public void loop() {
        double currentTime = loopTimer.seconds();
        double deltaTime = Math.min(currentTime - lastLoopTime, 0.1);
        loopTimeMs = deltaTime * 1000.0;
        lastLoopTime = currentTime;

        updateVoltage();

        if (odoEnabled && robot.pinpoint != null) {
            robot.updatePinpoint();
            odometry.update();
        }

        if (odoEnabled && odometry != null) {
            Odometry.Position pos = odometry.getPosition();
            currentHeadingDeg = Math.toDegrees(pos.headingRad);
        }

        updateDrive();
        updateFlywheelTarget();
        updateFlywheel(deltaTime);
        updateIntake();
        updateServo();
        updateTelemetry();
    }

    // ══════════════════════════════════════════════════════════════
    // VOLTAGE MONITORING
    // ══════════════════════════════════════════════════════════════

    private void updateVoltage() {
        if (voltageSensor != null) {
            double rawVoltage = voltageSensor.getVoltage();
            voltageSamples[voltageSampleIndex] = rawVoltage;
            voltageSampleIndex = (voltageSampleIndex + 1) % VOLTAGE_SAMPLE_COUNT;

            double sum = 0;
            for (double sample : voltageSamples) sum += sample;
            smoothedVoltage = sum / VOLTAGE_SAMPLE_COUNT;
        }

        if (smoothedVoltage < VOLTAGE_BROWNOUT_THRESHOLD) {
            inBrownout = true;
        } else if (smoothedVoltage > VOLTAGE_RECOVERY_THRESHOLD) {
            inBrownout = false;
        }
    }

    // ══════════════════════════════════════════════════════════════
    // AUTO-AIM HEADING LOCK
    // ══════════════════════════════════════════════════════════════

    /**
     * Saves the exact current heading as the aim target.
     * Called when GP1 RT is pressed.
     */
    private void saveAimHeading() {
        if (odoEnabled && odometry != null) {
            Odometry.Position pos = odometry.getPosition();
            savedAimHeadingRad = pos.headingRad;
        }
    }

    private double normalizeAngleRad(double angle) {
        while (angle > Math.PI) angle -= 2.0 * Math.PI;
        while (angle < -Math.PI) angle += 2.0 * Math.PI;
        return angle;
    }

    /**
     * Pure P controller for heading lock.
     * Correction negated to match mecanum mixing direction.
     */
    private double computeAimCorrection() {
        if (Double.isNaN(savedAimHeadingRad) || !odoEnabled || odometry == null) {
            aimErrorDeg = 0;
            aimCorrectionPower = 0;
            return 0;
        }

        Odometry.Position pos = odometry.getPosition();
        double errorRad = normalizeAngleRad(savedAimHeadingRad - pos.headingRad);
        double errorDeg = Math.toDegrees(errorRad);
        aimErrorDeg = errorDeg;

        if (Math.abs(errorDeg) < AIM_DEADZONE_DEG) {
            aimCorrectionPower = 0;
            return 0;
        }

        double correction = -(AIM_P * errorRad);

        if (correction > AIM_MAX_POWER) correction = AIM_MAX_POWER;
        if (correction < -AIM_MAX_POWER) correction = -AIM_MAX_POWER;

        aimCorrectionPower = correction;
        return correction;
    }

    // ══════════════════════════════════════════════════════════════
    // DRIVE
    // ══════════════════════════════════════════════════════════════

    private void updateDrive() {
        isSlowMode = gamepad1.right_bumper;
        aimLockActive = gamepad1.left_bumper && !Double.isNaN(savedAimHeadingRad) && odoEnabled;

        // GP1 RT saves current heading for aim lock
        if (gamepad1.right_trigger > 0.5) {
            saveAimHeading();
        }

        double fwd = -gamepad1.left_stick_y;
        double str = gamepad1.left_stick_x;
        double rot;

        if (Math.abs(fwd) < DRIVE_DEADZONE) fwd = 0;
        if (Math.abs(str) < DRIVE_DEADZONE) str = 0;

        if (aimLockActive) {
            rot = computeAimCorrection();
        } else {
            rot = gamepad1.right_stick_x;
            if (Math.abs(rot) < DRIVE_DEADZONE) rot = 0;
            aimCorrectionPower = 0;
            aimErrorDeg = 0;
        }

        if (isSlowMode) {
            fwd *= SLOW_MODE_MULTIPLIER;
            str *= SLOW_MODE_MULTIPLIER;
            if (!aimLockActive) {
                rot *= SLOW_MODE_MULTIPLIER;
            }
        }

        double fl = fwd + str + rot;
        double fr = fwd - str - rot;
        double bl = fwd - str + rot;
        double br = fwd + str - rot;

        double max = Math.max(Math.abs(fl), Math.max(Math.abs(fr),
                Math.max(Math.abs(bl), Math.abs(br))));
        if (max > 1.0) {
            fl /= max;
            fr /= max;
            bl /= max;
            br /= max;
        }

        if (inBrownout) {
            fl *= BROWNOUT_DRIVE_SCALE;
            fr *= BROWNOUT_DRIVE_SCALE;
            bl *= BROWNOUT_DRIVE_SCALE;
            br *= BROWNOUT_DRIVE_SCALE;
        }

        flPower = fl;
        frPower = fr;
        blPower = bl;
        brPower = br;

        if (robot.frontLeft != null) robot.frontLeft.setPower(fl);
        if (robot.frontRight != null) robot.frontRight.setPower(fr);
        if (robot.backLeft != null) robot.backLeft.setPower(bl);
        if (robot.backRight != null) robot.backRight.setPower(br);
    }

    // ══════════════════════════════════════════════════════════════
    // FLYWHEEL
    // ══════════════════════════════════════════════════════════════

    private void updateFlywheelTarget() {
        if (gamepad2.dpad_up && !prevDpadUp) {
            flywheelTarget = Math.min(flywheelTarget + FLYWHEEL_STEP, FLYWHEEL_MAX);
        }
        prevDpadUp = gamepad2.dpad_up;

        if (gamepad2.dpad_down && !prevDpadDown) {
            flywheelTarget = Math.max(flywheelTarget - FLYWHEEL_STEP, FLYWHEEL_MIN);
        }
        prevDpadDown = gamepad2.dpad_down;
    }

    private void updateFlywheel(double deltaTime) {
        if (gamepad2.x && !prevFlywheelToggle) {
            flywheelOn = !flywheelOn;
            if (!flywheelOn) stopFlywheel();
        }
        prevFlywheelToggle = gamepad2.x;

        updateFlywheelCommon(deltaTime, gamepad1.left_trigger > 0.5);
    }

    // ══════════════════════════════════════════════════════════════
    // INTAKE
    // ══════════════════════════════════════════════════════════════

    private void updateIntake() {
        double desired1;
        if (gamepad2.left_trigger > 0.3) {
            desired1 = INTAKE_POWER;
        } else if (gamepad2.right_trigger > 0.3) {
            desired1 = -INTAKE_POWER;
        } else {
            desired1 = 0;
        }

        double desired2;
        if (gamepad2.a) {
            desired2 = INTAKE_POWER;
        } else if (gamepad2.b) {
            desired2 = -INTAKE_POWER;
        } else {
            desired2 = 0;
        }

        if (inBrownout) {
            desired1 *= BROWNOUT_INTAKE_SCALE;
            desired2 *= BROWNOUT_INTAKE_SCALE;
        }

        intake1Power = desired1;
        intake2Power = desired2;

        if (robot.intake1 != null) robot.intake1.setPower(intake1Power);
        if (robot.intake2 != null) robot.intake2.setPower(intake2Power);
    }

    // ══════════════════════════════════════════════════════════════
    // SERVO — GP1 LT shoots (heading saved separately via RT)
    // ══════════════════════════════════════════════════════════════

    private void updateServo() {
        if (robot.transferServo != null) {
            boolean shooting = gamepad1.left_trigger > 0.5;
            robot.transferServo.setPosition(shooting ? SERVO_SHOOT : SERVO_IDLE);
        }
    }

    // ══════════════════════════════════════════════════════════════
    // FLYWHEEL VELOCITY CONTROL
    // ══════════════════════════════════════════════════════════════

    private void updateFlywheelCommon(double deltaTime, boolean shooting) {
        vel1 = (robot.deposit1 instanceof DcMotorEx) ? Math.abs(((DcMotorEx) robot.deposit1).getVelocity()) : 0;
        vel2 = (robot.deposit2 instanceof DcMotorEx) ? Math.abs(((DcMotorEx) robot.deposit2).getVelocity()) : 0;
        avgVel = (vel1 + vel2) / 2.0;

        if (!flywheelOn) {
            commandedTarget = 0;
            return;
        }

        double finalTarget = flywheelTarget + (shooting ? SHOOT_BOOST : 0);

        if (currentRampTarget < finalTarget) {
            currentRampTarget = Math.min(currentRampTarget + RAMP_RATE * deltaTime, finalTarget);
        } else if (currentRampTarget > finalTarget) {
            currentRampTarget = Math.max(currentRampTarget - RAMP_RATE * deltaTime, finalTarget);
        }

        double velocityError = currentRampTarget - avgVel;
        if (!inRecoveryMode && velocityError > RECOVERY_THRESHOLD) {
            inRecoveryMode = true;
            updatePIDF(RECOVERY_P);
        } else if (inRecoveryMode && velocityError < RECOVERY_EXIT) {
            inRecoveryMode = false;
            updatePIDF(PIDF_P);
        }

        commandedTarget = currentRampTarget;
        setFlywheelVelocity(commandedTarget);
    }

    private void setFlywheelVelocity(double velocity) {
        if (robot.deposit1 instanceof DcMotorEx) {
            ((DcMotorEx) robot.deposit1).setVelocity(DEPOSIT1_REVERSED ? -velocity : velocity);
        }
        if (robot.deposit2 instanceof DcMotorEx) {
            ((DcMotorEx) robot.deposit2).setVelocity(DEPOSIT2_REVERSED ? -velocity : velocity);
        }
    }

    private void stopFlywheel() {
        setFlywheelVelocity(0);
        currentRampTarget = 0;
        inRecoveryMode = false;
        updatePIDF(PIDF_P);
    }

    private void updatePIDF(double pValue) {
        if (Math.abs(pValue - lastP) < 0.01) return;
        try {
            for (DcMotor m : new DcMotor[]{robot.deposit1, robot.deposit2}) {
                if (m instanceof DcMotorEx) {
                    ((DcMotorEx) m).setVelocityPIDFCoefficients(pValue, PIDF_I, PIDF_D, PIDF_F);
                }
            }
            lastP = pValue;
        } catch (Exception ignored) {}
    }

    // ══════════════════════════════════════════════════════════════
    // TELEMETRY
    // ══════════════════════════════════════════════════════════════

    private void updateTelemetry() {
        if (inBrownout) {
            telemetry.addData("Battery", "%.2fV ⚠ BROWNOUT — drive 80%% intakes 33%%", smoothedVoltage);
        } else {
            telemetry.addData("Battery", "%.2fV", smoothedVoltage);
        }

        telemetry.addLine("");

        // Auto-aim
        telemetry.addLine("── AIM LOCK ──");
        telemetry.addData("Current Heading", "%.1f°", currentHeadingDeg);

        if (Double.isNaN(savedAimHeadingRad)) {
            telemetry.addLine("No heading saved (RT to save)");
        } else {
            telemetry.addData("Saved Target", "%.1f°", Math.toDegrees(savedAimHeadingRad));
            if (aimLockActive) {
                String lockStatus = Math.abs(aimErrorDeg) < AIM_DEADZONE_DEG ? "LOCKED" : "CORRECTING";
                telemetry.addData("Status", "%s (err: %.1f°)", lockStatus, aimErrorDeg);
                telemetry.addData("Correction", "%.3f", aimCorrectionPower);
            } else {
                telemetry.addLine("Hold LB to engage");
            }
        }

        telemetry.addLine("");

        // Flywheel
        telemetry.addLine("── FLYWHEEL ──");
        if (flywheelOn) {
            double error = commandedTarget - avgVel;
            String status = inRecoveryMode ? "RECOVERING" :
                    (currentRampTarget < flywheelTarget - 10) ? "RAMPING" :
                            (Math.abs(error) < 20) ? "READY" : "STABILIZING";

            telemetry.addData("Status", status);
            telemetry.addData("Target", "%.0f [Dpad ±%.0f]", flywheelTarget, FLYWHEEL_STEP);
            telemetry.addData("Velocity", "%.0f / %.0f", avgVel, commandedTarget);
            telemetry.addData("M1 | M2", "%.0f | %.0f", vel1, vel2);
        } else {
            telemetry.addLine("OFF (GP2.X to start)");
            telemetry.addData("Target", "%.0f", flywheelTarget);
        }

        telemetry.addLine("");

        // Drive
        telemetry.addLine("── DRIVE ──");
        String modeStr;
        if (aimLockActive && isSlowMode) {
            modeStr = "AIM LOCK + SLOW";
        } else if (aimLockActive) {
            modeStr = "AIM LOCK";
        } else if (isSlowMode) {
            modeStr = "SLOW (40%)";
        } else {
            modeStr = "NORMAL";
        }
        telemetry.addData("Mode", modeStr);
        telemetry.addData("FL|FR", "%+.2f | %+.2f", flPower, frPower);
        telemetry.addData("BL|BR", "%+.2f | %+.2f", blPower, brPower);

        telemetry.addLine("");

        // Intakes
        String i1 = intake1Power > 0.1 ? "IN" : intake1Power < -0.1 ? "OUT" : "OFF";
        String i2 = intake2Power > 0.1 ? "IN" : intake2Power < -0.1 ? "OUT" : "OFF";
        telemetry.addData("Intakes", "%s | %s", i1, i2);

        // Position
        if (odoEnabled && odometry != null) {
            Odometry.Position p = odometry.getPosition();
            telemetry.addData("Pos", "%.1f, %.1f in  hdg %.1f°",
                    p.xMm / 25.4, p.yMm / 25.4, Math.toDegrees(p.headingRad));
        }

        telemetry.addLine("");
        double hz = loopTimeMs > 0 ? 1000.0 / loopTimeMs : 0;
        telemetry.addData("Loop", "%.1fms (%.0fHz)", loopTimeMs, hz);

        telemetry.update();
    }
}