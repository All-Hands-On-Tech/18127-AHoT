package org.firstinspires.ftc.teamcode.main;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.config.RobotConstants;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;

import java.io.File;
import java.io.FileWriter;
import java.util.ArrayList;
import java.util.Locale;

/**
 * Fully automatic flywheel tuner. Its only job is to produce constants.
 *
 * Press PLAY and leave it alone. It will:
 *  1. Run two open-loop power steps and measure the flywheel (gain, friction offset, time constant, dead time).
 *  2. Compute feedforward (kS, kV) and PI gains (kP, kI) from that model.
 *  3. Spin to RobotConstants.FLYWHEEL_TICKS_PER_SECOND and inject fake "shots" (brief torque dips),
 *     measuring dip / overshoot / ringing / recovery time, and nudge kP/kI until recovery is clean.
 *  4. Switches to REAL MODE: holds the target and waits for you to feed real balls. Every real shot
 *     is measured (dip, overshoot, ringing, recovery) and the gains keep tuning from those shots.
 *
 * Gamepad 1 (real mode only):
 *   Hold RIGHT BUMPER : open blocker + run intake + feed servos (feed balls)
 *   B                 : inject a fake shot (no ball needed)
 * You can also just hand-feed balls; shots are detected from the velocity drop either way.
 *
 * Results are shown in telemetry, written to the robot log, and saved to
 *   /sdcard/FIRST/flywheel_constants.txt
 *
 * Control law the constants are meant for (copy into your real code):
 *
 *   double err = target - velocity;                       // ticks/s
 *   integral += err * dt;
 *   integral = clamp(integral, -0.25 / kI, 0.25 / kI);    // anti-windup
 *   double u = kS + kV * target + kP * err + kI * integral;
 *   motor.setPower(clamp(u * 12.0 / batteryVoltage, 0, 1));
 *
 * (Use full power while err > 15% of target for fast spin-up/shot recovery and reset integral then.)
 */
@TeleOp(name = "Tuner: Flywheel PID", group = "Tuning")
public class FlywheelPidTunerTeleOp extends OpMode {

    private enum State { IDENTIFY_LOW, IDENTIFY_HIGH, SPIN_UP, VALIDATE, REAL, FAILED }

    // ---- knobs (defaults should be fine) ----
    private static final double NOMINAL_VOLTAGE = 12.0;
    private static final double U_LOW = 0.40;
    private static final double U_HIGH = 0.80;
    private static final double VEL_FILTER_ALPHA = 0.6;
    private static final double BOOST_FRACTION = 0.15;
    private static final double SETTLED_BAND = 0.025;
    private static final double SHOT_DIP_FRACTION = 0.03;
    private static final double RECOVERY_GOAL_S = 0.30;
    private static final double OVERSHOOT_LIMIT = 0.05;
    private static final int MAX_VALIDATIONS = 8;
    private static final int GOOD_EVENTS_TO_CONVERGE = 2;

    // ---- real-mode feeding (directions are already set in RobotHardware; adjust magnitudes to taste) ----
    private static final double FEED_INTAKE_POWER = 1.0;
    private static final double FEED_SERVO_POWER = 1.0;

    private RobotHardware hardware;
    private DcMotorEx left, right;
    private VoltageSensor battery;

    private State state = State.IDENTIFY_LOW;
    private String status = "Waiting for PLAY";

    private double target;
    private double vel = 0.0;
    private boolean velInit = false;
    private double lastTime = 0.0;
    private double battVolts = NOMINAL_VOLTAGE;
    private double lastBattRead = -1.0;
    private double lastOutput = 0.0;

    // model + gains
    private double modelK = 0.0, tau = 0.0, deadTime = 0.0;
    private double kS = 0.0, kV = 0.0, kP = 0.0, kI = 0.0, kD = 0.0;
    private double baseKp = 0.0, baseKi = 0.0;

    // control state
    private double iSum = 0.0;
    private double disturbance = 0.0, disturbanceEnd = 0.0;
    private boolean pendingPulse = false;
    private double steadySince = -1.0, settleSince = -1.0;

    // state machine helpers
    private double stateStart = 0.0, checkT = 0.0, checkV = 0.0, vLow = 0.0;
    private final ArrayList<double[]> samples = new ArrayList<>();
    private int validations = 0, goodStreak = 0;

    // shot / event analysis
    private boolean evtActive = false;
    private double evtStart, evtPeakDip, evtOvershoot, evtInBandSince = -1.0;
    private int evtSignChanges, evtLastSign;
    private double lastDip, lastOvershoot, lastRecovery;
    private int lastRings, eventCount;

    private boolean constantsDirty = false;
    private String savedMessage = "";

    // real mode
    private boolean feeding = false;
    private boolean bPrev = false;
    private boolean evtReal = false;
    private boolean evtFeeding = false;
    private int realShots = 0;
    private double sumDip = 0.0, sumRec = 0.0, worstDip = 0.0, worstRec = 0.0;

    // =====================================================================================
    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        hardware = new RobotHardware(hardwareMap);
        left = hardware.leftFlywheel;
        right = hardware.rightFlywheel;
        battery = hardwareMap.voltageSensor.iterator().hasNext()
                ? hardwareMap.voltageSensor.iterator().next() : null;
        target = RobotConstants.FLYWHEEL_TICKS_PER_SECOND;

        for (DcMotorEx m : new DcMotorEx[]{left, right}) {
            if (m != null) {
                m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            }
        }

        telemetry.addLine("Make sure the flywheel is clear and empty, then press PLAY.");
        telemetry.addData("Target", String.format(Locale.US, "%.0f ticks/s", target));
        telemetry.update();
    }

    @Override
    public void start() {
        double now = now();
        lastTime = now;
        enterState(State.IDENTIFY_LOW, now);
        status = "Identifying (low power step)";
        hardware.setBlockerClosed(true);
    }

    @Override
    public void loop() {
        double now = now();
        double dt = Math.min(0.1, Math.max(0.001, now - lastTime));
        lastTime = now;

        updateVelocity();
        updateBattery(now);

        // ---- real mode controls ----
        boolean wantFeed = state == State.REAL && gamepad1.right_bumper;
        if (wantFeed != feeding) {
            feeding = wantFeed;
            applyFeed(feeding);
        }
        if (gamepad1.b && !bPrev && state == State.REAL) pendingPulse = true;
        bPrev = gamepad1.b;

        switch (state) {
            case IDENTIFY_LOW:  runIdentifyLow(now); break;
            case IDENTIFY_HIGH: runIdentifyHigh(now); break;
            case SPIN_UP:
            case VALIDATE:
            case REAL:           runControl(now, dt); break;
            case FAILED:
            default:            setMotors(0.0); break;
        }

        if (constantsDirty && goodStreak >= GOOD_EVENTS_TO_CONVERGE && state == State.REAL) {
            saveConstants();
            constantsDirty = false;
        }

        sendTelemetry();
    }

    @Override
    public void stop() {
        setMotors(0.0);
        if (hardware != null) hardware.stopMechanisms();
    }

    // =====================================================================================
    // Identification
    // =====================================================================================
    private void runIdentifyLow(double now) {
        setU(U_LOW);
        samples.add(new double[]{now, vel});

        if (stepSettled(now) || now - stateStart > 6.0) {
            if (vel < 50.0) {
                fail("No flywheel velocity detected. Check encoders / motor directions.");
                return;
            }
            vLow = vel;
            analyzeStepDynamics();
            enterState(State.IDENTIFY_HIGH, now);
            status = "Identifying (high power step)";
        }
    }

    private void runIdentifyHigh(double now) {
        setU(U_HIGH);

        if (stepSettled(now) || now - stateStart > 6.0) {
            modelK = (vel - vLow) / (U_HIGH - U_LOW);
            if (modelK <= 1.0) {
                fail("Model gain invalid (flywheel did not speed up with more power).");
                return;
            }
            kS = U_LOW - vLow / modelK;
            kV = 1.0 / modelK;

            // Lambda / IMC tuning for first-order-plus-dead-time: Kp = tau/(K(lambda+L)), Ki = Kp/tau
            double L = Math.max(deadTime, 0.02);
            double lambda = Math.max(1.5 * L, 0.35 * tau);
            kP = tau / (modelK * (lambda + L));
            kI = kP / tau;
            kD = 0.0; // velocity is too noisy for D to help
            baseKp = kP;
            baseKi = kI;

            double needed = kS + kV * target;
            status = needed > 0.97
                    ? String.format(Locale.US, "WARNING: target needs %.2f power, may be unreachable", needed)
                    : "Model identified, spinning up";
            enterState(State.SPIN_UP, now);
        }
    }

    private boolean stepSettled(double now) {
        if (now - checkT < 0.4) return false;
        double delta = Math.abs(vel - checkV);
        checkT = now;
        checkV = vel;
        return (now - stateStart) > 0.8 && delta < 0.015 * Math.max(Math.abs(vel), 50.0);
    }

    private void analyzeStepDynamics() {
        if (samples.size() < 4) { tau = 0.3; deadTime = 0.05; return; }
        double v0 = samples.get(0)[1];
        double dv = vel - v0;
        double tD = 0.0, t63 = 0.0;
        boolean gotD = false, got63 = false;
        for (double[] s : samples) {
            double rel = (s[1] - v0) / dv;
            double t = s[0] - stateStart;
            if (!gotD && rel > 0.02) { tD = t; gotD = true; }
            if (!got63 && rel >= 0.632) { t63 = t; got63 = true; break; }
        }
        deadTime = Math.max(tD, 0.01);
        tau = got63 ? Math.max(t63 - tD, 0.05) : 0.4;
    }

    // =====================================================================================
    // Closed-loop control (internal, only used to test the gains)
    // =====================================================================================
    private void runControl(double now, double dt) {
        double err = target - vel;

        if (pendingPulse) {
            pendingPulse = false;
            double pulse = 0.12;
            double dip = 0.10 * target; // aim for roughly a 10% dip like a real ball
            double dU = dip / (modelK * (1.0 - Math.exp(-pulse / Math.max(tau, 0.05))));
            disturbance = -Math.min(0.5, Math.max(0.05, dU));
            disturbanceEnd = now + pulse;
            beginEvent(now, false);
        }
        if (disturbance != 0.0 && now >= disturbanceEnd) disturbance = 0.0;

        double u;
        if (target <= 0.0) {
            u = 0.0; iSum = 0.0;
        } else if (err > BOOST_FRACTION * target) {
            u = 1.0; iSum = 0.0;
        } else if (err < -BOOST_FRACTION * target) {
            u = 0.0; iSum = 0.0;
        } else {
            if (kI > 1e-9) {
                iSum += err * dt;
                double lim = 0.25 / kI;
                iSum = Math.max(-lim, Math.min(lim, iSum));
            }
            u = kS + kV * target + kP * err + kI * iSum;
        }
        setU(u + disturbance);

        boolean inBand = target > 0 && Math.abs(err) < SETTLED_BAND * target;
        if (inBand) { if (steadySince < 0) steadySince = now; } else steadySince = -1.0;

        // fold the integral term into kS so feedforward matches reality
        if (state == State.REAL && steadySince >= 0 && now - steadySince > 1.0 && kI > 1e-9) {
            double bias = kI * iSum * 0.5;
            if (Math.abs(bias) > 1e-5) {
                kS += bias;
                iSum -= bias / kI;
                constantsDirty = true;
            }
        }

        switch (state) {
            case SPIN_UP:
                status = "Spinning up";
                if (inBand) {
                    if (settleSince < 0) settleSince = now;
                    if (now - settleSince > 0.6) {
                        validations++;
                        enterState(State.VALIDATE, now);
                        pendingPulse = true;
                        status = "Validating (test shot " + validations + "/" + MAX_VALIDATIONS + ")";
                    }
                } else {
                    settleSince = -1.0;
                }
                break;
            case VALIDATE:
                trackEvent(now, err);
                break;
            case REAL:
                if (feeding) status = "REAL MODE - feeding";
                else status = goodStreak >= GOOD_EVENTS_TO_CONVERGE
                        ? "REAL MODE - ready, hold RIGHT BUMPER to feed (constants saved)"
                        : "REAL MODE - ready, hold RIGHT BUMPER to feed (still adapting)";
                if (!evtActive && target > 0 && err > SHOT_DIP_FRACTION * target
                        && steadySince < 0 && now - stateStart > 0.5) {
                    beginEvent(now, true);
                }
                trackEvent(now, err);
                break;
            default:
                break;
        }
    }

    // =====================================================================================
    // Shot analysis + adaptation
    // =====================================================================================
    private void beginEvent(double now, boolean real) {
        evtReal = real;
        evtFeeding = feeding;
        evtActive = true;
        evtStart = now;
        evtPeakDip = 0.0;
        evtOvershoot = 0.0;
        evtInBandSince = -1.0;
        evtSignChanges = 0;
        evtLastSign = 0;
    }

    private void trackEvent(double now, double err) {
        if (!evtActive || target <= 0) return;
        evtFeeding |= feeding;

        evtPeakDip = Math.max(evtPeakDip, err);
        evtOvershoot = Math.max(evtOvershoot, -err);

        if (Math.abs(err) > 0.015 * target) {
            int sign = err > 0 ? 1 : -1;
            if (evtLastSign != 0 && sign != evtLastSign) evtSignChanges++;
            evtLastSign = sign;
        }

        boolean inBand = Math.abs(err) < SETTLED_BAND * target && disturbance == 0.0;
        if (inBand) { if (evtInBandSince < 0) evtInBandSince = now; } else evtInBandSince = -1.0;

        boolean recovered = evtInBandSince >= 0 && now - evtInBandSince > 0.15;
        boolean timedOut = now - evtStart > 2.0;
        if (recovered || timedOut) {
            lastRecovery = timedOut ? 2.0 : (evtInBandSince - evtStart);
            lastDip = evtPeakDip / target;
            lastOvershoot = evtOvershoot / target;
            lastRings = evtSignChanges;
            evtActive = false;
            eventCount++;
            if (evtReal && state == State.REAL) recordRealShot();
            // while balls are still being fed continuously the flywheel can't fully recover,
            // so a timeout then says nothing about the gains; don't adapt from it
            boolean skipAdapt = timedOut && evtFeeding;
            if (!skipAdapt) {
                adaptGains();
                constantsDirty = true;
            }

            if (state == State.VALIDATE) {
                boolean done = goodStreak >= GOOD_EVENTS_TO_CONVERGE || validations >= MAX_VALIDATIONS;
                enterState(done ? State.REAL : State.SPIN_UP, now);
                if (done) { saveConstants(); constantsDirty = false; }
            }
        }
    }

    private void recordRealShot() {
        realShots++;
        sumDip += lastDip;
        sumRec += lastRecovery;
        worstDip = Math.max(worstDip, lastDip);
        worstRec = Math.max(worstRec, lastRecovery);
    }

    private void applyFeed(boolean on) {
        hardware.setBlockerClosed(!on);
        if (hardware.intakeMotor != null) hardware.intakeMotor.setPower(on ? FEED_INTAKE_POWER : 0.0);
        if (hardware.continuousServoLeft != null) hardware.continuousServoLeft.setPower(on ? FEED_SERVO_POWER : 0.0);
        if (hardware.continuousServoRight != null) hardware.continuousServoRight.setPower(on ? FEED_SERVO_POWER : 0.0);
    }

    private void adaptGains() {
        boolean ringing = lastRings >= 3 || lastOvershoot > OVERSHOOT_LIMIT;
        boolean slow = lastRecovery > RECOVERY_GOAL_S;
        if (ringing) { kP *= 0.85; kI *= 0.70; goodStreak = 0; }
        else if (slow) { kP *= 1.20; kI *= 1.25; goodStreak = 0; }
        else goodStreak++;
        kP = clamp(kP, 0.3 * baseKp, 4.0 * baseKp);
        kI = clamp(kI, 0.2 * baseKi, 6.0 * baseKi);
    }

    // =====================================================================================
    // Output of constants
    // =====================================================================================
    private String constantsText() {
        // The SDK's built-in setVelocityPIDF scales gains by 32767; shown for convenience only.
        double s = 32767.0;
        StringBuilder sb = new StringBuilder();
        sb.append("// Flywheel constants (target ").append(String.format(Locale.US, "%.0f", target)).append(" ticks/s)\n");
        sb.append("// For the custom controller (power per tick/s, voltage-normalised to 12V):\n");
        sb.append(String.format(Locale.US, "public static double FLYWHEEL_KS = %.6f;\n", kS));
        sb.append(String.format(Locale.US, "public static double FLYWHEEL_KV = %.9f;\n", kV));
        sb.append(String.format(Locale.US, "public static double FLYWHEEL_KP = %.9f;\n", kP));
        sb.append(String.format(Locale.US, "public static double FLYWHEEL_KI = %.9f;\n", kI));
        sb.append(String.format(Locale.US, "public static double FLYWHEEL_KD = %.9f;\n", kD));
        sb.append("// Approx. equivalents if you use DcMotorEx.setVelocityPIDF (no kS, no voltage comp):\n");
        sb.append(String.format(Locale.US, "// P=%.4f I=%.4f D=%.4f F=%.4f\n", kP * s, kI * s, kD * s, kV * s));
        sb.append(String.format(Locale.US, "// model: K=%.1f ticks/s per power, tau=%.3fs, deadTime=%.3fs\n", modelK, tau, deadTime));
        sb.append(String.format(Locale.US, "// last shot: dip=%.1f%% overshoot=%.1f%% recovery=%.2fs rings=%d\n",
                lastDip * 100, lastOvershoot * 100, lastRecovery, lastRings));
        if (realShots > 0) {
            sb.append(String.format(Locale.US, "// real shots: n=%d avgDip=%.1f%% worstDip=%.1f%% avgRecovery=%.2fs worstRecovery=%.2fs\n",
                    realShots, 100.0 * sumDip / realShots, 100.0 * worstDip, sumRec / realShots, worstRec));
        }
        return sb.toString();
    }

    private void saveConstants() {
        String text = constantsText();
        RobotLog.ii("FlywheelTuner", "\n" + text);
        try {
            File file = new File(AppUtil.FIRST_FOLDER, "flywheel_constants.txt");
            FileWriter w = new FileWriter(file, false);
            w.write(text);
            w.close();
            savedMessage = "Saved to " + file.getAbsolutePath();
        } catch (Exception e) {
            savedMessage = "Could not save file: " + e.getMessage();
        }
    }

    private void sendTelemetry() {
        double error = target - vel;

        telemetry.addData("graph/flywheel_target_ticks_per_s", target);
        telemetry.addData("graph/flywheel_avg_ticks_per_s", vel);
        telemetry.addData("graph/flywheel_error_ticks_per_s", error);

        telemetry.addData("State", state);
        telemetry.addData("Status", status);
        telemetry.addData("Target / Actual", String.format(Locale.US, "%.0f / %.0f ticks/s", target, vel));
        telemetry.addData("Battery", String.format(Locale.US, "%.2f V", battVolts));
        telemetry.addData("Model", String.format(Locale.US, "K=%.0f tau=%.3fs L=%.3fs", modelK, tau, deadTime));
        telemetry.addData("Last shot", String.format(Locale.US, "dip %.1f%%  over %.1f%%  rec %.2fs  rings %d",
                lastDip * 100, lastOvershoot * 100, lastRecovery, lastRings));

        if (state == State.REAL) {
            telemetry.addLine("---- REAL MODE: hold RIGHT BUMPER to feed, B = fake shot ----");
            telemetry.addData("Feeding", feeding);
            telemetry.addData("Real shots analysed", realShots);
            if (realShots > 0) {
                telemetry.addData("Avg / worst dip", String.format(Locale.US, "%.1f%% / %.1f%%",
                        100.0 * sumDip / realShots, 100.0 * worstDip));
                telemetry.addData("Avg / worst recovery", String.format(Locale.US, "%.2fs / %.2fs",
                        sumRec / realShots, worstRec));
            }
        }

        if (state == State.REAL || state == State.VALIDATE || state == State.SPIN_UP) {
            telemetry.addLine("---- CONSTANTS ----");
            telemetry.addData("kS", String.format(Locale.US, "%.6f", kS));
            telemetry.addData("kV", String.format(Locale.US, "%.9f", kV));
            telemetry.addData("kP", String.format(Locale.US, "%.9f", kP));
            telemetry.addData("kI", String.format(Locale.US, "%.9f", kI));
            telemetry.addData("kD", String.format(Locale.US, "%.9f", kD));
            if (!savedMessage.isEmpty()) telemetry.addData("File", savedMessage);
        }
        telemetry.update();
    }

    // =====================================================================================
    // Helpers
    // =====================================================================================
    private void enterState(State s, double now) {
        state = s;
        stateStart = now;
        samples.clear();
        checkT = now;
        checkV = vel;
        settleSince = -1.0;
        steadySince = -1.0;
        if (s == State.IDENTIFY_LOW) samples.add(new double[]{now, vel});
    }

    private void fail(String msg) {
        state = State.FAILED;
        status = "FAILED: " + msg;
        setMotors(0.0);
    }

    private void updateVelocity() {
        double sum = 0.0;
        int n = 0;
        if (left != null) { sum += left.getVelocity(); n++; }
        if (right != null) { sum += right.getVelocity(); n++; }
        double raw = n > 0 ? sum / n : 0.0;
        if (!velInit) { vel = raw; velInit = true; }
        else vel += VEL_FILTER_ALPHA * (raw - vel);
    }

    private void updateBattery(double now) {
        if (battery != null && now - lastBattRead > 0.1) {
            double v = battery.getVoltage();
            if (v > 5.0) battVolts = v;
            lastBattRead = now;
        }
    }

    /** Voltage-normalised command: u is the power you would use at 12 V. */
    private void setU(double u) {
        double p = clamp(u * NOMINAL_VOLTAGE / battVolts, 0.0, 1.0);
        lastOutput = p;
        setMotors(p);
    }

    private void setMotors(double p) {
        if (left != null) left.setPower(p);
        if (right != null) right.setPower(p);
    }

    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }

    private static double now() {
        return System.nanoTime() / 1e9;
    }
}