package org.firstinspires.ftc.teamcode.Old;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.auxiliary.FlywheelController;
import org.firstinspires.ftc.teamcode.config.RobotConstants;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;

import java.io.File;
import java.io.FileWriter;
import java.util.Locale;

/**
 * Automatic flywheel PIDF tuner, driven by shots. Press PLAY, let the wheel
 * spin up; the tuner measures the steady-state power to hold target, sets
 * FLYWHEEL_KV from that, then injects a fake shot (brief power cut) and keeps
 * adapting KP/KI from the dip / ringing / recovery of that shot plus any real
 * shots you feed through (hold gamepad1 RIGHT BUMPER).
 *
 * When two consecutive shots recover cleanly the gains stop changing; the
 * 4-variable snippet is shown on telemetry / the robot log and saved to
 * /sdcard/FIRST/flywheel_constants.txt, and the constants are already live in
 * RobotConstants (copy them into the file when happy).
 */
@TeleOp(name = "Tuner: Flywheel PIDF", group = "Tuning")
public class FlywheelPidTunerTeleOp extends OpMode {

    private enum State { SPIN_UP, TUNE, DONE, FAILED }

    private static final double SETTLED_BAND = 0.02;      // "at speed" / "recovered" band, fraction of target
    private static final double SHOT_DIP_FRACTION = 0.03; // a dip bigger than this counts as a shot
    private static final double FAKE_SHOT_CUT_S = 0.12;   // fake shot = power cut for this long
    private static final double SHOT_TIMEOUT_S = 2.0;
    private static final double OVERSHOOT_LIMIT = 0.05;   // ringing thresholds for gain adaptation
    private static final int RING_LIMIT = 3;
    private static final double RECOVERY_GOAL_S = 0.35;
    private static final int GOOD_SHOTS_TO_DONE = 3;

    private RobotHardware hardware;
    private FlywheelController flywheel;

    private State state = State.SPIN_UP;
    private String status = "Spinning up";
    private final ElapsedTimeFacade clock = new ElapsedTimeFacade();
    private double lastTime = 0.0;

    private double target;
    private double vel = 0.0;
    private boolean velInit = false;

    // gains currently being tuned (written into RobotConstants when changed)
    private double kv, kp, ki, kd;

    // control state (mirrors FlywheelController's law)
    private double integral = 0.0;
    private double baseKp, baseKi;

    // shot capture
    private boolean shotActive = false, shotReal = false;
    private double shotStart, shotPeakDip, shotOvershoot, shotInBandSince = -1.0;
    private int shotSignChanges, shotLastSign = 0;
    private double lastDip, lastOvershoot, lastRecovery;
    private int lastRings, shotCount, goodStreak, realShots = 0;

    private double nextFakeShotAt = Double.POSITIVE_INFINITY;
    private double fakeShotEnd = -1.0;
    private boolean feedPrev = false;
    private boolean bPrev = false;
    private String savedMessage = "";

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        hardware = new RobotHardware(hardwareMap);
        flywheel = new FlywheelController(hardware.shooterMotor, null);
        target = RobotConstants.FLYWHEEL_TICKS_PER_SECOND;

        kv = RobotConstants.FLYWHEEL_KV;
        kp = RobotConstants.FLYWHEEL_KP;
        ki = RobotConstants.FLYWHEEL_KI;
        kd = 0.0; // velocity is too noisy for D to help a flywheel

        // Seeds for the shot-based adaptation: any positive P/I in constants, else KV-based guesses.
        if (kp <= 0.0) {
            kp = 2.0 / Math.max(kv, 1e-6) * 1e-6;
        }
        ki = ki > 0.0 ? ki : kp * 10.0;
        baseKp = kp;
        baseKi = ki;

        telemetry.addLine("Flywheel clear and empty, then press PLAY. Feed real shots with gamepad1 RIGHT BUMPER.");
        telemetry.update();
    }

    @Override
    public void start() {
        hardware.setBlockerClosed(true);
        lastTime = clock.now();
    }

    @Override
    public void loop() {
        double now = clock.now();
        double dt = Math.min(0.1, Math.max(0.001, now - lastTime));
        lastTime = now;

        // Velocity tracking (same low-pass shape as the controller).
        double raw = hardware.shooterMotor != null ? hardware.shooterMotor.getVelocity() : 0.0;
        if (!velInit) { vel = raw; velInit = true; }
        else { vel += 0.6 * (raw - vel); }

        // Controls: hold RIGHT BUMPER to feed real balls; B = fake shot now.
        boolean feeding = gamepad1.right_bumper;
        if (feeding != feedPrev) {
            applyFeed(feeding);
            feedPrev = feeding;
        }
        if (gamepad1.b && !bPrev && state == State.TUNE && !shotActive) {
            triggerFakeShot(now);
        }
        bPrev = gamepad1.b;

        if (state == State.SPIN_UP || state == State.TUNE) {
            runControl(now, dt);

            if (state == State.SPIN_UP) {
                boolean atSpeed = Math.abs(target - vel) < SETTLED_BAND * target;
                if (!atSpeed) {
                    shotInBandSince = -1.0;
                } else if (shotInBandSince < 0) {
                    shotInBandSince = now;
                } else if (now - shotInBandSince > 0.8) {
                    // Steady-state power holds the wheel at speed: kv = power / target.
                    double steadyPower = flywheel.getOutputPower();
                    if (steadyPower > 0.02 && steadyPower < 0.97) {
                        kv = steadyPower / target;
                        writeConstants();
                    }
                    status = "KV = steady power / target; starting shot tuning";
                    state = State.TUNE;
                    nextFakeShotAt = 0.0;
                    fakeShotEnd = -1.0;
                }
            } else {
                trackShot(now);

                // Periodic fake shots whenever the wheel is settled and no
                // balls are being fed through.
                if (!shotActive && !feeding && fakeShotEnd < 0.0 && now >= nextFakeShotAt) {
                    triggerFakeShot(now);
                }

                if (goodStreak >= GOOD_SHOTS_TO_DONE && shotCount > 0) {
                    writeConstants();
                    saveSnippet();
                    state = State.DONE;
                    status = "Tuned from " + shotCount + " shots - constants live in RobotConstants";
                    applyFeed(false);
                    feedPrev = false;
                }
            }
        } else {
            setMotors(0.0);
        }

        sendTelemetry();
    }

    @Override
    public void stop() {
        setMotors(0.0);
        if (hardware != null) {
            hardware.stopMechanisms();
        }
    }

    // =========================================================================
    // Control: the exact same PIDF law the FlywheelController runs
    // =========================================================================
    private void runControl(double now, double dt) {
        double err = target - vel;

        // Fake shot = brief full-power cut, like a ball passing through.
        boolean cutting = fakeShotEnd > 0.0 && now < fakeShotEnd;

        double u;
        if (target <= 0.0) {
            u = 0.0;
            integral = 0.0;
        } else if (err > 0.15 * target) {
            u = 1.0;
            integral = 0.0;
        } else if (err < -0.15 * target) {
            u = 0.0;
            integral = 0.0;
        } else {
            integral += err * dt;
            if (ki > 1e-9) {
                double lim = 0.25 / ki;
                integral = Math.max(-lim, Math.min(lim, integral));
            }
            u = kv * target + kp * err + ki * integral;
        }

        setMotors(cutting ? 0.0 : Math.max(0.0, Math.min(1.0, u)));

        if (!shotActive) {
            if (cutting) {
                // Fake shot load window still open; the shot event starts when it ends.
            } else if (err > SHOT_DIP_FRACTION * target
                    && (fakeShotEnd <= 0.0 || now >= fakeShotEnd)) {
                // A dip bigger than the threshold outside a fake-shot window is a real shot.
                beginShot(now, true);
            } else if (fakeShotEnd > 0.0 && now >= fakeShotEnd) {
                beginShot(now, false);
            }
        }
    }

    private void triggerFakeShot(double now) {
        fakeShotEnd = now + FAKE_SHOT_CUT_S;
        nextFakeShotAt = Double.POSITIVE_INFINITY; // re-armed when the shot completes
    }

    private void beginShot(double now, boolean real) {
        shotActive = true;
        shotReal = real;
        shotStart = now;
        shotPeakDip = 0.0;
        shotOvershoot = 0.0;
        shotInBandSince = -1.0;
        shotSignChanges = 0;
        shotLastSign = 0;
        fakeShotEnd = -1.0;
    }

    // =========================================================================
    // Shot analysis + adaptation
    // =========================================================================
    private void trackShot(double now) {
        if (!shotActive || target <= 0) {
            return;
        }

        double err = target - vel;
        shotPeakDip = Math.max(shotPeakDip, err);
        shotOvershoot = Math.max(shotOvershoot, -err);

        if (Math.abs(err) > 0.015 * target) {
            int sign = err > 0 ? 1 : -1;
            if (shotLastSign != 0 && sign != shotLastSign) {
                shotSignChanges++;
            }
            shotLastSign = sign;
        }

        boolean inBand = Math.abs(err) < SETTLED_BAND * target;
        if (inBand) {
            if (shotInBandSince < 0) {
                shotInBandSince = now;
            }
        } else {
            shotInBandSince = -1.0;
        }

        boolean recovered = shotInBandSince >= 0 && now - shotInBandSince > 0.15;
        boolean timedOut = now - shotStart > SHOT_TIMEOUT_S;
        if (recovered || timedOut) {
            lastRecovery = timedOut ? SHOT_TIMEOUT_S : (shotInBandSince - shotStart);
            lastDip = shotPeakDip / target;
            lastOvershoot = shotOvershoot / target;
            lastRings = shotSignChanges;
            shotActive = false;
            shotCount++;
            if (shotReal) {
                realShots++;
            }
            adaptGains();
            // Schedule the next fake shot shortly after this one completes.
            nextFakeShotAt = now + 0.7;
        }
    }

    private void adaptGains() {
        boolean ringing = lastRings >= RING_LIMIT || lastOvershoot > OVERSHOOT_LIMIT;
        boolean slow = lastRecovery > RECOVERY_GOAL_S;
        if (ringing) {
            kp *= 0.85;
            ki *= 0.70;
            goodStreak = 0;
        } else if (slow) {
            kp *= 1.20;
            ki *= 1.25;
            goodStreak = 0;
        } else {
            goodStreak++;
        }
        kp = clamp(kp, 0.3 * baseKp, 4.0 * baseKp);
        ki = clamp(ki, 0.2 * baseKi, 6.0 * baseKi);
        writeConstants();
    }

    // =========================================================================
    // Output
    // =========================================================================
    private void writeConstants() {
        RobotConstants.FLYWHEEL_KV = kv;
        RobotConstants.FLYWHEEL_KP = kp;
        RobotConstants.FLYWHEEL_KI = ki;
        RobotConstants.FLYWHEEL_KD = kd;
    }

    private void saveSnippet() {
        String text = String.format(Locale.US,
                "// Flywheel PIDF (target %.0f ticks/s, tuned from %d shots (%d real))%n"
                        + "public static double FLYWHEEL_KV = %.9f;%n"
                        + "public static double FLYWHEEL_KP = %.9f;%n"
                        + "public static double FLYWHEEL_KI = %.9f;%n"
                        + "public static double FLYWHEEL_KD = %.9f;%n",
                target, shotCount, realShots, kv, kp, ki, kd);
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
        telemetry.addData("Shots analysed", String.format(Locale.US, "%d (%d real)", shotCount, realShots));
        if (shotCount > 0) {
            telemetry.addData("Last shot", String.format(Locale.US, "dip %.1f%%  over %.1f%%  rec %.2fs  rings %d%s",
                    lastDip * 100, lastOvershoot * 100, lastRecovery, lastRings, shotReal ? " (real)" : ""));
        }
        telemetry.addData("Good streak", goodStreak + " / " + GOOD_SHOTS_TO_DONE);

        telemetry.addLine("---- 4-VARIABLE PIDF ----");
        telemetry.addData("FLYWHEEL_KV", String.format(Locale.US, "%.9f", kv));
        telemetry.addData("FLYWHEEL_KP", String.format(Locale.US, "%.9f", kp));
        telemetry.addData("FLYWHEEL_KI", String.format(Locale.US, "%.9f", ki));
        telemetry.addData("FLYWHEEL_KD", String.format(Locale.US, "%.9f", kd));
        if (!savedMessage.isEmpty()) {
            telemetry.addData("File", savedMessage);
        }
        telemetry.addData("Controls", "RB hold = feed real balls, B = fake shot");
        telemetry.update();
    }

    // =========================================================================
    // Helpers
    // =========================================================================
    private void applyFeed(boolean on) {
        hardware.setBlockerClosed(!on);
        if (hardware.intakeMotor != null) {
            hardware.intakeMotor.setPower(on ? 1.0 : 0.0);
        }
        if (hardware.continuousServoLeft != null) {
            hardware.continuousServoLeft.setPower(on ? 1.0 : 0.0);
        }
        if (hardware.continuousServoRight != null) {
            hardware.continuousServoRight.setPower(on ? 1.0 : 0.0);
        }
    }

    private void setMotors(double p) {
        if (hardware.shooterMotor != null) {
            hardware.shooterMotor.setPower(p);
        }
    }

    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }

    /** Tiny wrapper so the tuner's time source stays swappable for tests. */
    private static final class ElapsedTimeFacade {
        private final com.qualcomm.robotcore.util.ElapsedTime timer = new com.qualcomm.robotcore.util.ElapsedTime();

        double now() {
            return timer.seconds();
        }
    }
}
