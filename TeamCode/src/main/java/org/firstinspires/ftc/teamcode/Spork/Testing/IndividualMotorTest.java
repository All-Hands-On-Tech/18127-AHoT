package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/**
 * Enhanced Individual Motor Test OpMode with RPM Monitoring
 *
 * MOTOR SELECTION (press to toggle):
 * - A: Front Left    B: Front Right    X: Back Left    Y: Back Right
 *
 * POWER CONTROL (for selected motor):
 * - D-Pad UP: +0.05 power      D-Pad DOWN: -0.05 power
 * - D-Pad LEFT: Stop (0.0)     D-Pad RIGHT: Full (1.0)
 * - Left Trigger: -0.01/sec    Right Trigger: +0.01/sec (hold for continuous)
 *
 * TESTING FEATURES:
 * - Left Bumper: Toggle encoder position display
 * - Right Bumper: Reset all encoder positions and stats
 * - Start: Reset average RPM statistics
 *
 * Displays: Power %, RPM, Velocity, Encoder Position, Average RPM, Power Correction Factor
 */
@TeleOp(name = "Individual Motor Test", group = "Testing")
public class IndividualMotorTest extends LinearOpMode {

    private DcMotorEx frontLeft, frontRight, backLeft, backRight;

    // Motor power levels
    private double flPower = 0;
    private double frPower = 0;
    private double blPower = 0;
    private double brPower = 0;

    // Motor selection (which motors are currently active)
    private boolean flSelected = false;
    private boolean frSelected = false;
    private boolean blSelected = false;
    private boolean brSelected = false;

    // RPM tracking
    private static final double TICKS_PER_REV = 537.7; // goBILDA 5202/5203 series (adjust if using different motors)
    private ElapsedTime runtime = new ElapsedTime();

    // Average RPM tracking (for comparing motors at same power)
    private double flRpmSum = 0, frRpmSum = 0, blRpmSum = 0, brRpmSum = 0;
    private int flSamples = 0, frSamples = 0, blSamples = 0, brSamples = 0;

    // Control tracking
    private boolean prevA = false, prevB = false, prevX = false, prevY = false;
    private boolean prevDpadUp = false, prevDpadDown = false;
    private boolean prevDpadLeft = false, prevDpadRight = false;
    private boolean prevLB = false, prevRB = false, prevStart = false;
    private boolean showEncoders = true;

    @Override
    public void runOpMode() {
        // Initialize motors as DcMotorEx for velocity access
        try {
            frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
            frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
            backLeft = hardwareMap.get(DcMotorEx.class, "backLeft");
            backRight = hardwareMap.get(DcMotorEx.class, "backRight");

            // Set motor directions
            frontLeft.setDirection(DcMotor.Direction.FORWARD);
            frontRight.setDirection(DcMotor.Direction.REVERSE);
            backLeft.setDirection(DcMotor.Direction.FORWARD);
            backRight.setDirection(DcMotor.Direction.REVERSE);

            // Set zero power behavior
            frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            // Enable RUN_USING_ENCODER mode for velocity feedback
            frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        } catch (Exception e) {
            telemetry.addData("ERROR", "Failed to initialize: " + e.getMessage());
            telemetry.update();
        }

        telemetry.addLine("=== ENHANCED MOTOR TEST ===");
        telemetry.addLine();
        telemetry.addLine("Press A/B/X/Y to select motors");
        telemetry.addLine("Use D-pad to control power");
        telemetry.addLine("Watch RPM to compare motor speeds");
        telemetry.update();

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            // === MOTOR SELECTION ===
            if (gamepad1.a && !prevA) {
                flSelected = !flSelected;
                if (!flSelected) flPower = 0;
            }
            prevA = gamepad1.a;

            if (gamepad1.b && !prevB) {
                frSelected = !frSelected;
                if (!frSelected) frPower = 0;
            }
            prevB = gamepad1.b;

            if (gamepad1.x && !prevX) {
                blSelected = !blSelected;
                if (!blSelected) blPower = 0;
            }
            prevX = gamepad1.x;

            if (gamepad1.y && !prevY) {
                brSelected = !brSelected;
                if (!brSelected) brPower = 0;
            }
            prevY = gamepad1.y;

            // === POWER CONTROL (for selected motors) ===
            // D-pad up: increase power by 0.05
            if (gamepad1.dpad_up && !prevDpadUp) {
                if (flSelected) flPower = Math.min(1.0, flPower + 0.05);
                if (frSelected) frPower = Math.min(1.0, frPower + 0.05);
                if (blSelected) blPower = Math.min(1.0, blPower + 0.05);
                if (brSelected) brPower = Math.min(1.0, brPower + 0.05);
            }
            prevDpadUp = gamepad1.dpad_up;

            // D-pad down: decrease power by 0.05
            if (gamepad1.dpad_down && !prevDpadDown) {
                if (flSelected) flPower = Math.max(0.0, flPower - 0.05);
                if (frSelected) frPower = Math.max(0.0, frPower - 0.05);
                if (blSelected) blPower = Math.max(0.0, blPower - 0.05);
                if (brSelected) brPower = Math.max(0.0, brPower - 0.05);
            }
            prevDpadDown = gamepad1.dpad_down;

            // D-pad left: stop (0.0)
            if (gamepad1.dpad_left && !prevDpadLeft) {
                if (flSelected) flPower = 0.0;
                if (frSelected) frPower = 0.0;
                if (blSelected) blPower = 0.0;
                if (brSelected) brPower = 0.0;
            }
            prevDpadLeft = gamepad1.dpad_left;

            // D-pad right: full power (1.0)
            if (gamepad1.dpad_right && !prevDpadRight) {
                if (flSelected) flPower = 1.0;
                if (frSelected) frPower = 1.0;
                if (blSelected) blPower = 1.0;
                if (brSelected) brPower = 1.0;
            }
            prevDpadRight = gamepad1.dpad_right;

            // Triggers: fine control (continuous when held)
            double triggerAdjust = 0.01 * (gamepad1.right_trigger - gamepad1.left_trigger);
            if (Math.abs(triggerAdjust) > 0.01) {
                if (flSelected) flPower = Math.max(0.0, Math.min(1.0, flPower + triggerAdjust));
                if (frSelected) frPower = Math.max(0.0, Math.min(1.0, frPower + triggerAdjust));
                if (blSelected) blPower = Math.max(0.0, Math.min(1.0, blPower + triggerAdjust));
                if (brSelected) brPower = Math.max(0.0, Math.min(1.0, brPower + triggerAdjust));
            }

            // === TESTING CONTROLS ===
            // Left bumper: toggle encoder display
            if (gamepad1.left_bumper && !prevLB) {
                showEncoders = !showEncoders;
            }
            prevLB = gamepad1.left_bumper;

            // Right bumper: reset encoders and stats
            if (gamepad1.right_bumper && !prevRB) {
                if (frontLeft != null) {
                    frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                }
                if (frontRight != null) {
                    frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                }
                if (backLeft != null) {
                    backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                }
                if (backRight != null) {
                    backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                }
                // Reset stats
                flRpmSum = frRpmSum = blRpmSum = brRpmSum = 0;
                flSamples = frSamples = blSamples = brSamples = 0;
                runtime.reset();
            }
            prevRB = gamepad1.right_bumper;

            // Start: reset average RPM stats only
            if (gamepad1.start && !prevStart) {
                flRpmSum = frRpmSum = blRpmSum = brRpmSum = 0;
                flSamples = frSamples = blSamples = brSamples = 0;
            }
            prevStart = gamepad1.start;

            // === SET MOTOR POWERS ===
            if (frontLeft != null) frontLeft.setPower(flPower);
            if (frontRight != null) frontRight.setPower(frPower);
            if (backLeft != null) backLeft.setPower(blPower);
            if (backRight != null) backRight.setPower(brPower);

            // === GET RPM DATA ===
            double flRpm = 0, frRpm = 0, blRpm = 0, brRpm = 0;
            double flVel = 0, frVel = 0, blVel = 0, brVel = 0;
            int flPos = 0, frPos = 0, blPos = 0, brPos = 0;

            if (frontLeft != null) {
                flVel = frontLeft.getVelocity(AngleUnit.DEGREES); // deg/sec
                flRpm = (flVel / 360.0) * 60.0; // convert to RPM
                flPos = frontLeft.getCurrentPosition();
                if (flPower > 0.1 && Math.abs(flRpm) > 5) { // Only track when motor is running
                    flRpmSum += Math.abs(flRpm);
                    flSamples++;
                }
            }

            if (frontRight != null) {
                frVel = frontRight.getVelocity(AngleUnit.DEGREES);
                frRpm = (frVel / 360.0) * 60.0;
                frPos = frontRight.getCurrentPosition();
                if (frPower > 0.1 && Math.abs(frRpm) > 5) {
                    frRpmSum += Math.abs(frRpm);
                    frSamples++;
                }
            }

            if (backLeft != null) {
                blVel = backLeft.getVelocity(AngleUnit.DEGREES);
                blRpm = (blVel / 360.0) * 60.0;
                blPos = backLeft.getCurrentPosition();
                if (blPower > 0.1 && Math.abs(blRpm) > 5) {
                    blRpmSum += Math.abs(blRpm);
                    blSamples++;
                }
            }

            if (backRight != null) {
                brVel = backRight.getVelocity(AngleUnit.DEGREES);
                brRpm = (brVel / 360.0) * 60.0;
                brPos = backRight.getCurrentPosition();
                if (brPower > 0.1 && Math.abs(brRpm) > 5) {
                    brRpmSum += Math.abs(brRpm);
                    brSamples++;
                }
            }

            // === TELEMETRY ===
            telemetry.clear();
            telemetry.addLine("=== ENHANCED MOTOR TEST ===");
            telemetry.addData("Runtime", String.format("%.1f sec", runtime.seconds()));
            telemetry.addLine();

            // Motor selection status
            telemetry.addLine("--- SELECTED MOTORS (A/B/X/Y) ---");
            telemetry.addData("Front Left (A)", flSelected ? "SELECTED" : "-");
            telemetry.addData("Front Right (B)", frSelected ? "SELECTED" : "-");
            telemetry.addData("Back Left (X)", blSelected ? "SELECTED" : "-");
            telemetry.addData("Back Right (Y)", brSelected ? "SELECTED" : "-");
            telemetry.addLine();

            // Power and RPM data
            telemetry.addLine("--- MOTOR DATA ---");
            telemetry.addData("FL Power", String.format("%.2f (%.0f%%)", flPower, flPower * 100));
            telemetry.addData("FL RPM", String.format("%.1f RPM", Math.abs(flRpm)));
            if (flSamples > 0) {
                telemetry.addData("FL Avg RPM", String.format("%.1f RPM (%d samples)", flRpmSum / flSamples, flSamples));
            }
            if (showEncoders) telemetry.addData("FL Position", flPos);
            telemetry.addLine();

            telemetry.addData("FR Power", String.format("%.2f (%.0f%%)", frPower, frPower * 100));
            telemetry.addData("FR RPM", String.format("%.1f RPM", Math.abs(frRpm)));
            if (frSamples > 0) {
                telemetry.addData("FR Avg RPM", String.format("%.1f RPM (%d samples)", frRpmSum / frSamples, frSamples));
            }
            if (showEncoders) telemetry.addData("FR Position", frPos);
            telemetry.addLine();

            telemetry.addData("BL Power", String.format("%.2f (%.0f%%)", blPower, blPower * 100));
            telemetry.addData("BL RPM", String.format("%.1f RPM", Math.abs(blRpm)));
            if (blSamples > 0) {
                telemetry.addData("BL Avg RPM", String.format("%.1f RPM (%d samples)", blRpmSum / blSamples, blSamples));
            }
            if (showEncoders) telemetry.addData("BL Position", blPos);
            telemetry.addLine();

            telemetry.addData("BR Power", String.format("%.2f (%.0f%%)", brPower, brPower * 100));
            telemetry.addData("BR RPM", String.format("%.1f RPM", Math.abs(brRpm)));
            if (brSamples > 0) {
                telemetry.addData("BR Avg RPM", String.format("%.1f RPM (%d samples)", brRpmSum / brSamples, brSamples));
            }
            if (showEncoders) telemetry.addData("BR Position", brPos);
            telemetry.addLine();

            // Power correction recommendations
            telemetry.addLine("--- POWER CORRECTION GUIDE ---");
            telemetry.addLine("Run all motors at same power (e.g., 0.50)");
            telemetry.addLine("Compare Average RPM values:");
            telemetry.addLine("  If motor RPM is LOW → increase power factor");
            telemetry.addLine("  If motor RPM is HIGH → decrease power factor");
            telemetry.addLine();

            // Controls reminder
            telemetry.addLine("--- CONTROLS ---");
            telemetry.addData("D-Pad Up/Down", "+/- 0.05 power");
            telemetry.addData("D-Pad L/R", "Stop / Full power");
            telemetry.addData("Triggers", "Fine adjust +/- 0.01");
            telemetry.addData("Left Bumper", showEncoders ? "Hide encoders" : "Show encoders");
            telemetry.addData("Right Bumper", "Reset all");
            telemetry.addData("Start", "Reset avg stats");

            telemetry.update();
        }

        // Stop all motors
        if (frontLeft != null) frontLeft.setPower(0);
        if (frontRight != null) frontRight.setPower(0);
        if (backLeft != null) backLeft.setPower(0);
        if (backRight != null) backRight.setPower(0);
    }
}

