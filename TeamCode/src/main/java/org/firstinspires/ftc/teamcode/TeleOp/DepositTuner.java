package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.common.RobotHardware;
import org.firstinspires.ftc.teamcode.common.Odometry;
import org.firstinspires.ftc.teamcode.common.PanelsPublisher;
import org.firstinspires.ftc.teamcode.common.PIDController;

import java.util.Locale;
//I need to push ignore this
@TeleOp(name = "Deposit Tuner", group = "TeleOp")
public class DepositTuner extends LinearOpMode {
    @Override
    public void runOpMode() {
        RobotHardware hw = new RobotHardware(hardwareMap);
        hw.initPinpoint();
        PanelsPublisher panels = new PanelsPublisher();
        panels.init();

        telemetry.addLine("Init complete - waiting start");
        telemetry.addLine("Use gamepad1 dpad to adjust preset X (used when holding X)");
        telemetry.addLine("Use gamepad2 dpad to adjust preset Y (used when holding Y)");
        telemetry.addLine("Hold gamepadX/BACK while using D-pad to adjust Preset B");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        double lastLoopTime = getRuntime();

        // Presets (velocity in encoder ticks per second)
        double presetX = 850.0; // default used when gamepad2.x is pressed
        double presetY = 850.0; // default used when gamepad2.y is pressed
        double presetB = 850.0; // third preset used when gamepad2.b is pressed

        final double MIN_V = 0.0;
        final double MAX_V = 5000.0;
        final double STEP_SMALL = 10.0;
        final double STEP_LARGE = 100.0;

        // For D-pad edge detection and hold-repeat
        boolean prevGp1DpadUp = false, prevGp1DpadDown = false, prevGp1DpadLeft = false, prevGp1DpadRight = false;
        boolean prevGp2DpadUp = false, prevGp2DpadDown = false, prevGp2DpadLeft = false, prevGp2DpadRight = false;
        long gp1LastChange = System.currentTimeMillis();
        long gp2LastChange = System.currentTimeMillis();
        final long FIRST_REPEAT_DELAY_MS = 350;
        final long REPEAT_INTERVAL_MS = 120;

        // Odometry
        Odometry odometry = new Odometry(hw, hw.pinpoint);

        // Set initial servo positions - cam
        if (hw.cam != null) hw.cam.setPosition(0.48);

        boolean prevComboReset = false;
        boolean prevComboRecalibrate = false;

        // Toggle state for deposit buttons (X, Y, B)
        boolean depositToggleX = false;
        boolean depositToggleY = false;
        boolean depositToggleB = false;
        boolean prevGp2X = false;
        boolean prevGp2Y = false;
        boolean prevGp2B = false;

        // PID controller for deposit motors (ticks/sec error -> power output)
        PIDController depositPid = new PIDController(0.0008, 0.0000015, 0.00005);
        depositPid.setOutputLimits(-1.0, 1.0);
        depositPid.setIntegratorLimits(-2000, 2000);
        double prevTarget = 0.0;

        // Live-tuning parameters (mirrors the PID internals and feedforward)
        double kP = 0.0008;
        double kI = 0.0000015;
        double kD = 0.00005;
        double kFF = 0.00015; // linear feedforward coefficient (power per ticks/sec)

        // PID tuning UI state
        int pidSelect = 0; // 0=kP,1=kI,2=kD,3=kFF
        String[] pidNames = new String[] {"kP","kI","kD","kFF"};
        boolean prevGp1A = false; // for cycling selection

        // Steps for tuning (small/large)
        double[] smallStep = new double[] {0.00005, 0.0000005, 0.00001, 0.00001};
        double[] largeStep = new double[] {0.0005, 0.000005, 0.00005, 0.00005};

        while (opModeIsActive()) {
            double nowTime = getRuntime();
            double dt = Math.max(0.0, Math.min(0.1, nowTime - lastLoopTime));
            lastLoopTime = nowTime;

            hw.updatePinpoint();
            odometry.update();
            Odometry.Position pos = odometry.getPosition();

            // ===== DRIVER 1: CHASSIS CONTROL =====
            double forward = Math.abs(gamepad1.left_stick_y) > 0.05 ? -gamepad1.left_stick_y : 0;
            double strafe = Math.abs(gamepad1.left_stick_x) > 0.05 ? gamepad1.left_stick_x : 0;
            double rotate = Math.abs(gamepad1.right_stick_x) > 0.05 ? gamepad1.right_stick_x : 0;

            rotate += -0.15 * forward;

            double speedMul = gamepad1.left_bumper ? 0.5 : 1.0;
            forward *= speedMul;
            strafe *= speedMul;
            rotate *= speedMul;
            strafe *= 1.1; // Strafe compensation


            // Mecanum drive formulas with proper normalization
            double fl = forward + strafe + rotate;
            double fr = forward - strafe - rotate;
            double bl = forward - strafe + rotate;
            double br = forward + strafe - rotate;

            // Normalize to prevent exceeding max power
            double maxPower = Math.max(1.0, Math.max(Math.abs(fl), Math.max(Math.abs(fr), Math.max(Math.abs(bl), Math.abs(br)))));
            fl /= maxPower;
            fr /= maxPower;
            bl /= maxPower;
            br /= maxPower;

            // Reduce rotation component after normalization to maintain max speed movement
            double rotationScale = 0.65; // 65% rotation speed (increased from 50%)
            double rotationComponent = rotate / maxPower;
            fl -= rotationComponent * (1.0 - rotationScale);
            fr += rotationComponent * (1.0 - rotationScale);
            bl -= rotationComponent * (1.0 - rotationScale);
            br += rotationComponent * (1.0 - rotationScale);

            hw.setDrivePowers(fl, fr, bl, br);

            // Driver 1: Safe combos (pinpoint reset / recalibrate)
            boolean comboReset = gamepad1.start && gamepad1.back;
            if (comboReset && !prevComboReset) {
                if (hw.pinpoint != null) hw.pinpoint.resetPosAndIMU();
                else if (hw.imu != null) hw.imu.resetYaw();
            }
            prevComboReset = comboReset;

            boolean comboRecalibrate = gamepad1.x && gamepad1.y;
            if (comboRecalibrate && !prevComboRecalibrate && hw.pinpoint != null) hw.pinpoint.recalibrateIMU();
            prevComboRecalibrate = comboRecalibrate;

            // Intake (triggers, both => stop)
            boolean rt = gamepad2.right_trigger > 0.1;
            boolean lt = gamepad2.left_trigger > 0.1;
            double intakePower = (rt && lt) ? 0.0 : rt ? 0.9 : lt ? -0.9 : 0.0;
            if (hw.intakeMotor != null) hw.intakeMotor.setPower(intakePower);

            // ===== DPAD TUNING =====
            // Read dpad states
            boolean gp1Up = gamepad1.dpad_up;
            boolean gp1Down = gamepad1.dpad_down;
            boolean gp1Left = gamepad1.dpad_left;
            boolean gp1Right = gamepad1.dpad_right;

            boolean gp2Up = gamepad2.dpad_up;
            boolean gp2Down = gamepad2.dpad_down;
            boolean gp2Left = gamepad2.dpad_left;
            boolean gp2Right = gamepad2.dpad_right;

            long now = System.currentTimeMillis();

            // Determine which preset each controller is adjusting. Holding BACK makes the controller adjust presetB instead.
            boolean gp1AdjustB = gamepad1.back;
            boolean gp2AdjustB = gamepad2.back;

            // Cycle PID selection when gamepad1.A is pressed (edge detect)
            if (gamepad1.a && !prevGp1A) {
                pidSelect = (pidSelect + 1) % pidNames.length;
            }
            prevGp1A = gamepad1.a;

            // If driver holds right bumper, use gp1 D-pad to tune PID coefficients instead of presets
            boolean gp1TunePid = gamepad1.right_bumper;

            // Gamepad1 controls presetX or presetB, but if right_bumper held, it tunes PID params
            if (gp1TunePid) {
                // adjust selected PID coefficient via dpad
                if (gp1Up && (!prevGp1DpadUp || now - gp1LastChange > FIRST_REPEAT_DELAY_MS)) {
                    if (pidSelect == 0) kP += smallStep[0];
                    if (pidSelect == 1) kI += smallStep[1];
                    if (pidSelect == 2) kD += smallStep[2];
                    if (pidSelect == 3) kFF += smallStep[3];
                    gp1LastChange = now;
                }
                if (gp1Down && (!prevGp1DpadDown || now - gp1LastChange > FIRST_REPEAT_DELAY_MS)) {
                    if (pidSelect == 0) kP = Math.max(0.0, kP - smallStep[0]);
                    if (pidSelect == 1) kI = Math.max(0.0, kI - smallStep[1]);
                    if (pidSelect == 2) kD = Math.max(0.0, kD - smallStep[2]);
                    if (pidSelect == 3) kFF = Math.max(0.0, kFF - smallStep[3]);
                    gp1LastChange = now;
                }
                if (gp1Right && (!prevGp1DpadRight || now - gp1LastChange > FIRST_REPEAT_DELAY_MS)) {
                    if (pidSelect == 0) kP += largeStep[0];
                    if (pidSelect == 1) kI += largeStep[1];
                    if (pidSelect == 2) kD += largeStep[2];
                    if (pidSelect == 3) kFF += largeStep[3];
                    gp1LastChange = now;
                }
                if (gp1Left && (!prevGp1DpadLeft || now - gp1LastChange > FIRST_REPEAT_DELAY_MS)) {
                    if (pidSelect == 0) kP = Math.max(0.0, kP - largeStep[0]);
                    if (pidSelect == 1) kI = Math.max(0.0, kI - largeStep[1]);
                    if (pidSelect == 2) kD = Math.max(0.0, kD - largeStep[2]);
                    if (pidSelect == 3) kFF = Math.max(0.0, kFF - largeStep[3]);
                    gp1LastChange = now;
                }
                // whenever we change coefficients, push them into the PID
                depositPid.setCoefficients(kP, kI, kD);
            } else {
                // existing behavior: adjust presets using gp1 dpad
                if (!gp1AdjustB) {
                    if (gp1Up && (!prevGp1DpadUp || now - gp1LastChange > FIRST_REPEAT_DELAY_MS)) {
                        presetX = Math.min(MAX_V, presetX + STEP_LARGE);
                        gp1LastChange = now;
                    }
                    if (gp1Down && (!prevGp1DpadDown || now - gp1LastChange > FIRST_REPEAT_DELAY_MS)) {
                        presetX = Math.max(MIN_V, presetX - STEP_LARGE);
                        gp1LastChange = now;
                    }
                    if (gp1Right && (!prevGp1DpadRight || now - gp1LastChange > FIRST_REPEAT_DELAY_MS)) {
                        presetX = Math.min(MAX_V, presetX + STEP_SMALL);
                        gp1LastChange = now;
                    }
                    if (gp1Left && (!prevGp1DpadLeft || now - gp1LastChange > FIRST_REPEAT_DELAY_MS)) {
                        presetX = Math.max(MIN_V, presetX - STEP_SMALL);
                        gp1LastChange = now;
                    }
                } else {
                    if (gp1Up && (!prevGp1DpadUp || now - gp1LastChange > FIRST_REPEAT_DELAY_MS)) {
                        presetB = Math.min(MAX_V, presetB + STEP_LARGE);
                        gp1LastChange = now;
                    }
                    if (gp1Down && (!prevGp1DpadDown || now - gp1LastChange > FIRST_REPEAT_DELAY_MS)) {
                        presetB = Math.max(MIN_V, presetB - STEP_LARGE);
                        gp1LastChange = now;
                    }
                    if (gp1Right && (!prevGp1DpadRight || now - gp1LastChange > FIRST_REPEAT_DELAY_MS)) {
                        presetB = Math.min(MAX_V, presetB + STEP_SMALL);
                        gp1LastChange = now;
                    }
                    if (gp1Left && (!prevGp1DpadLeft || now - gp1LastChange > FIRST_REPEAT_DELAY_MS)) {
                        presetB = Math.max(MIN_V, presetB - STEP_SMALL);
                        gp1LastChange = now;
                    }
                }
            }

            // while holding, use faster repeat interval for gamepad1
            if ((gp1Up || gp1Down || gp1Left || gp1Right) && now - gp1LastChange > REPEAT_INTERVAL_MS) {
                gp1LastChange = now - (REPEAT_INTERVAL_MS + 1);
            }

            // Gamepad2 controls presetY or presetB
            if (!gp2AdjustB) {
                if (gp2Up && (!prevGp2DpadUp || now - gp2LastChange > FIRST_REPEAT_DELAY_MS)) {
                    presetY = Math.min(MAX_V, presetY + STEP_LARGE);
                    gp2LastChange = now;
                }
                if (gp2Down && (!prevGp2DpadDown || now - gp2LastChange > FIRST_REPEAT_DELAY_MS)) {
                    presetY = Math.max(MIN_V, presetY - STEP_LARGE);
                    gp2LastChange = now;
                }
                if (gp2Right && (!prevGp2DpadRight || now - gp2LastChange > FIRST_REPEAT_DELAY_MS)) {
                    presetY = Math.min(MAX_V, presetY + STEP_SMALL);
                    gp2LastChange = now;
                }
                if (gp2Left && (!prevGp2DpadLeft || now - gp2LastChange > FIRST_REPEAT_DELAY_MS)) {
                    presetY = Math.max(MIN_V, presetY - STEP_SMALL);
                    gp2LastChange = now;
                }
            } else {
                if (gp2Up && (!prevGp2DpadUp || now - gp2LastChange > FIRST_REPEAT_DELAY_MS)) {
                    presetB = Math.min(MAX_V, presetB + STEP_LARGE);
                    gp2LastChange = now;
                }
                if (gp2Down && (!prevGp2DpadDown || now - gp2LastChange > FIRST_REPEAT_DELAY_MS)) {
                    presetB = Math.max(MIN_V, presetB - STEP_LARGE);
                    gp2LastChange = now;
                }
                if (gp2Right && (!prevGp2DpadRight || now - gp2LastChange > FIRST_REPEAT_DELAY_MS)) {
                    presetB = Math.min(MAX_V, presetB + STEP_SMALL);
                    gp2LastChange = now;
                }
                if (gp2Left && (!prevGp2DpadLeft || now - gp2LastChange > FIRST_REPEAT_DELAY_MS)) {
                    presetB = Math.max(MIN_V, presetB - STEP_SMALL);
                    gp2LastChange = now;
                }
            }

            if ((gp2Up || gp2Down || gp2Left || gp2Right) && now - gp2LastChange > REPEAT_INTERVAL_MS) {
                gp2LastChange = now - (REPEAT_INTERVAL_MS + 1);
            }

            // Apply deposit velocities based on gamepad2 buttons: X -> presetX, Y -> presetY, B -> presetB
            // Toggle-based control: press button to toggle on/off
            if (gamepad2.x && !prevGp2X) {
                depositToggleX = !depositToggleX;
                depositToggleY = false;
                depositToggleB = false;
            }
            if (gamepad2.y && !prevGp2Y) {
                depositToggleY = !depositToggleY;
                depositToggleX = false;
                depositToggleB = false;
            }
            if (gamepad2.b && !prevGp2B) {
                depositToggleB = !depositToggleB;
                depositToggleX = false;
                depositToggleY = false;
            }
            prevGp2X = gamepad2.x;
            prevGp2Y = gamepad2.y;
            prevGp2B = gamepad2.b;

            double target = 0.0;
            if (depositToggleX) target = presetX;
            else if (depositToggleY) target = presetY;
            else if (depositToggleB) target = presetB;

            // Read actual velocity from motors (ticks/sec). If motor references are null, treat as 0.
            double actual = 0.0;
            if (hw.depositMotorL != null) actual = hw.depositMotorL.getVelocity();

            double power;

            // Reset PID when target changes significantly or target is zero
            if (Math.abs(prevTarget - target) > 1.0) {
                depositPid.reset();
            }
            prevTarget = target;

            if (Math.abs(target) < 1.0) {
                // Stop motors and reset integrator to avoid windup
                power = 0.0;
                depositPid.reset();
            } else {
                // PID expects setpoint in same units as measurement (ticks/sec)
                power = depositPid.update(target, actual, dt);
                // feedforward proportional to target
                double ff = kFF * target;
                power += ff;
                // clamp power
                if (power > 1.0) power = 1.0;
                if (power < -1.0) power = -1.0;
            }

            if (hw.depositMotorL != null) hw.depositMotorL.setPower(power);
            if (hw.depositMotorR != null) hw.depositMotorR.setPower(-power);

            // Cam (LB/RB) continuous hold-based control - only update when buttons pressed
            if (hw.cam != null) {
                if (gamepad2.left_bumper || gamepad2.right_bumper) {
                    double camPosition = hw.cam.getPosition();
                    if (gamepad2.left_bumper) camPosition -= 0.001; // Move in
                    if (gamepad2.right_bumper) camPosition += 0.001; // Move out
                    // Clamp between reasonable bounds
                    camPosition = Math.max(0.4622, Math.min(0.5522, camPosition));
                    hw.cam.setPosition(camPosition);
                }
            }

            // Telemetry
            telemetry.addData("Preset X (gp1 dpad)", String.format(Locale.US, "%.0f ticks/s", presetX));
            telemetry.addData("Preset Y (gp2 dpad)", String.format(Locale.US, "%.0f ticks/s", presetY));
            telemetry.addData("Preset B (hold BACK + dpad)", String.format(Locale.US, "%.0f ticks/s", presetB));
            telemetry.addData("Active target", String.format(Locale.US, "%.0f", target));

            // PID tuning telemetry
            telemetry.addData("PID Select", pidNames[pidSelect]);
            telemetry.addData("kP/kI/kD/kFF", String.format(Locale.US, "%.6f / %.8f / %.6f / %.6f", kP, kI, kD, kFF));
            telemetry.addData("PID integrator/err/der", String.format(Locale.US, "I: %.1f E: %.1f D: %.1f", depositPid.getIntegrator(), depositPid.getLastError(), depositPid.getLastDerivative()));
            telemetry.addData("PID out+ff/power", String.format(Locale.US, "ff: %.4f pwr: %.3f", kFF*target, power));

            if (hw.depositMotorL != null && hw.depositMotorR != null) {
                double ticksPerRev = 28.0;
                double vL = hw.depositMotorL.getVelocity();
                double vR = hw.depositMotorR.getVelocity();
                double rpmL = (vL / ticksPerRev) * 60.0;
                double rpmR = (vR / ticksPerRev) * 60.0;
                telemetry.addData("Launcher RPM", String.format(Locale.US, "L: %.0f, R: %.0f", rpmL, rpmR));
            }

            telemetry.addData("Controls", "GP1 Dpad -> Preset X (BACK->B). GP2 Dpad -> Preset Y (BACK->B)");
            telemetry.addData("Use", "Hold GP2.X or GP2.Y or GP2.B to run deposit at preset");

            // Additional telemetry from Robot1
            telemetry.addData("Speed Mode", gamepad1.left_bumper ? "SLOW (50%)" : "Normal");
            telemetry.addData("Heading", String.format(Locale.US, "%.1f°", pos.getHeadingDeg()));
            telemetry.addData("Position", String.format(Locale.US, "X: %.1f, Y: %.1f in", pos.getXmm()/25.4, pos.getYmm()/25.4));
            telemetry.addData("Cam Position", hw.cam != null ? String.format(Locale.US, "%.4f", hw.cam.getPosition()) : "N/A");
            telemetry.addData("Intake Power", String.format(Locale.US, "%.1f", intakePower));

            telemetry.addData("Loop Hz", String.format(Locale.US, "%.1f", 1.0 / Math.max(1e-6, dt)));
            telemetry.update();

            // Panels publishing (inches)
            double xIn = pos.getXmm()/25.4;
            double yIn = pos.getYmm()/25.4;
            panels.putText("teleop/pinpoint/status", hw.pinpoint!=null? hw.pinpoint.getDeviceStatus().name():"NONE");
            panels.publishPose("teleop/pose", xIn, yIn, pos.getHeadingDeg(), 50);

            // update previous dpad states
            prevGp1DpadUp = gp1Up; prevGp1DpadDown = gp1Down; prevGp1DpadLeft = gp1Left; prevGp1DpadRight = gp1Right;
            prevGp2DpadUp = gp2Up; prevGp2DpadDown = gp2Down; prevGp2DpadLeft = gp2Left; prevGp2DpadRight = gp2Right;

            // small sleep to yield
            sleep(20);
        }
    }
}
