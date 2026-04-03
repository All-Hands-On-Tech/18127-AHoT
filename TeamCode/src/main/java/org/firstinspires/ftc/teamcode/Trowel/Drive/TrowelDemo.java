package org.firstinspires.ftc.teamcode.Trowel.Drive;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Trowel.Configs.TrowelConfigs;
import org.firstinspires.ftc.teamcode.Trowel.Configs.TrowelHardware;
import org.firstinspires.ftc.teamcode.Trowel.common.Odometry;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;

@Configurable
@TeleOp(name = "Trowel Demo", group = "Trowel")
public class TrowelDemo extends OpMode {

    // ══════════════════════════════════════════════════════════════════════════
    // SINGLE DRIVER TELEOP
    // Simplified controls: one gamepad for drive, flywheel, intake
    // Limited speed and power for controlled operation
    // ══════════════════════════════════════════════════════════════════════════

    // ══════════════════════════════════════════════════════════════════════════
    // ABSOLUTE FIELD COORDINATE SYSTEM  (used only for odometry display)
    // (0,0) = bottom-left corner of field when viewed from above
    // X increases to the right (toward red alliance side)
    // Y increases upward (toward back wall from driver station)
    // ══════════════════════════════════════════════════════════════════════════

    public enum Alliance { NONE, RED, BLUE }

    private Alliance selectedAlliance = Alliance.NONE;


    private TrowelHardware robot;
    private Odometry       odometry;
    private final ElapsedTime loopTimer = new ElapsedTime();

    private boolean odoEnabled   = false;
    private boolean flywheelOn   = false;
    private boolean inRecoveryMode = false;
    private double  flywheelTarget;

    private boolean prevFlywheelToggle = false;
    private boolean prevDpadUp         = false;
    private boolean prevDpadDown       = false;

    private double lastLoopTime = 0.0;
    private double lastP        = -1;

    private double vel1 = 0, vel2 = 0, avgVel = 0, commandedTarget = 0;
    private double flPower = 0, frPower = 0, blPower = 0, brPower = 0;
    private double intake1Power = 0, intake2Power = 0;
    private double loopTimeMs   = 0;
    private boolean isSlowMode  = false;

    // Brownout
    private VoltageSensor voltageSensor;
    private double[]      voltageSamples;
    private int           voltageSampleIndex = 0;
    private double        smoothedVoltage    = 12.0;
    private boolean       inBrownout         = false;

    // General AprilTag state (independent of auto-aim)
    private boolean tagDetected       = false;
    private int     detectedTagId     = -1;
    private double  detectedTagRange  = 0.0;
    private double  detectedTagBearing = 0.0;
    private int     totalTagsVisible  = 0;

    // Odometry camera-correction state
    private double lastCameraPoseX       = 0;
    private double lastCameraPoseY       = 0;
    private double lastCameraPoseHeading = 0;

    // Initial pose state
    private boolean initialPoseEstablished = false;
    private int     initialPoseAttempts    = 0;
    private static final int MAX_INITIAL_POSE_ATTEMPTS = 100;

    // ══════════════════════════════════════════════════════════════
    // INITIALIZATION
    // ══════════════════════════════════════════════════════════════

    @Override
    public void init() {
        robot = new TrowelHardware(hardwareMap);
        loopTimer.reset();

        initVoltageSensor();
        initDriveMotors();
        initFlywheelMotors(TrowelConfigs.PIDF_P);
        initOdometry();


        initialPoseEstablished = false;
        initialPoseAttempts    = 0;

        telemetry.update();


        showControls();
    }

    @Override
    public void init_loop() {
        if (gamepad1.x || gamepad2.x) selectedAlliance = Alliance.BLUE;
        else if (gamepad1.b || gamepad2.b) selectedAlliance = Alliance.RED;

        if (!initialPoseEstablished && robot.isVisionEnabled()) {
            attemptInitialPoseFromCamera();
        }

        telemetry.addLine("══════════════════════════════");
        telemetry.addLine("     ALLIANCE SELECTION");
        telemetry.addLine("══════════════════════════════");
        telemetry.addLine("");
        String allianceStr;
        switch (selectedAlliance) {
            case RED:   allianceStr = "🔴 RED ALLIANCE"; break;
            case BLUE:  allianceStr = "🔵 BLUE ALLIANCE"; break;
            default:    allianceStr = "⬜ NOT SELECTED"; break;
        }
        telemetry.addData("Current Selection", allianceStr);
        telemetry.addLine("");
        telemetry.addLine("Press X for BLUE  |  Press B for RED");
        telemetry.addLine("");

        if (initialPoseEstablished) {
            telemetry.addData("Pose Status", "✓ Initial pose established from camera");
            if (odometry != null) {
                Odometry.Position p = odometry.getPosition();
                telemetry.addData("Odometry Reports", "X: %.1f in, Y: %.1f in, H: %.1f°",
                        p.xMm / 25.4, p.yMm / 25.4, Math.toDegrees(p.headingRad));
            }
            if (lastCameraPoseX != 0 || lastCameraPoseY != 0) {
                telemetry.addData("Camera Calculated", "X: %.1f in, Y: %.1f in, H: %.1f°",
                        lastCameraPoseX, lastCameraPoseY, lastCameraPoseHeading);
            }
        } else {
            telemetry.addData("Pose Status", "⏳ Waiting for AprilTag... (%d attempts)", initialPoseAttempts);
            telemetry.addLine("Point camera at AprilTag to establish position");
        }

        if (robot.isVisionEnabled()) {
            List<AprilTagDetection> detections = robot.getDetections();
            telemetry.addData("AprilTags Visible", detections.size());
            for (AprilTagDetection det : detections) {
                if (det.metadata != null) {
                    telemetry.addData("  Tag " + det.id, "%.1f\" away, bearing %.1f°",
                            det.ftcPose.range, det.ftcPose.bearing);
                } else {
                    telemetry.addData("  Tag " + det.id, "(unknown size — no metadata)");
                }
            }
        }
        telemetry.update();
    }

    /**
     * Establishes the robot's initial pose using AprilTag camera data.
     * Uses ABSOLUTE FIELD COORDINATES where (0,0) is bottom-left of field.
     */
    private void attemptInitialPoseFromCamera() {
        initialPoseAttempts++;

        List<AprilTagDetection> detections = robot.getDetections();
        for (AprilTagDetection det : detections) {
            if (det.robotPose != null && det.ftcPose != null) {
                try {
                    org.firstinspires.ftc.robotcore.external.navigation.Position robotPos =
                            det.robotPose.getPosition();
                    org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles robotYpr =
                            det.robotPose.getOrientation();

                    double xInches    = robotPos.toUnit(DistanceUnit.INCH).x;
                    double yInches    = robotPos.toUnit(DistanceUnit.INCH).y;
                    double headingDeg = robotYpr.getYaw(AngleUnit.DEGREES);

                    if (xInches < 0 || xInches > 144 || yInches < 0 || yInches > 144) {
                        double[] recalc = recalcPoseFromTag(det, headingDeg);
                        if (recalc == null) continue;
                        xInches = recalc[0];
                        yInches = recalc[1];
                    }

                    if (xInches < -10 || xInches > 154 || yInches < -10 || yInches > 154) continue;

                    if (robot.pinpoint != null) {
                        robot.pinpoint.setPosition(new Pose2D(
                                DistanceUnit.INCH, xInches, yInches, AngleUnit.DEGREES, headingDeg));
                        if (odometry != null) odometry.update();

                        initialPoseEstablished = true;
                        lastCameraPoseX       = xInches;
                        lastCameraPoseY       = yInches;
                        lastCameraPoseHeading = headingDeg;
                    }
                    break;
                } catch (Exception ignored) {}
            }
        }

        // Fallback after max attempts
        if (!initialPoseEstablished && initialPoseAttempts > MAX_INITIAL_POSE_ATTEMPTS) {
            if (robot.pinpoint != null && selectedAlliance != Alliance.NONE) {
                double dx, dy, dh;
                if (selectedAlliance == Alliance.BLUE) { dx = 24; dy = 120; dh = 90; }
                else                                   { dx = 120; dy = 120; dh = 90; }
                robot.pinpoint.setPosition(new Pose2D(
                        DistanceUnit.INCH, dx, dy, AngleUnit.DEGREES, dh));
                if (odometry != null) { robot.updatePinpoint(); odometry.update(); }
            }
            initialPoseEstablished = true;
        }
    }

    /**
     * Recalculates robot position from known AprilTag field position and
     * camera range/bearing when the SDK-reported robotPose is out of bounds.
     *
     * @return {x, y} in inches, or null if tag field position is unknown.
     */
    private double[] recalcPoseFromTag(AprilTagDetection det, double headingDeg) {
        double tagFieldX, tagFieldY;

        if (det.id >= TrowelConfigs.BLUE_TAG_ID_MIN && det.id <= TrowelConfigs.BLUE_TAG_ID_MAX) {
            tagFieldX = TrowelConfigs.BLUE_TAG_X;  tagFieldY = TrowelConfigs.BLUE_TAG_Y;
        } else if (det.id >= TrowelConfigs.RED_TAG_ID_MIN && det.id <= TrowelConfigs.RED_TAG_ID_MAX) {
            tagFieldX = TrowelConfigs.RED_TAG_X;   tagFieldY = TrowelConfigs.RED_TAG_Y;
        } else if (det.metadata != null && det.metadata.fieldPosition != null) {
            tagFieldX = det.metadata.fieldPosition.get(0) / 25.4;
            tagFieldY = det.metadata.fieldPosition.get(1) / 25.4;
        } else {
            return null;
        }

        double range           = det.ftcPose.range;
        double angleRobotToTag = Math.toRadians(headingDeg) + Math.toRadians(det.ftcPose.bearing);

        double x = tagFieldX - range * Math.cos(angleRobotToTag);
        double y = tagFieldY - range * Math.sin(angleRobotToTag);

        if (x < -10 || x > 154 || y < -10 || y > 154) return null;
        return new double[]{ x, y };
    }

    private void initVoltageSensor() {
        voltageSensor = null;
        double maxVoltage = 0;
        for (VoltageSensor sensor : hardwareMap.voltageSensor) {
            double v = sensor.getVoltage();
            if (v > maxVoltage) { maxVoltage = v; voltageSensor = sensor; }
        }
        voltageSamples = new double[TrowelConfigs.VOLTAGE_SAMPLE_COUNT];
        for (int i = 0; i < TrowelConfigs.VOLTAGE_SAMPLE_COUNT; i++)
            voltageSamples[i] = maxVoltage > 0 ? maxVoltage : 12.0;
        smoothedVoltage = maxVoltage > 0 ? maxVoltage : 12.0;
    }

    private void showControls() {
        telemetry.addLine("══════════════════════════════");
        telemetry.addLine("    RANDY BUTTER KNUBS");
        telemetry.addLine("══════════════════════════════");
        telemetry.addLine("");
        telemetry.addLine("INIT — ALLIANCE SELECT:");
        telemetry.addLine("  X: Blue  |  B: Red");
        telemetry.addLine("");
        telemetry.addLine("SINGLE DRIVER (Gamepad 1):");
        telemetry.addLine("  Left Stick: Move");
        telemetry.addLine("  Right Stick X: Turn");
        telemetry.addLine("  RB hold: Slow mode (50%)");
        telemetry.addLine("  X: Toggle flywheel");
        telemetry.addLine("  Dpad Up/Down: Adjust flywheel speed");
        telemetry.addLine("  LT: Intake in  |  RT: Intake reverse");
        telemetry.addLine("  A: Intake2 in  |  B: Intake2 reverse");
        telemetry.addLine("  LB: Shoot");
        telemetry.addLine("");
        telemetry.addData("Battery", "%.2fV", smoothedVoltage);
        telemetry.addData("Vision", robot.getVisionStatusString());
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
                    motor.setVelocityPIDFCoefficients(pValue, TrowelConfigs.PIDF_I, TrowelConfigs.PIDF_D, TrowelConfigs.PIDF_F);
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
        lastLoopTime   = 0;
        flywheelTarget = TrowelConfigs.FLYWHEEL_DEFAULT_TARGET;
    }

    @Override
    public void stop() {
        for (DcMotor m : new DcMotor[]{robot.frontLeft, robot.frontRight, robot.backLeft, robot.backRight})
            if (m != null) m.setPower(0);
        if (robot.deposit1 != null) robot.deposit1.setPower(0);
        if (robot.deposit2 != null) robot.deposit2.setPower(0);
        robot.stop();
        robot.closeVision();
    }

    // ══════════════════════════════════════════════════════════════
    // MAIN LOOP
    // ══════════════════════════════════════════════════════════════

    @Override
    public void loop() {
        double currentTime = loopTimer.seconds();
        double deltaTime   = Math.min(currentTime - lastLoopTime, 0.1);
        loopTimeMs         = deltaTime * 1000.0;
        lastLoopTime       = currentTime;

        updateVoltage();

        if (odoEnabled && robot.pinpoint != null) {
            robot.updatePinpoint();
            odometry.update();
        }

        updateAprilTagDetection();

        if (!initialPoseEstablished && robot.isVisionEnabled()) {
            attemptInitialPoseFromCamera();
        }

        // Continuously correct odometry with camera (separate from aiming)
        continuouslyUpdateOdometryFromCamera();

        updateDrive();
        updateFlywheelTarget();
        updateFlywheel();
        updateIntake();
        updateTelemetry();
    }

    // ══════════════════════════════════════════════════════════════
    // VOLTAGE MONITORING
    // ══════════════════════════════════════════════════════════════

    private void updateVoltage() {
        if (voltageSensor != null) {
            voltageSamples[voltageSampleIndex] = voltageSensor.getVoltage();
            voltageSampleIndex = (voltageSampleIndex + 1) % TrowelConfigs.VOLTAGE_SAMPLE_COUNT;
            double sum = 0;
            for (double s : voltageSamples) sum += s;
            smoothedVoltage = sum / TrowelConfigs.VOLTAGE_SAMPLE_COUNT;
        }
        if      (smoothedVoltage < TrowelConfigs.VOLTAGE_BROWNOUT_THRESHOLD)  inBrownout = true;
        else if (smoothedVoltage > TrowelConfigs.VOLTAGE_RECOVERY_THRESHOLD)  inBrownout = false;
    }

    // ══════════════════════════════════════════════════════════════
    // GENERAL APRILTAG DETECTION  (runs every loop, independent of aim)
    // ══════════════════════════════════════════════════════════════

    private void updateAprilTagDetection() {
        tagDetected        = false;
        detectedTagId      = -1;
        detectedTagRange   = 0;
        detectedTagBearing = 0;
        totalTagsVisible   = 0;

        if (!robot.isVisionEnabled()) return;

        List<AprilTagDetection> detections = robot.getDetections();
        totalTagsVisible = detections.size();
        if (detections.isEmpty()) return;

        AprilTagDetection best = null;
        double bestRange = Double.MAX_VALUE;
        for (AprilTagDetection d : detections) {
            if (d.ftcPose != null && d.ftcPose.range < bestRange) {
                bestRange = d.ftcPose.range;
                best = d;
            }
        }
        if (best != null) {
            tagDetected        = true;
            detectedTagId      = best.id;
            detectedTagRange   = best.ftcPose.range;
            detectedTagBearing = best.ftcPose.bearing;
        }
    }

    // ══════════════════════════════════════════════════════════════
    // CONTINUOUS ODOMETRY CORRECTION  (camera → pinpoint, NOT aiming)
    // ══════════════════════════════════════════════════════════════

    private void continuouslyUpdateOdometryFromCamera() {
        if (!robot.isVisionEnabled() || !tagDetected) return;

        List<AprilTagDetection> detections = robot.getDetections();
        for (AprilTagDetection det : detections) {
            if (det.id == detectedTagId && det.robotPose != null && det.ftcPose != null) {
                try {
                    org.firstinspires.ftc.robotcore.external.navigation.Position robotPos =
                            det.robotPose.getPosition();
                    org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles robotYpr =
                            det.robotPose.getOrientation();

                    double xInches    = robotPos.toUnit(DistanceUnit.INCH).x;
                    double yInches    = robotPos.toUnit(DistanceUnit.INCH).y;
                    double headingDeg = robotYpr.getYaw(AngleUnit.DEGREES);

                    if (xInches < 0 || xInches > 144 || yInches < 0 || yInches > 144) {
                        double[] recalc = recalcPoseFromTag(det, headingDeg);
                        if (recalc == null) continue;
                        xInches = recalc[0];
                        yInches = recalc[1];
                    }

                    if (xInches < -10 || xInches > 154 || yInches < -10 || yInches > 154) continue;

                    if (robot.pinpoint != null) {
                        robot.pinpoint.setPosition(new Pose2D(
                                DistanceUnit.INCH, xInches, yInches, AngleUnit.DEGREES, headingDeg));
                        if (odometry != null) odometry.update();
                        lastCameraPoseX       = xInches;
                        lastCameraPoseY       = yInches;
                        lastCameraPoseHeading = headingDeg;
                    }
                    break;
                } catch (Exception ignored) {}
            }
        }
    }

    // ══════════════════════════════════════════════════════════════
    // DRIVE & CONTROLS
    // ══════════════════════════════════════════════════════════════

    private void updateDrive() {
        isSlowMode = gamepad1.right_bumper;

        // Manual drive only (no auto-aim)
        double fwd = -gamepad1.left_stick_y;
        double str =  gamepad1.left_stick_x;
        double rot =  gamepad1.right_stick_x;

        if (Math.abs(fwd) < TrowelConfigs.DRIVE_DEADZONE) fwd = 0;
        if (Math.abs(str) < TrowelConfigs.DRIVE_DEADZONE) str = 0;
        if (Math.abs(rot) < TrowelConfigs.DRIVE_DEADZONE) rot = 0;

        // Apply speed limits
        double speedMultiplier = 0.6; // Limit to 60% of normal speed
        fwd *= speedMultiplier;
        str *= speedMultiplier;
        rot *= speedMultiplier;

        if (isSlowMode) {
            fwd *= 0.5;  // 50% of already-limited speed
            str *= 1;
            rot *= 0.5;
        }

        double fl = fwd + str + rot;
        double fr = fwd - str - rot;
        double bl = fwd - str + rot;
        double br = fwd + str - rot;

        double max = Math.max(Math.abs(fl), Math.max(Math.abs(fr),
                Math.max(Math.abs(bl), Math.abs(br))));
        if (max > 1.0) { fl /= max; fr /= max; bl /= max; br /= max; }

        if (inBrownout) { fl *= TrowelConfigs.BROWNOUT_DRIVE_SCALE; fr *= TrowelConfigs.BROWNOUT_DRIVE_SCALE;
            bl *= TrowelConfigs.BROWNOUT_DRIVE_SCALE; br *= TrowelConfigs.BROWNOUT_DRIVE_SCALE; }

        applyDrivePower(fl, fr, bl, br);
    }


    private void applyDrivePower(double fl, double fr, double bl, double br) {
        flPower = fl; frPower = fr; blPower = bl; brPower = br;
        if (robot.frontLeft  != null) robot.frontLeft.setPower(fl);
        if (robot.frontRight != null) robot.frontRight.setPower(fr);
        if (robot.backLeft   != null) robot.backLeft.setPower(bl);
        if (robot.backRight  != null) robot.backRight.setPower(br);
    }


    // ══════════════════════════════════════════════════════════════
    // FLYWHEEL
    // ══════════════════════════════════════════════════════════════

    private void updateFlywheelTarget() {
        if (gamepad1.dpad_up && !prevDpadUp)
            flywheelTarget = Math.min(flywheelTarget + TrowelConfigs.FLYWHEEL_STEP, TrowelConfigs.FLYWHEEL_MAX);
        prevDpadUp = gamepad1.dpad_up;

        if (gamepad1.dpad_down && !prevDpadDown)
            flywheelTarget = Math.max(flywheelTarget - TrowelConfigs.FLYWHEEL_STEP, TrowelConfigs.FLYWHEEL_MIN);
        prevDpadDown = gamepad1.dpad_down;
    }

    private void updateFlywheel() {
        if (gamepad1.x && !prevFlywheelToggle) {
            flywheelOn = !flywheelOn;
            if (!flywheelOn) stopFlywheel();
        }
        prevFlywheelToggle = gamepad1.x;
        updateFlywheelCommon(gamepad1.left_bumper);
    }

    private void updateFlywheelCommon(boolean shooting) {
        vel1 = (robot.deposit1 != null) ? Math.abs(robot.deposit1.getVelocity()) : 0;
        vel2 = (robot.deposit2 != null) ? Math.abs(robot.deposit2.getVelocity()) : 0;
        avgVel = (vel1 + vel2) / 2.0;

        if (!flywheelOn) { commandedTarget = 0; return; }

        // Limit flywheel speed to 70% of configured max
        double maxFlywheel = flywheelTarget * 0.7;
        double shootBoost = shooting ? Math.min(TrowelConfigs.SHOOT_BOOST * 0.6, 200) : 0;
        double exactTarget = maxFlywheel + shootBoost;
        commandedTarget = exactTarget;

        double velocityError = exactTarget - avgVel;
        if (!inRecoveryMode && velocityError > TrowelConfigs.RECOVERY_THRESHOLD) {
            inRecoveryMode = true;  updatePIDF(TrowelConfigs.RECOVERY_P);
        } else if (inRecoveryMode && velocityError < TrowelConfigs.RECOVERY_EXIT) {
            inRecoveryMode = false; updatePIDF(TrowelConfigs.PIDF_P);
        }

        setFlywheelVelocity(exactTarget);
    }

    private void setFlywheelVelocity(double velocity) {
        if (robot.deposit1 != null)
            robot.deposit1.setVelocity(TrowelConfigs.DEPOSIT1_REVERSED ? -velocity : velocity);
        if (robot.deposit2 != null)
            robot.deposit2.setVelocity(TrowelConfigs.DEPOSIT2_REVERSED ? -velocity : velocity);
    }

    private void stopFlywheel() {
        setFlywheelVelocity(0);
        inRecoveryMode = false;
        updatePIDF(TrowelConfigs.PIDF_P);
    }

    private void updatePIDF(double pValue) {
        if (Math.abs(pValue - lastP) < 0.01) return;
        try {
            if (robot.deposit1 != null)
                robot.deposit1.setVelocityPIDFCoefficients(pValue, TrowelConfigs.PIDF_I, TrowelConfigs.PIDF_D, TrowelConfigs.PIDF_F);
            if (robot.deposit2 != null)
                robot.deposit2.setVelocityPIDFCoefficients(pValue, TrowelConfigs.PIDF_I, TrowelConfigs.PIDF_D, TrowelConfigs.PIDF_F);
            lastP = pValue;
        } catch (Exception ignored) {}
    }

    // ══════════════════════════════════════════════════════════════
    // INTAKE
    // ══════════════════════════════════════════════════════════════

    private void updateIntake() {
        double desired1;
        if      (gamepad1.left_trigger  > 0.3) desired1 =  TrowelConfigs.INTAKE_POWER;
        else if (gamepad1.right_trigger > 0.3) desired1 = -TrowelConfigs.INTAKE_POWER;
        else                                   desired1 =  0;

        double desired2;
        if      (gamepad1.a) desired2 =  TrowelConfigs.INTAKE_POWER;
        else if (gamepad1.b) desired2 = -TrowelConfigs.INTAKE_POWER;
        else                 desired2 =  0;

        if (inBrownout) { desired1 *= TrowelConfigs.BROWNOUT_INTAKE_SCALE; desired2 *= TrowelConfigs.BROWNOUT_INTAKE_SCALE; }

        intake1Power = desired1;
        intake2Power = desired2;
        if (robot.intake1 != null) robot.intake1.setPower(intake1Power);
        if (robot.intake2 != null) robot.intake2.setPower(intake2Power);
    }


    // ══════════════════════════════════════════════════════════════
    // TELEMETRY
    // ══════════════════════════════════════════════════════════════

    private void updateTelemetry() {
        // ── Battery ──
        if (inBrownout)
            telemetry.addData("Battery", "%.2fV ⚠ BROWNOUT", smoothedVoltage);
        else
            telemetry.addData("Battery", "%.2fV", smoothedVoltage);

        // ── Alliance ──
        String allianceStr;
        switch (selectedAlliance) {
            case RED:  allianceStr = "🔴 RED";  break;
            case BLUE: allianceStr = "🔵 BLUE"; break;
            default:   allianceStr = "⬜ NONE"; break;
        }
        telemetry.addData("Alliance", allianceStr);
        telemetry.addData("Pose Init", initialPoseEstablished ? "✓" : "⏳");
        telemetry.addLine("");

        // ── AprilTag (general) ──
        telemetry.addLine("── APRILTAG ──");
        telemetry.addData("Vision", robot.getVisionStatusString());
        telemetry.addData("Tags Visible", totalTagsVisible);
        if (tagDetected)
            telemetry.addData("Closest Tag", "#%d  %.1f\"  bearing %.1f°",
                    detectedTagId, detectedTagRange, detectedTagBearing);
        telemetry.addLine("");

        // ── Flywheel ──
        telemetry.addLine("── FLYWHEEL ──");
        if (flywheelOn) {
            double error  = commandedTarget - avgVel;
            String status = inRecoveryMode ? "RECOVERING"
                    : (Math.abs(error) < 20) ? "READY ✓" : "STABILIZING";
            telemetry.addData("Status", status);
            telemetry.addData("Target", "%.0f ticks/s [Dpad ±%.0f]", flywheelTarget, TrowelConfigs.FLYWHEEL_STEP);
            telemetry.addData("Actual", "%.0f / %.0f (err: %.0f)", avgVel, commandedTarget, error);
            telemetry.addData("Motors", "%.0f | %.0f", vel1, vel2);
        } else {
            telemetry.addLine("OFF (GP2.X to start)");
            telemetry.addData("Target", "%.0f", flywheelTarget);
        }
        telemetry.addLine("");

        // ── Drive ──
        telemetry.addLine("── DRIVE ──");
        String modeStr = isSlowMode ? "SLOW (40%)" : "NORMAL";
        telemetry.addData("Mode", modeStr);
        telemetry.addData("FL|FR", "%+.2f | %+.2f", flPower, frPower);
        telemetry.addData("BL|BR", "%+.2f | %+.2f", blPower, brPower);
        telemetry.addLine("");

        // ── Intakes ──
        String i1 = intake1Power > 0.1 ? "IN" : intake1Power < -0.1 ? "OUT" : "OFF";
        String i2 = intake2Power > 0.1 ? "IN" : intake2Power < -0.1 ? "OUT" : "OFF";
        telemetry.addData("Intakes", "%s | %s", i1, i2);
        telemetry.addLine("");

        // ── Odometry ──
        telemetry.addLine("── ODOMETRY ──");
        if (odoEnabled && odometry != null) {
            Odometry.Position p = odometry.getPosition();
            telemetry.addData("Position", "X: %.1f in  Y: %.1f in",
                    p.xMm / 25.4, p.yMm / 25.4);
            telemetry.addData("Heading", "%.1f°", Math.toDegrees(p.headingRad));
            if (selectedAlliance != Alliance.NONE) {
                double gx = selectedAlliance == Alliance.BLUE ? TrowelConfigs.BLUE_GOAL_X : TrowelConfigs.RED_GOAL_X;
                double gy = selectedAlliance == Alliance.BLUE ? TrowelConfigs.BLUE_GOAL_Y : TrowelConfigs.RED_GOAL_Y;
                double dx = gx - (p.xMm / 25.4);
                double dy = gy - (p.yMm / 25.4);
                telemetry.addData("Dist to Goal", "%.1f in", Math.sqrt(dx * dx + dy * dy));
            }
            if (lastCameraPoseX != 0 || lastCameraPoseY != 0)
                telemetry.addData("Last Cam Pose", "X:%.1f Y:%.1f H:%.1f°",
                        lastCameraPoseX, lastCameraPoseY, lastCameraPoseHeading);
        } else {
            telemetry.addLine("Odometry not available");
        }

        telemetry.addLine("");
        double hz = loopTimeMs > 0 ? 1000.0 / loopTimeMs : 0;
        telemetry.addData("Loop", "%.1fms (%.0fHz)", loopTimeMs, hz);
        telemetry.update();
    }
}












