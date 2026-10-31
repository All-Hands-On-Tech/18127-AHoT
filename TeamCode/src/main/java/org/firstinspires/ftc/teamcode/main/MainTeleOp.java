package org.firstinspires.ftc.teamcode.main;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import java.util.TreeSet;

import org.firstinspires.ftc.teamcode.auxiliary.AllianceSelector;
import org.firstinspires.ftc.teamcode.auxiliary.AutoAimController;
import org.firstinspires.ftc.teamcode.auxiliary.FlywheelController;
import org.firstinspires.ftc.teamcode.auxiliary.PinpointLocalizer;
import org.firstinspires.ftc.teamcode.auxiliary.PoseStorage;
import org.firstinspires.ftc.teamcode.auxiliary.Relocalizer;
import org.firstinspires.ftc.teamcode.config.RobotConstants;
import org.firstinspires.ftc.teamcode.drive.Alliance;
import org.firstinspires.ftc.teamcode.drive.Drivetrain;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;

@TeleOp(name = "Main: BBB Driver", group = "Main")
public class MainTeleOp extends OpMode {
    private Drivetrain drivetrain;
    private RobotHardware hardware;
    private boolean shooterEnabled = false;
    private boolean shooterButtonPrev = false;
    private boolean blockerClosed = false;
    private boolean blockerButtonPrev = false;
    private boolean dpadUpPrev = false;
    private boolean dpadDownPrev = false;
    private double flywheelTargetTicksPerSecond = RobotConstants.FLYWHEEL_TICKS_PER_SECOND;
    private FlywheelController flywheelController;
    private double flowerPower = 0.0;
    private String flowerServoDetection = "";

    // Localization / aim / relocalize (pose read by everything through localizer).
    private PinpointLocalizer localizer;
    private AutoAimController autoAim;
    private Relocalizer relocalizer;
    private boolean relocalizeButtonPrev = false;
    private boolean aimBlockedRumblePrev = false;

    @Override
    public void init() {
        hardware = new RobotHardware(hardwareMap);
        flywheelController = new FlywheelController(
                hardware.leftFlywheel,
                hardware.rightFlywheel,
                hardwareMap != null && hardwareMap.voltageSensor.iterator().hasNext()
                        ? hardwareMap.voltageSensor.iterator().next() : null);
        drivetrain = new Drivetrain(hardwareMap);
        flowerServoDetection = describeFlowerServos();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // Constructing these validates the aim/relocalize constant arrays at
        // init; a length mismatch throws here with a clear message.
        localizer = new PinpointLocalizer(hardwareMap);
        autoAim = new AutoAimController(localizer);
        relocalizer = new Relocalizer(localizer);

        // A pose left by a previous Autonomous OpMode gives aiming a head start.
        if (PoseStorage.valid) {
            localizer.setPose(PoseStorage.xInches, PoseStorage.yInches, PoseStorage.headingRadians);
        }

        // Alliance selection persists from an earlier OpMode in the same boot.
        Alliance previous = AllianceSelector.get();
        if (previous != null) {
            drivetrain.setAlliance(previous);
        }

        if (hardware.blockerServo != null || hardware.axonBlockerServo != null) {
            hardware.setBlockerClosed(false);
        }
    }

    @Override
    public void init_loop() {
        AllianceSelector.update(gamepad1, gamepad2);
        AllianceSelector.addTelemetry(telemetry);

        telemetry.addData("Flower servo detection", flowerServoDetection);
        telemetry.addData("Pose valid", localizer.isPoseValid() ? "yes" : "no (relocalize or run Autonomous first)");
        telemetry.addData("Driver 1", "Left stick drive / Right stick turn / RT/LT intake");
        telemetry.addData("Driver 2", "Y toggles shooter / A toggles blocker / LB hold = auto aim / Back = relocalize / ZR/ZL flower control");
        telemetry.update();
    }

    @Override
    public void start() {
        Alliance alliance = AllianceSelector.get();
        if (alliance != null) {
            drivetrain.setAlliance(alliance);
            autoAim.setAlliance(alliance);
        }
    }

    @Override
    public void loop() {
        // Localization first: exactly one Pinpoint update per loop.
        localizer.update();

        Alliance alliance = AllianceSelector.get();
        autoAim.setAlliance(alliance);

        // Auto-aim target selection and heading math run every loop so
        // telemetry stays live; whether they MOVE the robot is gated below.
        autoAim.update();
        double aimTurn = autoAim.computeTurn();

        boolean aimHeld = gamepad2.left_bumper;
        boolean poseGoodForAim = !RobotConstants.AIM_REQUIRES_VALID_POSE || localizer.isPoseValid();
        boolean aimOverrideActive = aimHeld && alliance != null && poseGoodForAim;

        // Holding aim without a valid pose must not turn the robot: nudge
        // gamepad2 instead (edge-detected so the rumble doesn't machine-gun).
        boolean aimBlocked = aimHeld && alliance != null && !poseGoodForAim;
        if (aimBlocked && !aimBlockedRumblePrev) {
            gamepad2.rumble(0.5, 0.5, 200);
        }
        aimBlockedRumblePrev = aimBlocked;

        // Aiming replaces only the turn stick; driver 1 translation untouched.
        drivetrain.update(gamepad1, gamepad2, aimOverrideActive, aimTurn);

        if (gamepad1.dpad_up && !dpadUpPrev) {
            flywheelTargetTicksPerSecond += 10.0;
        }
        if (gamepad1.dpad_down && !dpadDownPrev) {
            flywheelTargetTicksPerSecond = Math.max(0.0, flywheelTargetTicksPerSecond - 10.0);
        }
        dpadUpPrev = gamepad1.dpad_up;
        dpadDownPrev = gamepad1.dpad_down;

        if (gamepad2.y && !shooterButtonPrev) {
            shooterEnabled = !shooterEnabled;
        }
        shooterButtonPrev = gamepad2.y;

        // Custom flywheel controller runs every loop (even while shooting).
        if (shooterEnabled) {
            flywheelController.setTarget(flywheelTargetTicksPerSecond);
            flywheelController.update();
        } else {
            flywheelController.stop();
        }

        if (gamepad2.a && !blockerButtonPrev) {
            blockerClosed = !blockerClosed;
        }
        blockerButtonPrev = gamepad2.a;

        if (hardware.blockerServo != null || hardware.axonBlockerServo != null) {
            hardware.setBlockerClosed(blockerClosed);
        }

        // Driver 2 Back = corner relocalize (rising edge, one shot per press).
        if (gamepad2.back && !relocalizeButtonPrev && alliance != null) {
            relocalizer.relocalize(alliance);
            gamepad2.rumble(1.0, 1.0, 250);
        }
        relocalizeButtonPrev = gamepad2.back;

        if (hardware.hasContinuousServos) {
            double leftPower = 0.0;
            double rightPower = 0.0;

            /*
             * ZR spins the flower forward, ZL spins it back. The SDK has no
             * zr/zl fields: some controllers report them as analog triggers,
             * others only as digital presses (analog value stays 0), so check
             * both left_trigger/right_trigger and *_trigger_pressed.
             */
            boolean zrPressed = gamepad2.right_trigger > 0.05 || gamepad2.right_trigger_pressed;
            boolean zlPressed = gamepad2.left_trigger > 0.05 || gamepad2.left_trigger_pressed;

            if (zrPressed) {
                leftPower = 1.0;
                rightPower = 1.0;
            } else if (zlPressed) {
                leftPower = -1.0;
                rightPower = -1.0;
            }

            hardware.continuousServoLeft.setPower(leftPower);
            hardware.continuousServoRight.setPower(rightPower);

            flowerPower = leftPower;
        }

        if (gamepad1.right_trigger > 0.05) {
            if (hardware.intakeMotor != null) {
                hardware.intakeMotor.setPower(1.0);
            }
        } else if (gamepad1.left_trigger > 0.05) {
            if (hardware.intakeMotor != null) {
                hardware.intakeMotor.setPower(-1.0);
            }
        } else if (hardware.intakeMotor != null) {
            hardware.intakeMotor.setPower(0.0);
        }

        double leftVelocity = hardware.leftFlywheel != null ? hardware.leftFlywheel.getVelocity() : 0.0;
        double rightVelocity = hardware.rightFlywheel != null ? hardware.rightFlywheel.getVelocity() : 0.0;
        double averageVelocity = (leftVelocity + rightVelocity) * 0.5;
        double velocityError = (shooterEnabled ? flywheelTargetTicksPerSecond : 0.0) - averageVelocity;

        // Numeric values for live Panels graphing.
        telemetry.addData("graph/flywheel_target_ticks_per_s", shooterEnabled ? flywheelTargetTicksPerSecond : 0.0);
        telemetry.addData("graph/flywheel_left_ticks_per_s", leftVelocity);
        telemetry.addData("graph/flywheel_right_ticks_per_s", rightVelocity);
        telemetry.addData("graph/flywheel_avg_ticks_per_s", averageVelocity);
        telemetry.addData("graph/flywheel_error_ticks_per_s", velocityError);
        telemetry.addData("graph/flywheel_controller_power", flywheelController.getOutputPower());
        telemetry.addData("graph/flywheel_ready", flywheelController.isReady());
        telemetry.addData("graph/flower_power", flowerPower);
        telemetry.addData("graph/aim_heading_error_deg", autoAim.getHeadingErrorDegrees());

        telemetry.addData("Alliance", alliance != null ? alliance.getDisplayName() : "none selected (X = blue, B = red)");
        telemetry.addData("Drive enabled", drivetrain.hasDrive());
        telemetry.addData("Pinpoint", localizer.getStatusText());
        telemetry.addData("Pose", String.format("(%.1f in, %.1f in, %.1f deg)",
                localizer.getXInches(), localizer.getYInches(), Math.toDegrees(localizer.getHeadingRadians())));
        telemetry.addData("Pose valid", localizer.isPoseValid() ? "yes" : "no - press Back at a corner to relocalize");
        telemetry.addData("Auto aim", aimOverrideActive ? "DRIVING TURN" : aimHeld ? "held, not driving (no alliance/pose)" : "off (hold LB)");
        telemetry.addData("Goal point", String.format("#%d (%.1f in, %.1f in), %.1f in away",
                autoAim.getTargetIndex(), autoAim.getTargetXInches(), autoAim.getTargetYInches(), autoAim.getDistanceInches()));
        telemetry.addData("Aim error", String.format("%.1f deg %s", autoAim.getHeadingErrorDegrees(), autoAim.isAimed() ? "(AIMED)" : ""));
        if (relocalizer.getLastIndex() >= 0) {
            telemetry.addData("Last relocalize", String.format("corner #%d (%s heading)",
                    relocalizer.getLastIndex(), relocalizer.isLastHeadingKept() ? "kept" : "set from constant"));
        }
        if (alliance == null) {
            telemetry.addData("Aim/relocalize", "disabled - no alliance selected");
        }
        if (aimBlocked) {
            telemetry.addData("Relocalize first", "pose invalid, holding aim does not turn");
        }

        telemetry.addData("Driver 1", "LS drive / RS turn / RT/LT intake / D-pad up/down flywheel target +/-10");
        telemetry.addData("Driver 2", "Y toggles shooter / A toggles blocker / LB auto aim / Back relocalize / ZR/ZL flower");
        telemetry.addData("Shooter", shooterEnabled ? "ON" : "OFF");
        telemetry.addData("Shooter target", String.format("%.1f ticks/s", flywheelTargetTicksPerSecond));
        if (hardware.leftFlywheel != null) {
            telemetry.addData("Shooter actual left", leftVelocity);
        }
        if (hardware.rightFlywheel != null) {
            telemetry.addData("Shooter actual right", rightVelocity);
        }
        telemetry.addData("Blocker", blockerClosed ? "CLOSED" : "OPEN");
        telemetry.addData("Flower servo detection", flowerServoDetection);
        telemetry.addData("Flower", String.format("%.1f power (ZR = forward, ZL = back)", flowerPower));
        telemetry.update();
    }

    @Override
    public void stop() {
        if (flywheelController != null) {
            flywheelController.stop();
        }
        if (drivetrain != null) {
            drivetrain.stop();
        }
        if (hardware != null) {
            hardware.stopMechanisms();
        }
    }

    /**
     * Diagnostic for the flower (feed) servos: reports whether they were found
     * under the configured names (continuous_servo_left/right) and lists the
     * names actually present on the robot so misconfigured entries can be
     * spotted from the Driver Station.
     */
    private String describeFlowerServos() {
        if (hardware == null || hardwareMap == null) {
            return "unknown";
        }

        String crServoNames;
        try {
            TreeSet<String> names = new TreeSet<>(hardwareMap.getAllNames(CRServo.class));
            crServoNames = names.isEmpty() ? "<none>" : String.join(", ", names);
        } catch (Exception e) {
            crServoNames = "error: " + e.getMessage();
        }

        String summary = String.format("Left: %s, Right: %s | CRServos on robot: %s",
                hardware.continuousServoLeft != null ? "detected" : "MISSING",
                hardware.continuousServoRight != null ? "detected" : "MISSING",
                crServoNames);

        if (hardware.continuousServoLeft == null || hardware.continuousServoRight == null) {
            // Expected names are missing - dump every device name on the map.
            try {
                TreeSet<String> allNames = new TreeSet<>(hardwareMap.getAllNames(HardwareDevice.class));
                summary += " | All devices: " + String.join(", ", allNames);
            } catch (Exception e) {
                summary += " | All devices: error: " + e.getMessage();
            }
        }

        return summary;
    }
}
