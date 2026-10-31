package org.firstinspires.ftc.teamcode.main;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.auxiliary.FieldGeometry;
import org.firstinspires.ftc.teamcode.auxiliary.FlywheelController;
import org.firstinspires.ftc.teamcode.auxiliary.PinpointLocalizer;
import org.firstinspires.ftc.teamcode.config.RobotConstants;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;

/**
 * Basic LinearOpMode autonomous, no Pedro Pathing: drive straight forward
 * AUTO_DRIVE_DISTANCE_IN inches, spin up the flywheel, open the gate, then run
 * the intake forward / reverse / forward to feed the gate, and stop.
 *
 * Every phase is timed (or distance-gated for the drive) and all tunables live
 * in RobotConstants for FTC Dashboard editing.
 */
@Autonomous(name = "Auto: Basic", group = "Main")
public class BasicAuto extends LinearOpMode {

    private enum Stage {
        DRIVE_FORWARD, SPIN_UP, GATE_OPEN, INTAKE_RUN, INTAKE_REVERSE, INTAKE_RUN_TWO, DONE
    }

    @Override
    public void runOpMode() {
        RobotHardware hardware = new RobotHardware(hardwareMap);
        PinpointLocalizer localizer = new PinpointLocalizer(hardwareMap);
        FlywheelController flywheel = new FlywheelController(
                hardware.leftFlywheel,
                hardware.rightFlywheel,
                hardwareMap != null && hardwareMap.voltageSensor.iterator().hasNext()
                        ? hardwareMap.voltageSensor.iterator().next() : null);

        // Gate shut so nothing falls through while driving / spinning up.
        hardware.setDrivePowers(0.0, 0.0, 0.0, 0.0);
        hardware.setBlockerClosed(true);

        // The Pinpoint starts at heading 0 after resetPosAndIMU; re-stamp the
        // pose so the zeroed position is reported at AUTO_START_HEADING_DEG.
        localizer.update();
        localizer.setPose(localizer.getXInches(), localizer.getYInches(),
                Math.toRadians(RobotConstants.AUTO_START_HEADING_DEG));

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        Stage stage = Stage.DRIVE_FORWARD;
        ElapsedTime stageTimer = new ElapsedTime();
        boolean flywheelEnabled = false;
        boolean gateOpen = false;

        while (!isStarted() && !isStopRequested()) {
            telemetry.addData("Basic auto", "ready - place robot facing the goal");
            telemetry.addData("Drive distance", String.format("%.1f in at %.2f power", RobotConstants.AUTO_DRIVE_DISTANCE_IN, RobotConstants.AUTO_DRIVE_POWER));
            telemetry.addData("Pinpoint", localizer.getStatusText());
            telemetry.update();
            sleep(20);
        }

        stageTimer.reset();

        while (opModeIsActive()) {
            // One Pinpoint update per loop, always, before anything reads it.
            localizer.update();

            double intakePower = 0.0;

            switch (stage) {
                case DRIVE_FORWARD: {
                    // Straight line holding AUTO_START_HEADING_DEG so the drive
                    // does not curve off; Pinpoint is the distance source.
                    double headingError = FieldGeometry.angleWrap(
                            Math.toRadians(RobotConstants.AUTO_START_HEADING_DEG) - localizer.getHeadingRadians());
                    double turn = Range.clip(
                            RobotConstants.AUTO_HEADING_HOLD_GAIN * headingError,
                            -RobotConstants.AUTO_HEADING_MAX_TURN,
                            RobotConstants.AUTO_HEADING_MAX_TURN);
                    double power = RobotConstants.AUTO_DRIVE_POWER;
                    hardware.setDrivePowers(power + turn, power + turn, power - turn, power - turn);

                    if (localizer.getXInches() >= RobotConstants.AUTO_DRIVE_DISTANCE_IN
                            || stageTimer.seconds() > RobotConstants.AUTO_DRIVE_TIMEOUT_S) {
                        hardware.setDrivePowers(0.0, 0.0, 0.0, 0.0);
                        // Arrived at the corner: snap the heading estimate to
                        // the nearest cardinal so the pose reads 0/90/180/270.
                        localizer.setPose(localizer.getXInches(), localizer.getYInches(),
                                snapToNearestCardinal(localizer.getHeadingRadians()));
                        flywheelEnabled = true;
                        stage = Stage.SPIN_UP;
                        stageTimer.reset();
                    }
                    break;
                }

                case SPIN_UP: {
                    if (stageTimer.seconds() >= RobotConstants.AUTO_SPIN_UP_SECONDS) {
                        hardware.setBlockerClosed(false); // open the gate
                        gateOpen = true;
                        stage = Stage.GATE_OPEN;
                        stageTimer.reset();
                    }
                    break;
                }

                case GATE_OPEN: {
                    if (stageTimer.seconds() >= RobotConstants.AUTO_GATE_WAIT_SECONDS) {
                        stage = Stage.INTAKE_RUN;
                        stageTimer.reset();
                    }
                    break;
                }

                case INTAKE_RUN: {
                    intakePower = RobotConstants.INTAKE_POWER;
                    if (stageTimer.seconds() >= RobotConstants.AUTO_INTAKE1_SECONDS) {
                        stage = Stage.INTAKE_REVERSE;
                        stageTimer.reset();
                    }
                    break;
                }

                case INTAKE_REVERSE: {
                    intakePower = -RobotConstants.INTAKE_POWER;
                    if (stageTimer.seconds() >= RobotConstants.AUTO_REVERSE_SECONDS) {
                        stage = Stage.INTAKE_RUN_TWO;
                        stageTimer.reset();
                    }
                    break;
                }

                case INTAKE_RUN_TWO: {
                    intakePower = RobotConstants.INTAKE_POWER;
                    if (stageTimer.seconds() >= RobotConstants.AUTO_INTAKE2_SECONDS) {
                        // Sequence over: stop the flywheel too and idle here.
                        flywheelEnabled = false;
                        stage = Stage.DONE;
                        stageTimer.reset();
                    }
                    break;
                }

                case DONE:
                default: {
                    break;
                }
            }

            // Flywheel runs in every phase after the drive (never stops early);
            // the controller runs every loop, exactly like TeleOp.
            if (flywheelEnabled) {
                flywheel.setTarget(RobotConstants.FLYWHEEL_TICKS_PER_SECOND);
                flywheel.update();
            } else {
                flywheel.stop();
            }

            if (hardware.intakeMotor != null) {
                hardware.intakeMotor.setPower(intakePower);
            }

            telemetry.addData("Stage", stage);
            telemetry.addData("Stage time", "%.1f s", stageTimer.seconds());
            telemetry.addData("Pose", String.format("(%.1f in, %.1f in, %.1f deg)",
                    localizer.getXInches(), localizer.getYInches(), Math.toDegrees(localizer.getHeadingRadians())));
            telemetry.addData("Gate", gateOpen ? "OPEN" : "closed");
            telemetry.addData("Shooter", String.format("target %.1f ticks/s, at %.1f ticks/s, ready %s",
                    flywheel.getTarget(), flywheel.getVelocity(), flywheel.isReady() ? "yes" : "no"));
            telemetry.addData("Shooter power", flywheel.getOutputPower());
            telemetry.update();
        }

        // Clean shutdown: stop driving, feeding, and the flywheel; close gate.
        hardware.setDrivePowers(0.0, 0.0, 0.0, 0.0);
        hardware.stopMechanisms();
        flywheel.stop();
        hardware.setBlockerClosed(true);
    }

    /**
     * Snaps a heading (radians) to the nearest multiple of 90 degrees:
     * 0, 90, 180, or 270 (equivalently -180..180 after wrapping). Only touches
     * the pose estimate, not the physical robot.
     */
    private static double snapToNearestCardinal(double headingRadians) {
        double degrees = Math.toDegrees(headingRadians);
        double snapped = Math.round(degrees / 90.0) * 90.0;
        return FieldGeometry.angleWrap(Math.toRadians(snapped));
    }
}
