package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.CommandBuilder;
import com.pedropathing.ivy.commands.Commands;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.auxiliary.FlywheelController;
import org.firstinspires.ftc.teamcode.config.RobotConstants;

import static com.pedropathing.ivy.groups.Groups.sequential;

/**
 * Mechanism actions used by the auto plans.
 *
 * Every helper uses the same timing pattern: wait before, do the action,
 * wait after. If a command is meant to run while the robot is moving, attach it
 * to the drive command with .with(...).
 */
@Config("AutoAuxiliaries")
public final class Auxiliaries {
    private Auxiliaries() {
    }

    public static double FLYWHEEL_TARGET_TICKS_PER_SECOND = RobotConstants.FLYWHEEL_TICKS_PER_SECOND;
    public static double GATE_SETTLE_SECONDS = 0.30;
    public static double INTAKE_POWER = RobotConstants.INTAKE_POWER;
    public static double FLOWER_POWER = 1.0;
    public static double FLOWER_RAMP_UP_SECONDS = 0.50;
    public static double FLOWER_RAMP_DOWN_SECONDS = 0.50;
    public static double SHOOT_FEED_SECONDS = 1.00;

    private static DcMotorEx intakeMotor;
    private static CRServo flowerLeft;
    private static CRServo flowerRight;
    private static Servo blockerServo;
    private static Servo axonBlockerServo;
    private static FlywheelController flywheel;

    private static boolean gateIsClosed = false;
    private static double flowerPower = 0.0;

    public static void init(HardwareMap hardwareMap) {
        intakeMotor = null;
        flywheel = null;
        flowerLeft = null;
        flowerRight = null;
        blockerServo = null;
        axonBlockerServo = null;
        if (hardwareMap != null) {
            intakeMotor = hardwareMap.tryGet(DcMotorEx.class, RobotConstants.INTAKE_MOTOR);
            flowerLeft = hardwareMap.tryGet(CRServo.class, RobotConstants.CONTINUOUS_SERVO_LEFT);
            flowerRight = hardwareMap.tryGet(CRServo.class, RobotConstants.CONTINUOUS_SERVO_RIGHT);
            blockerServo = hardwareMap.tryGet(Servo.class, RobotConstants.BLOCKER_SERVO);
            axonBlockerServo = hardwareMap.tryGet(Servo.class, RobotConstants.AXON_BLOCKER_SERVO);
            flywheel = new FlywheelController(
                    hardwareMap.tryGet(DcMotorEx.class, RobotConstants.SHOOTER_MOTOR), null);
        }
        if (flowerLeft != null) {
            flowerLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        }
        if (flowerRight != null) {
            flowerRight.setDirection(DcMotorSimple.Direction.FORWARD);
        }
        gateIsClosed = false;
        flowerPower = 0.0;
    }

    public static void shutdown() {
        if (flywheel != null) {
            flywheel.stop();
        }
        secure();
    }

    public static CommandBuilder gateClosed(double beforeSeconds, double actionSeconds,
                                           double afterSeconds, boolean moveAtSameTime) {
        if (moveAtSameTime) {
            return sequential(
                    Commands.waitMs(toMillis(beforeSeconds)),
                    Command.build()
                            .setStart(() -> setGate(true))
                            .setDone(() -> true),
                    Commands.waitMs(toMillis(afterSeconds)));
        }
        return moveGate(true, beforeSeconds, actionSeconds, afterSeconds);
    }

    public static CommandBuilder gateClosed(double beforeSeconds, double actionSeconds,
                                           double afterSeconds) {
        return gateClosed(beforeSeconds, actionSeconds, afterSeconds, false);
    }

    public static CommandBuilder gateClosed(double waitSeconds) {
        return gateClosed(waitSeconds, 0.0, 0.0, false);
    }

    public static CommandBuilder gateOpen(double beforeSeconds, double actionSeconds,
                                         double afterSeconds, boolean moveAtSameTime) {
        if (moveAtSameTime) {
            return sequential(
                    Commands.waitMs(toMillis(beforeSeconds)),
                    Command.build()
                            .setStart(() -> setGate(false))
                            .setDone(() -> true),
                    Commands.waitMs(toMillis(afterSeconds)));
        }
        return moveGate(false, beforeSeconds, actionSeconds, afterSeconds);
    }

    public static CommandBuilder gateOpen(double beforeSeconds, double actionSeconds,
                                         double afterSeconds) {
        return gateOpen(beforeSeconds, actionSeconds, afterSeconds, false);
    }

    public static CommandBuilder gateOpen(double waitSeconds) {
        return gateOpen(waitSeconds, 0.0, 0.0, false);
    }

    public static boolean isGateClosed() {
        return gateIsClosed;
    }

    public static CommandBuilder shooterOn(double beforeSeconds, double actionSeconds,
                                          double afterSeconds, boolean moveAtSameTime) {
        return sequential(
                gateClosed(beforeSeconds, 0.0, 0.0, moveAtSameTime),
                Commands.instant(() -> {
                    if (flywheel != null) {
                        flywheel.setTarget(FLYWHEEL_TARGET_TICKS_PER_SECOND);
                        flywheel.update();
                    }
                }),
                Commands.waitMs(toMillis(actionSeconds + afterSeconds)));
    }

    public static CommandBuilder shooterOn(double beforeSeconds, double actionSeconds,
                                          double afterSeconds) {
        return shooterOn(beforeSeconds, actionSeconds, afterSeconds, false);
    }

    public static CommandBuilder shooterOn(double waitSeconds, double holdSeconds) {
        return shooterOn(waitSeconds, holdSeconds, 0.0, false);
    }

    public static CommandBuilder shooterOff(double beforeSeconds, double actionSeconds,
                                           double afterSeconds, boolean moveAtSameTime) {
        return sequential(
                gateClosed(beforeSeconds, 0.0, 0.0, moveAtSameTime),
                Commands.instant(() -> {
                    if (flywheel != null) {
                        flywheel.stop();
                    }
                }),
                Commands.waitMs(toMillis(afterSeconds)));
    }

    public static CommandBuilder shooterOff(double beforeSeconds, double actionSeconds,
                                           double afterSeconds) {
        return shooterOff(beforeSeconds, actionSeconds, afterSeconds, false);
    }

    public static CommandBuilder shooterOff(double waitSeconds) {
        return shooterOff(waitSeconds, 0.0, 0.0, false);
    }

    public static CommandBuilder intakeOn(double beforeSeconds, double actionSeconds,
                                         double afterSeconds, boolean moveAtSameTime) {
        return intakeOn(beforeSeconds, actionSeconds, afterSeconds, moveAtSameTime, true);
    }

    public static CommandBuilder intakeOn(double beforeSeconds, double actionSeconds,
                                         double afterSeconds, boolean moveAtSameTime,
                                         boolean openGateForIntake) {
        return sequential(
                gateClosed(beforeSeconds, 0.0, 0.0, moveAtSameTime),
                Commands.instant(() -> {
                    if (flywheel != null) {
                        flywheel.setTarget(FLYWHEEL_TARGET_TICKS_PER_SECOND);
                        flywheel.update();
                    }
                }),
                openGateForIntake
                        ? gateOpen(0.0, 0.0, 0.0, false)
                        : Commands.instant(() -> setGate(true)),
                feed(actionSeconds),
                Commands.waitMs(toMillis(afterSeconds)));
    }

    public static CommandBuilder intakeOn(double beforeSeconds, double actionSeconds,
                                         double afterSeconds) {
        return intakeOn(beforeSeconds, actionSeconds, afterSeconds, false);
    }

    public static CommandBuilder intakeOn(double waitSeconds, double runSeconds) {
        return intakeOn(waitSeconds, runSeconds, 0.0, false);
    }

    public static CommandBuilder intakeOff(double beforeSeconds, double actionSeconds,
                                           double afterSeconds, boolean moveAtSameTime) {
        return sequential(
                gateClosed(beforeSeconds, 0.0, 0.0, moveAtSameTime),
                Commands.instant(() -> secure()),
                Commands.waitMs(toMillis(afterSeconds)));
    }

    public static CommandBuilder intakeOff(double beforeSeconds, double actionSeconds,
                                           double afterSeconds) {
        return intakeOff(beforeSeconds, actionSeconds, afterSeconds, false);
    }

    public static CommandBuilder intakeOff(double waitSeconds) {
        return intakeOff(waitSeconds, 0.0, 0.0, false);
    }

    public static CommandBuilder flowerRampUp(double beforeSeconds, double actionSeconds,
                                             double afterSeconds, boolean moveAtSameTime) {
        return rampFlower(beforeSeconds, actionSeconds, afterSeconds, FLOWER_POWER, moveAtSameTime);
    }

    public static CommandBuilder flowerRampUp(double beforeSeconds, double actionSeconds,
                                             double afterSeconds) {
        return flowerRampUp(beforeSeconds, actionSeconds, afterSeconds, false);
    }

    public static CommandBuilder flowerRampUp(double waitSeconds, double rampSeconds) {
        return flowerRampUp(waitSeconds, rampSeconds, 0.0, false);
    }

    public static CommandBuilder flowerRampDown(double beforeSeconds, double actionSeconds,
                                               double afterSeconds, boolean moveAtSameTime) {
        return rampFlower(beforeSeconds, actionSeconds, afterSeconds, 0.0, moveAtSameTime);
    }

    public static CommandBuilder flowerRampDown(double beforeSeconds, double actionSeconds,
                                               double afterSeconds) {
        return flowerRampDown(beforeSeconds, actionSeconds, afterSeconds, false);
    }

    public static CommandBuilder flowerRampDown(double waitSeconds, double rampSeconds) {
        return flowerRampDown(waitSeconds, rampSeconds, 0.0, false);
    }

    public static CommandBuilder shoot(double beforeSeconds, double actionSeconds,
                                      double afterSeconds, boolean moveAtSameTime) {
        return sequential(
                Commands.waitMs(toMillis(beforeSeconds)),
                Commands.instant(() -> {
                    if (flywheel != null) {
                        flywheel.setTarget(FLYWHEEL_TARGET_TICKS_PER_SECOND);
                        flywheel.update();
                    }
                }),
                gateOpen(0.0, 0.0, 0.0, false),
                feed(actionSeconds),
                Commands.waitMs(toMillis(afterSeconds)));
    }

    public static CommandBuilder shoot(double beforeSeconds, double actionSeconds,
                                      double afterSeconds) {
        return shoot(beforeSeconds, actionSeconds, afterSeconds, false);
    }

    public static CommandBuilder shoot(double waitSeconds, double feedSeconds) {
        return shoot(waitSeconds, feedSeconds, 0.0, false);
    }

    private static CommandBuilder moveGate(boolean closed, double beforeSeconds,
                                          double actionSeconds, double afterSeconds) {
        final ElapsedTime settle = new ElapsedTime();
        return sequential(
                Commands.waitMs(toMillis(beforeSeconds)),
                Command.build()
                        .setStart(() -> {
                            setGate(closed);
                            settle.reset();
                        })
                        .setDone(() -> settle.seconds() >= Math.max(GATE_SETTLE_SECONDS, actionSeconds)),
                Commands.waitMs(toMillis(afterSeconds)));
    }

    private static CommandBuilder spinUp(double holdSeconds) {
        final ElapsedTime hold = new ElapsedTime();
        final boolean[] atSpeed = {false};
        return Command.build()
                .setStart(hold::reset)
                .setExecute(() -> {
                    if (flywheel != null) {
                        flywheel.setTarget(FLYWHEEL_TARGET_TICKS_PER_SECOND);
                        flywheel.update();
                    }
                    if (!atSpeed[0] && shooterReady()) {
                        atSpeed[0] = true;
                        hold.reset();
                    }
                })
                .setDone(() -> atSpeed[0] && hold.seconds() >= holdSeconds);
    }

    private static CommandBuilder feed(double runSeconds) {
        final ElapsedTime run = new ElapsedTime();
        return Command.build()
                .setStart(run::reset)
                .setExecute(() -> {
                    runIntake(INTAKE_POWER);
                    runFlower(FLOWER_POWER);
                })
                .setDone(() -> runSeconds > 0.0 && run.seconds() >= runSeconds)
                .setEnd(endCondition -> secure());
    }

    private static CommandBuilder rampFlower(double beforeSeconds, double rampSeconds,
                                             double afterSeconds, double targetPower,
                                             boolean moveAtSameTime) {
        final ElapsedTime timer = new ElapsedTime();
        final double[] from = {0.0};
        return sequential(
                gateClosed(beforeSeconds, 0.0, 0.0, moveAtSameTime),
                Command.build()
                        .setStart(() -> {
                            timer.reset();
                            from[0] = flowerPower;
                        })
                        .setExecute(() -> {
                            double fraction = rampSeconds <= 0.0
                                    ? 1.0
                                    : Math.min(1.0, timer.seconds() / rampSeconds);
                            runFlower(from[0] + (targetPower - from[0]) * fraction);
                        })
                        .setDone(() -> timer.seconds() >= Math.max(rampSeconds, 0.0))
                        .setEnd(endCondition -> runFlower(targetPower)),
                Commands.waitMs(toMillis(afterSeconds)));
    }

    private static long toMillis(double seconds) {
        return Math.round(seconds * 1000.0);
    }

    private static boolean shooterReady() {
        return flywheel == null || flywheel.isReady();
    }

    private static void secure() {
        runIntake(0.0);
        runFlower(0.0);
        setGate(true);
    }

    private static void setGate(boolean closed) {
        if (blockerServo != null) {
            blockerServo.setPosition(closed ? 0.9 : 0.5);
        }
        if (axonBlockerServo != null) {
            axonBlockerServo.setPosition(closed ? 0.9 : 0.5);
        }
        gateIsClosed = closed;
    }

    private static void runIntake(double power) {
        if (intakeMotor != null) {
            intakeMotor.setPower(power);
        }
    }

    private static void runFlower(double power) {
        if (flowerLeft != null) {
            flowerLeft.setPower(power);
        }
        if (flowerRight != null) {
            flowerRight.setPower(power);
        }
        flowerPower = power;
    }
}
