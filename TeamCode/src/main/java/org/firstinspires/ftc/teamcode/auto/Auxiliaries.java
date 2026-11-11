package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.CommandBuilder;
import com.pedropathing.ivy.commands.Commands;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.auxiliary.FlywheelController;
import org.firstinspires.ftc.teamcode.config.RobotConstants;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;

import static com.pedropathing.ivy.groups.Groups.sequential;

/**
 * Mechanism actions for the auto plans. "before" = seconds to wait first;
 * "instant" = true means the gate move doesn't wait for the servo to settle
 * (use it for actions that run while the robot is driving).
 */
@Config("AutoAuxiliaries")
public final class Auxiliaries {
    private Auxiliaries() {
    }

    public static double FLYWHEEL_TARGET_TICKS_PER_SECOND = RobotConstants.FLYWHEEL_TICKS_PER_SECOND;
    public static double GATE_SETTLE_SECONDS = 0.30;
    public static double INTAKE_POWER = RobotConstants.INTAKE_POWER;
    public static double FLOWER_POWER = 1.0;

    private static RobotHardware hardware;
    private static FlywheelController flywheel;
    private static double flowerPower = 0.0;

    public static void init(HardwareMap map) {
        hardware = new RobotHardware(map);
        flywheel = new FlywheelController(hardware.shooterMotor);
        flowerPower = 0.0;
    }

    public static void shutdown() {
        if (flywheel != null) flywheel.stop();
        secure();
    }

    // ------------------------------------------------------------- actions

    public static CommandBuilder shooterOn(double before, double holdSeconds, boolean instant) {
        return sequential(
                gate(true, before, instant),
                startFlywheel(),
                Commands.waitMs(ms(holdSeconds)));
    }

    public static CommandBuilder shooterOff(double before) {
        return sequential(
                gate(true, before, false),
                Commands.instant(() -> flywheel.stop()));
    }

    public static CommandBuilder intakeOn(double before, double runSeconds, boolean instant,
                                          boolean openGate) {
        return sequential(
                gate(true, before, instant),
                startFlywheel(),
                openGate ? gate(false, 0.0, false) : Commands.instant(() -> hardware.setBlockerClosed(true)),
                feed(runSeconds));
    }

    public static CommandBuilder intakeOff(double before, boolean instant) {
        return sequential(
                gate(true, before, instant),
                Commands.instant(() -> secure()));
    }

    public static CommandBuilder flowerRampUp(double before, double rampSeconds, boolean instant) {
        return rampFlower(before, rampSeconds, FLOWER_POWER, instant);
    }

    public static CommandBuilder flowerRampDown(double before, double rampSeconds, boolean instant) {
        return rampFlower(before, rampSeconds, 0.0, instant);
    }

    public static CommandBuilder shoot(double before, double feedSeconds) {
        return sequential(
                Commands.waitMs(ms(before)),
                startFlywheel(),
                gate(false, 0.0, false),
                feed(feedSeconds));
    }

    // ------------------------------------------------------------- helpers

    private static CommandBuilder gate(boolean closed, double before, boolean instant) {
        if (instant) {
            return sequential(
                    Commands.waitMs(ms(before)),
                    Command.build()
                            .setStart(() -> hardware.setBlockerClosed(closed))
                            .setDone(() -> true));
        }
        final ElapsedTime settle = new ElapsedTime();
        return sequential(
                Commands.waitMs(ms(before)),
                Command.build()
                        .setStart(() -> {
                            hardware.setBlockerClosed(closed);
                            settle.reset();
                        })
                        .setDone(() -> settle.seconds() >= GATE_SETTLE_SECONDS));
    }

    private static CommandBuilder startFlywheel() {
        return Commands.instant(() -> {
            flywheel.setTarget(FLYWHEEL_TARGET_TICKS_PER_SECOND);
            flywheel.update();
        });
    }

    private static CommandBuilder feed(double runSeconds) {
        final ElapsedTime run = new ElapsedTime();
        return Command.build()
                .setStart(run::reset)
                .setExecute(() -> {
                    hardware.setIntakePower(INTAKE_POWER);
                    setFlower(FLOWER_POWER);
                })
                .setDone(() -> runSeconds > 0.0 && run.seconds() >= runSeconds)
                .setEnd(endCondition -> secure());
    }

    private static CommandBuilder rampFlower(double before, double rampSeconds, double targetPower,
                                             boolean instant) {
        final ElapsedTime timer = new ElapsedTime();
        final double[] from = {0.0};
        return sequential(
                gate(true, before, instant),
                Command.build()
                        .setStart(() -> {
                            timer.reset();
                            from[0] = flowerPower;
                        })
                        .setExecute(() -> {
                            double f = rampSeconds <= 0.0 ? 1.0 : Math.min(1.0, timer.seconds() / rampSeconds);
                            setFlower(from[0] + (targetPower - from[0]) * f);
                        })
                        .setDone(() -> timer.seconds() >= Math.max(rampSeconds, 0.0))
                        .setEnd(endCondition -> setFlower(targetPower)));
    }

    private static void setFlower(double power) {
        hardware.setFlowerPower(power);
        flowerPower = power;
    }

    private static void secure() {
        hardware.setIntakePower(0.0);
        setFlower(0.0);
        hardware.setBlockerClosed(true);
    }

    private static long ms(double seconds) {
        return Math.round(seconds * 1000.0);
    }
}