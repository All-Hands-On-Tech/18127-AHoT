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
 * "instant" = true means the gate move doesn't wait for the servo to settle.
 * To run an action while the robot is driving, attach it to the drive command:
 * PedroCommands.follow(follower, path).with(Auxiliaries.shoot(1.5, 3.0, 0.0))
 */
@Config("AutoAuxiliaries")
public final class Auxiliaries {
    private Auxiliaries() {
    }

    public static double FLYWHEEL_TARGET_TICKS_PER_SECOND = 1225.0;

    // Auto-only flywheel PIDF (TeleOp uses RobotConstants.FLYWHEEL_P/I/D/F)
    public static double AUTO_FLYWHEEL_P = RobotConstants.FLYWHEEL_P + 2.0;
    public static double AUTO_FLYWHEEL_I = RobotConstants.FLYWHEEL_I;
    public static double AUTO_FLYWHEEL_D = RobotConstants.FLYWHEEL_D;
    public static double AUTO_FLYWHEEL_F = RobotConstants.FLYWHEEL_F + 0.7;

    public static double GATE_SETTLE_SECONDS = 0.30;
    public static double INTAKE_POWER = RobotConstants.INTAKE_POWER;
    public static double FLOWER_DOWN_POWER = -1.0;    // flowerRampDown only
    public static double FLOWER_UP_POWER = 1.0;       // flowerRampUp only

    private static RobotHardware hardware;
    private static FlywheelController flywheel;

    public static void init(HardwareMap map) {
        hardware = new RobotHardware(map);
        flywheel = new FlywheelController(hardware.shooterMotor);
    }

    public static void shutdown() {
        if (flywheel != null) flywheel.stop();
        secure();
        hardware.setFlowerPower(0.0);
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
                                          boolean openGate, double power) {
        return sequential(
                gate(true, before, instant),
                startFlywheel(),
                openGate ? gate(false, 0.0, false) : Commands.instant(() -> hardware.setBlockerClosed(true)),
                feed(runSeconds, power));
    }

    public static CommandBuilder intakeOff(double before, boolean instant) {
        return sequential(
                gate(true, before, instant),
                Commands.instant(() -> secure()));
    }

    public static CommandBuilder flowerRampUp(double before, double runSeconds, boolean instant) {
        return pulseFlower(before, runSeconds, FLOWER_UP_POWER, instant);
    }

    public static CommandBuilder flowerRampDown(double before, double runSeconds, boolean instant) {
        return pulseFlower(before, runSeconds, FLOWER_DOWN_POWER, instant);
    }

    public static CommandBuilder shoot(double before, double feedSeconds, double power) {
        return sequential(
                Commands.waitMs(ms(before)),
                startFlywheel(),
                gate(false, 0.0, false),
                feed(feedSeconds, power));
    }

    public static Command wait(double seconds) {
        return Commands.waitMs(ms(seconds));
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

    /** Applies the auto PIDF and target. Re-read from Dashboard every time this runs. */
    private static CommandBuilder startFlywheel() {
        return Commands.instant(() -> {
            flywheel.setTarget(FLYWHEEL_TARGET_TICKS_PER_SECOND);
            flywheel.update(AUTO_FLYWHEEL_P, AUTO_FLYWHEEL_I, AUTO_FLYWHEEL_D, AUTO_FLYWHEEL_F);
        });
    }

    private static CommandBuilder feed(double runSeconds, double power) {
        final ElapsedTime run = new ElapsedTime();
        return Command.build()
                .setStart(run::reset)
                .setExecute(() -> hardware.setIntakePower(INTAKE_POWER - power))
                .setDone(() -> runSeconds > 0.0 && run.seconds() >= runSeconds)
                .setEnd(endCondition -> secure());
    }

    /** Leaves the intake running with the gate closed so balls ride along while driving. */
    public static CommandBuilder intakeCarry() {
        return sequential(
                Commands.instant(() -> {
                    hardware.setBlockerClosed(true);
                    hardware.setIntakePower(INTAKE_POWER);
                }));
    }

    private static void secure() {
        hardware.setIntakePower(0.0);
        hardware.setBlockerClosed(true);
    }

    /** Runs the flower one direction for runSeconds, then stops. Never holds power. */
    private static CommandBuilder pulseFlower(double before, double runSeconds, double power,
                                              boolean instant) {
        final ElapsedTime timer = new ElapsedTime();
        return sequential(
                gate(true, before, instant),
                Command.build()
                        .setStart(timer::reset)
                        .setExecute(() -> hardware.setFlowerPower(power))
                        .setDone(() -> timer.seconds() >= Math.max(runSeconds, 0.0))
                        .setEnd(endCondition -> hardware.setFlowerPower(0.0)));
    }

    private static long ms(double seconds) {
        return Math.round(seconds * 1000.0);
    }
}