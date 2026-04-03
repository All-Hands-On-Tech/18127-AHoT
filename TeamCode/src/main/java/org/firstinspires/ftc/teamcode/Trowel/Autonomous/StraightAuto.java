package org.firstinspires.ftc.teamcode.Trowel.Autonomous;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.Trowel.Configs.TrowelHardware;
import org.firstinspires.ftc.teamcode.Trowel.pedroPathing.Constants;

import java.util.Locale;

@Autonomous(name = "Trowel Auto", group = "Autonomous")
@Configurable
public class StraightAuto extends OpMode {

    public enum Team { NONE, RED, BLUE }
    private Team selectedTeam = Team.NONE;

    private TelemetryManager panelsTelemetry;
    public Follower follower;
    private int pathState = 0;
    private TrowelHardware robot;

    private PathChain Depo1, IntakeStart1, IntakeEnd1;
    private PathChain Depo2, IntakeStart2, IntakeEnd2;
    private PathChain Depo3, IntakeStart3, IntakeEnd3;
    private PathChain Depo4, Gate;

    public static double DEPOSIT_TARGET_VELOCITY = 550.0;

    public static double PID_P = 240.0;
    public static double PID_I = 0.0;
    public static double PID_D = 0.0;
    public static double PID_F = 24.7;

    private double deposit1Vel = 0;
    private double deposit2Vel = 0;
    private double avgDepositVel = 0;
    private double minVelDuringShooting = Double.MAX_VALUE;

    public static double SERVO_CLOSED_POSITION = 0.28;
    public static double SERVO_OPEN_POSITION = 0.8;

    public static long AUTO_SHOOT_DURATION_MS = 2100;
    public static long AUTO_SHOOT_RECOVERY_MS = 0;
    public static long PRE_SHOOT_DELAY_MS = 80;

    // Delay before opening servo after intaking finishes
    // Prevents balls from being pushed up through motors too early
    public static long PRE_OPEN_DELAY_MS = 400;

    public static double INTAKE1_POWER = 1.0;
    public static double INTAKE2_POWER = -1.0;

    // Path timeout configuration
    public static long PATH_TIMEOUT_MS = 2000; // 2 second timeout per path

    public static double RED_START_X = 120.741;
    public static double RED_START_Y = 127.624;
    public static double RED_START_HEADING_DEG = 35.0;

    public static double BLUE_START_X = 22.903;
    public static double BLUE_START_Y = 128.515;
    public static double BLUE_START_HEADING_DEG = 145.0;

    private long shootHoldEndMs = 0;
    private long preShootEndMs = 0;
    private boolean shooting = false;
    private boolean shootingStarted = false;

    // Pre-open delay state
    private boolean waitingToOpen = false;
    private long openServoAtMs = 0;
    private int stateAfterOpen = 0;
    private PathChain pathAfterOpen = null;

    // Path timeout tracking
    private long pathStartTimeMs = 0;
    private boolean pathTimedOut = false;

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        follower = Constants.createFollower(hardwareMap);
        panelsTelemetry.debug("Status", "Initialized - Select Team (X=BLUE, A=RED)");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void start() {
        robot = new TrowelHardware(hardwareMap);

        initDepositMotors();
        setDepositVelocity(DEPOSIT_TARGET_VELOCITY);

        robot.initTransferServo();
        stopIntakes();
        setServoClosed();

        Pose startPose;
        if (selectedTeam == Team.BLUE) {
            BluePaths.Paths blue = new BluePaths.Paths(follower);
            loadPaths(blue);
            startPose = new Pose(BLUE_START_X, BLUE_START_Y, Math.toRadians(BLUE_START_HEADING_DEG));
            panelsTelemetry.debug("Team Selected", "BLUE");
        } else {
            RedPaths.Paths red = new RedPaths.Paths(follower);
            loadPaths(red);
            startPose = new Pose(RED_START_X, RED_START_Y, Math.toRadians(RED_START_HEADING_DEG));
            panelsTelemetry.debug("Team Selected", "RED");
        }

        follower.setStartingPose(startPose);
        pathState = 0;
        waitingToOpen = false;

        Constants.setMotorPowerMultiplier("LEFT_FRONT", Constants.LEFT_FRONT_POWER);
        Constants.setMotorPowerMultiplier("LEFT_REAR", Constants.LEFT_REAR_POWER);
        Constants.setMotorPowerMultiplier("RIGHT_FRONT", Constants.RIGHT_FRONT_POWER);
        Constants.setMotorPowerMultiplier("RIGHT_REAR", Constants.RIGHT_REAR_POWER);

        panelsTelemetry.update(telemetry);
    }

    private void loadPaths(RedPaths.Paths p) {
        Depo1 = p.Depo1;  IntakeStart1 = p.IntakeStart1;  IntakeEnd1 = p.IntakeEnd1;
        Depo2 = p.Depo2;  IntakeStart2 = p.IntakeStart2;  IntakeEnd2 = p.IntakeEnd2;
        Depo3 = p.Depo3;  IntakeStart3 = p.IntakeStart3;  IntakeEnd3 = p.IntakeEnd3;
        Depo4 = p.Depo4;  Gate = p.Gate;
    }

    private void loadPaths(BluePaths.Paths p) {
        Depo1 = p.Depo1;  IntakeStart1 = p.IntakeStart1;  IntakeEnd1 = p.IntakeEnd1;
        Depo2 = p.Depo2;  IntakeStart2 = p.IntakeStart2;  IntakeEnd2 = p.IntakeEnd2;
        Depo3 = p.Depo3;  IntakeStart3 = p.IntakeStart3;  IntakeEnd3 = p.IntakeEnd3;
        Depo4 = p.Depo4;  Gate = p.Gate;
    }

    @Override
    public void init_loop() {
        if (gamepad1.x) selectedTeam = Team.BLUE;
        else if (gamepad1.a) selectedTeam = Team.RED;

        telemetry.addLine("=== TEAM SELECTION ===");
        switch (selectedTeam) {
            case BLUE:
                telemetry.addData("Selected Team", "BLUE");
                telemetry.addData("Start", String.format(Locale.US, "%.1f, %.1f, %.0f°",
                        BLUE_START_X, BLUE_START_Y, BLUE_START_HEADING_DEG));
                break;
            case RED:
                telemetry.addData("Selected Team", "RED");
                telemetry.addData("Start", String.format(Locale.US, "%.1f, %.1f, %.0f°",
                        RED_START_X, RED_START_Y, RED_START_HEADING_DEG));
                break;
            default:
                telemetry.addData("Selected Team", "NONE - Please select!");
                telemetry.addLine("Press X for BLUE | Press A for RED");
                break;
        }
        telemetry.addLine("");
        telemetry.addData("Servo", String.format(Locale.US, "closed=%.2f open=%.2f", SERVO_CLOSED_POSITION, SERVO_OPEN_POSITION));
        telemetry.addData("Shoot", AUTO_SHOOT_DURATION_MS + "ms (delay " + PRE_SHOOT_DELAY_MS + "ms)");
        telemetry.addData("Pre-Open Delay", PRE_OPEN_DELAY_MS + "ms");
        telemetry.addData("PIDF", String.format(Locale.US, "P=%.0f I=%.0f D=%.0f F=%.1f", PID_P, PID_I, PID_D, PID_F));
        telemetry.addData("Deposit Target", DEPOSIT_TARGET_VELOCITY);
        telemetry.addData("Intake Powers", String.format(Locale.US, "I1=%.1f I2=%.1f", INTAKE1_POWER, INTAKE2_POWER));
        telemetry.addLine("\nPress START when ready");
        telemetry.update();
    }

    @Override
    public void loop() {
        follower.update();
        readDepositVelocities();
        setDepositVelocity(DEPOSIT_TARGET_VELOCITY);

        if (shooting && shootingStarted) {
            if (avgDepositVel < minVelDuringShooting) {
                minVelDuringShooting = avgDepositVel;
            }
        }

        // Handle pre-open delay
        if (waitingToOpen && System.currentTimeMillis() >= openServoAtMs) {
            setServoOpen();
            follower.followPath(pathAfterOpen);
            startPathTimer();  // Start the timer for the new path
            pathState = stateAfterOpen;
            waitingToOpen = false;
        }

        pathState = autonomousPathUpdate();
        updateTelemetry();
    }

    @Override
    public void stop() {
        if (robot != null) {
            stopIntakes();
            stopDeposit();
            robot.stop();
        }
    }

    // ══════════════════════════════════════════════════════════════
    // DEPOSIT MOTORS
    // ══════════════════════════════════════════════════════════════

    private void initDepositMotors() {
        try {
            if (robot.deposit1 != null) {
                robot.deposit1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.deposit1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.deposit1.setVelocityPIDFCoefficients(PID_P, PID_I, PID_D, PID_F);
                robot.deposit1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            }
            if (robot.deposit2 != null) {
                robot.deposit2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.deposit2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.deposit2.setVelocityPIDFCoefficients(PID_P, PID_I, PID_D, PID_F);
                robot.deposit2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            }
        } catch (Exception e) {
            panelsTelemetry.debug("Deposit Init Error", e.getMessage());
        }
    }

    private void setDepositVelocity(double velocity) {
        try {
            if (robot.deposit1 != null) robot.deposit1.setVelocity(velocity);
            if (robot.deposit2 != null) robot.deposit2.setVelocity(velocity);
        } catch (Exception ignored) {}
    }

    private void readDepositVelocities() {
        try {
            if (robot.deposit1 != null) deposit1Vel = Math.abs(robot.deposit1.getVelocity());
            if (robot.deposit2 != null) deposit2Vel = Math.abs(robot.deposit2.getVelocity());
            avgDepositVel = (deposit1Vel + deposit2Vel) / 2.0;
        } catch (Exception ignored) {}
    }

    private void stopDeposit() {
        try {
            if (robot.deposit1 != null) robot.deposit1.setPower(0);
            if (robot.deposit2 != null) robot.deposit2.setPower(0);
        } catch (Exception ignored) {}
    }

    // ══════════════════════════════════════════════════════════════
    // SERVO
    // ══════════════════════════════════════════════════════════════

    private void setServoOpen() {
        if (robot.transferServo != null) robot.transferServo.setPosition(SERVO_OPEN_POSITION);
    }

    private void setServoClosed() {
        if (robot.transferServo != null) robot.transferServo.setPosition(SERVO_CLOSED_POSITION);
    }

    /**
     * Schedule servo open after a delay, then follow a path.
     * Stops intakes immediately, waits PRE_OPEN_DELAY_MS,
     * then opens servo and starts the depo path.
     */
    private void scheduleOpenAndFollow(PathChain depoPath, int nextState) {
        stopIntakes();
        waitingToOpen = true;
        openServoAtMs = System.currentTimeMillis() + PRE_OPEN_DELAY_MS;
        pathAfterOpen = depoPath;
        stateAfterOpen = nextState;
    }

    // ══════════════════════════════════════════════════════════════
    // INTAKES — Both always run together, both always stop together
    // ══════════════════════════════════════════════════════════════

    private void startIntakes() {
        if (robot.intake1 != null) robot.intake1.setPower(INTAKE1_POWER);
        if (robot.intake2 != null) robot.intake2.setPower(INTAKE2_POWER);
    }

    private void stopIntakes() {
        if (robot.intake1 != null) robot.intake1.setPower(0.0);
        if (robot.intake2 != null) robot.intake2.setPower(0.0);
    }

    // ══════════════════════════════════════════════════════════════
    // PATH TIMEOUT
    // ══════════════════════════════════════════════════════════════

    /**
     * Start tracking time for the current path.
     * Call this whenever follower.followPath() is called.
     */
    private void startPathTimer() {
        pathStartTimeMs = System.currentTimeMillis();
        pathTimedOut = false;
    }

    /**
     * Check if the current path has exceeded the timeout.
     * Returns true if timeout exceeded, false otherwise.
     */
    private boolean isPathTimedOut() {
        if (pathTimedOut) return true;
        if (System.currentTimeMillis() - pathStartTimeMs > PATH_TIMEOUT_MS) {
            pathTimedOut = true;
            panelsTelemetry.debug("TIMEOUT", "Path exceeded " + PATH_TIMEOUT_MS + "ms");
            return true;
        }
        return false;
    }

    /**
     * Check if path is done (either reached target or timed out).
     */
    private boolean isPathDone() {
        return !follower.isBusy() || isPathTimedOut();
    }

    // ══════════════════════════════════════════════════════════════
    // SHOOTING
    // ══════════════════════════════════════════════════════════════

    private void startShooting() {
        shooting = true;
        shootingStarted = false;
        minVelDuringShooting = Double.MAX_VALUE;
        long now = System.currentTimeMillis();
        preShootEndMs = now + PRE_SHOOT_DELAY_MS;
        shootHoldEndMs = preShootEndMs + AUTO_SHOOT_DURATION_MS;
        setServoOpen();
    }

    private boolean updateShooting() {
        if (!shooting) return false;
        long now = System.currentTimeMillis();

        if (!shootingStarted && now >= preShootEndMs) {
            startIntakes();
            shootingStarted = true;
        }

        if (shootingStarted && now >= shootHoldEndMs) {
            shooting = false;
            shootingStarted = false;
            stopIntakes();
            setServoClosed();

            double minVelPercent = (minVelDuringShooting / DEPOSIT_TARGET_VELOCITY) * 100;
            panelsTelemetry.debug("Shoot", String.format(Locale.US,
                    "Done. Min vel: %.0f (%.0f%%)", minVelDuringShooting, minVelPercent));

            if (AUTO_SHOOT_RECOVERY_MS > 0) {
                try { Thread.sleep(AUTO_SHOOT_RECOVERY_MS); } catch (InterruptedException ignored) {}
            }
            return true;
        }
        return false;
    }

    // ══════════════════════════════════════════════════════════════
    // TELEMETRY
    // ══════════════════════════════════════════════════════════════

    private void updateTelemetry() {
        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", Math.toDegrees(follower.getPose().getHeading()));
        panelsTelemetry.debug("Deposit Avg", String.format(Locale.US, "%.1f", avgDepositVel));
        panelsTelemetry.debug("Deposit %", String.format(Locale.US, "%.1f%%",
                (avgDepositVel / DEPOSIT_TARGET_VELOCITY) * 100));

        // Display path timer
        long elapsedMs = System.currentTimeMillis() - pathStartTimeMs;
        if (follower.isBusy()) {
            panelsTelemetry.debug("Path Timer", String.format(Locale.US, "%.1fs / %.1fs",
                    elapsedMs / 1000.0, PATH_TIMEOUT_MS / 1000.0));
        }

        if (shooting) {
            panelsTelemetry.debug("Shooting", "ACTIVE (" +
                    (shootHoldEndMs - System.currentTimeMillis()) + "ms left)");
        }
        if (waitingToOpen) {
            panelsTelemetry.debug("Waiting", "Pre-open delay (" +
                    (openServoAtMs - System.currentTimeMillis()) + "ms left)");
        }

        panelsTelemetry.update(telemetry);

        telemetry.addLine("═══════════════════════════════");
        telemetry.addData("Team", selectedTeam.toString());
        telemetry.addData("State", pathState);
        telemetry.addData("Pos", String.format(Locale.US, "%.1f, %.1f  hdg %.1f°",
                follower.getPose().getX(), follower.getPose().getY(),
                Math.toDegrees(follower.getPose().getHeading())));
        telemetry.addData("Deposit", String.format(Locale.US, "%.0f / %.0f (%.0f%%)",
                avgDepositVel, DEPOSIT_TARGET_VELOCITY,
                (avgDepositVel / DEPOSIT_TARGET_VELOCITY) * 100));

        // Display path timeout status
        if (follower.isBusy()) {
            long elapsed = System.currentTimeMillis() - pathStartTimeMs;
            telemetry.addData("Path Timer", String.format(Locale.US, "%.1fs / %.1fs",
                    elapsed / 1000.0, PATH_TIMEOUT_MS / 1000.0));
        }

        if (shooting) {
            telemetry.addData("SHOOTING", (shootHoldEndMs - System.currentTimeMillis()) + "ms left");
        }
        if (waitingToOpen) {
            telemetry.addData("WAITING", "Pre-open " + (openServoAtMs - System.currentTimeMillis()) + "ms");
        }
        telemetry.update();
    }

    // ══════════════════════════════════════════════════════════════
    // STATE MACHINE
    // ══════════════════════════════════════════════════════════════

    public int autonomousPathUpdate() {
        // Don't process state transitions while waiting for pre-open delay
        if (waitingToOpen) return pathState;

        switch (pathState) {

            // ── CYCLE 1 ──
            case 0:
                setServoOpen();
                follower.followPath(Depo1);
                startPathTimer();
                pathState = 1;
                break;
            case 1:
                if (isPathDone()) {
                    startShooting();
                    pathState = 14;
                }
                break;
            case 14:
                if (updateShooting()) {
                    startIntakes();
                    setServoClosed();
                    follower.followPath(IntakeStart1);
                    startPathTimer();
                    pathState = 2;
                }
                break;
            case 2:
                if (isPathDone()) {
                    follower.followPath(IntakeEnd1);
                    startPathTimer();
                    pathState = 3;
                }
                break;

            // ── CYCLE 2 ──
            case 3:
                if (isPathDone()) {
                    // Stop intakes, wait, then open servo and follow Depo2
                    scheduleOpenAndFollow(Depo2, 4);
                }
                break;
            case 4:
                if (isPathDone()) {
                    startShooting();
                    pathState = 15;
                }
                break;
            case 15:
                if (updateShooting()) {
                    startIntakes();
                    setServoClosed();
                    follower.followPath(IntakeStart2);
                    startPathTimer();
                    pathState = 5;
                }
                break;
            case 5:
                if (isPathDone()) {
                    follower.followPath(IntakeEnd2);
                    startPathTimer();
                    pathState = 6;
                }
                break;

            // ── CYCLE 3 ──
            case 6:
                if (isPathDone()) {
                    scheduleOpenAndFollow(Depo3, 7);
                }
                break;
            case 7:
                if (isPathDone()) {
                    startShooting();
                    pathState = 16;
                }
                break;
            case 16:
                if (updateShooting()) {
                    startIntakes();
                    setServoClosed();
                    follower.followPath(IntakeStart3);
                    startPathTimer();
                    pathState = 8;
                }
                break;
            case 8:
                if (isPathDone()) {
                    follower.followPath(IntakeEnd3);
                    startPathTimer();
                    pathState = 9;
                }
                break;

            // ── CYCLE 4 ──
            case 9:
                if (isPathDone()) {
                    scheduleOpenAndFollow(Depo4, 10);
                }
                break;
            case 10:
                if (isPathDone()) {
                    startShooting();
                    pathState = 17;
                }
                break;
            case 17:
                if (updateShooting()) {
                    follower.followPath(Gate);
                    startPathTimer();
                    pathState = 11;
                }
                break;

            // ── DONE ──
            case 11:
                if (isPathDone()) {
                    pathState = 12;
                }
                break;
            case 12:
                break;
        }
        return pathState;
    }
}