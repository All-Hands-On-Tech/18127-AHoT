package org.firstinspires.ftc.teamcode.Trowel.Autonomous;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.Trowel.Configs.TrowelHardware;
import org.firstinspires.ftc.teamcode.Trowel.pedroPathing.Constants;

@Autonomous(name = "Trowel Auto", group = "Autonomous")
@Configurable
public class StraightAuto extends OpMode {

    public enum Team { NONE, RED, BLUE }
    private Team selectedTeam = Team.NONE;

    private TelemetryManager panelsTelemetry;
    public Follower follower;
    private int pathState = 0;
    private Paths paths;
    private ElapsedTime timer;
    private TrowelHardware robot;

    // Pedro field is 144x144; used for alliance mirroring
    private static final double FIELD_SIZE = 144.0;

    // Auto shooting config (mirrors teleop feedforward + spinup style)
    public static double AUTO_DEPOSIT_VELOCITY = 165.0; // ticks/s target during auto shooting (reduced to curb overshoot)
    public static double AUTO_DEPOSIT_FF_FACTOR = 0.05;
    public static double AUTO_DEPOSIT_FF_BOOST_TICKS = 241.0;
    private static final double AUTO_DEPOSIT_SPINUP_POWER = 1.0;
    private static final long AUTO_DEPOSIT_SPINUP_MS = 635;
    public static long AUTO_SHOOT_DURATION_MS = 2100;    // shortened hold to reduce overshoot
    public static long AUTO_SHOOT_RECOVERY_MS = 200;      // optional recovery idle after shot
    public static long PRE_SHOOT_DELAY_MS = 130;          // wait before shooting to stabilize
    private static final double AUTO_INTAKE1_POWER = 1.0;
    private static final double AUTO_INTAKE2_POWER = -1.0;
    private static final double TRANSFER_IN = 0.0;
    private static final double TRANSFER_OUT = -1.0;   // drive transfers only while shooting
    private static final double TRANSFER_NEUTRAL = 0.5; // hold neutral when not shooting
    private static final long INTAKE2_BURST_MS = 1500;
    private long shootHoldEndMs = 0;
    private long shootSpinupEndMs = 0;
    private long intake2OffTimeMs = 1000;
    private long preShootEndMs = 0;
    private boolean shooting = false;
    private boolean shootingSpinup = false;
    private boolean shootingStarted = false;

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        follower = Constants.createFollower(hardwareMap);

        // Telemetry prompt for team selection; paths and pose set in start()
        panelsTelemetry.debug("Status", "Initialized - Select Team (X=BLUE, A=RED)");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void start() {
        // Initialize auxiliary hardware (intakes, deposit, transfer servos)
        robot = new TrowelHardware(hardwareMap);
        robot.resetDepositEncoders();
        robot.initTransferServos();
        robot.setDepositFeedforwardFactor(AUTO_DEPOSIT_FF_FACTOR);
        robot.setDepositFeedforwardBoostTicks(AUTO_DEPOSIT_FF_BOOST_TICKS);
        // Keep deposit running for the entire auto
        robot.setDepositVelocity(AUTO_DEPOSIT_VELOCITY);
        if (robot.intake1 != null) robot.intake1.setPower(0.0);
        if (robot.intake2 != null) robot.intake2.setPower(0.0);
        // Leave transfers neutral until shooting
        setTransferNeutral();
        panelsTelemetry.debug("Aux Init", robot.getInitializationStatus());

        // Apply starting pose based on selected team
        Pose startPose;
        if (selectedTeam == Team.BLUE) {
            startPose = new Pose(26.467, 129.584, Math.toRadians(145));
            panelsTelemetry.debug("Team Selected", "BLUE");
            panelsTelemetry.debug("StartPose", "BLUE (26.467, 129.584, 145°)");
        } else {
            startPose = new Pose(117.533, 129.584, Math.toRadians(35));
            panelsTelemetry.debug("Team Selected", "RED");
            panelsTelemetry.debug("StartPose", "RED (117.533, 129.584, 35°)");
        }
        follower.setStartingPose(startPose);

        // Build paths for selected team (red paths mirrored for blue)
        paths = new Paths(follower, selectedTeam);
        pathState = 0;
        timer = new ElapsedTime();

        // Initialize drive motors and add telemetry for their power multipliers
        Constants.setMotorPowerMultiplier("LEFT_FRONT", Constants.LEFT_FRONT_POWER);
        Constants.setMotorPowerMultiplier("LEFT_REAR", Constants.LEFT_REAR_POWER);
        Constants.setMotorPowerMultiplier("RIGHT_FRONT", Constants.RIGHT_FRONT_POWER);
        Constants.setMotorPowerMultiplier("RIGHT_REAR", Constants.RIGHT_REAR_POWER);

        panelsTelemetry.debug("Motor Power Multipliers", "LF: " + Constants.LEFT_FRONT_POWER + ", LR: " + Constants.LEFT_REAR_POWER + ", RF: " + Constants.RIGHT_FRONT_POWER + ", RR: " + Constants.RIGHT_REAR_POWER);
        panelsTelemetry.debug("StartPose X", follower.getPose().getX());
        panelsTelemetry.debug("StartPose Y", follower.getPose().getY());
        panelsTelemetry.debug("StartPose Heading", follower.getPose().getHeading());
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void init_loop() {
        if (gamepad1.x) {
            selectedTeam = Team.BLUE;
        } else if (gamepad1.a) {
            selectedTeam = Team.RED;
        }

        telemetry.addLine("=== TEAM SELECTION ===");
        switch (selectedTeam) {
            case BLUE:
                telemetry.addData("Selected Team", "BLUE");
                break;
            case RED:
                telemetry.addData("Selected Team", "RED");
                break;
            default:
                telemetry.addData("Selected Team", "NONE - Please select!");
                telemetry.addLine("Press X for BLUE team");
                telemetry.addLine("Press A for RED team");
                break;
        }
        telemetry.addLine("");
        telemetry.addLine("Press START when ready");
        telemetry.update();
    }

    @Override
    public void loop() {
        follower.update();
        // Time out intake2 burst after the window
        if (intake2OffTimeMs > 0 && System.currentTimeMillis() >= intake2OffTimeMs) {
            if (robot != null && robot.intake2 != null) robot.intake2.setPower(0.0);
            intake2OffTimeMs = 0;
        }
        pathState = autonomousPathUpdate();

        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", follower.getPose().getHeading());
        if (shooting) {
            panelsTelemetry.debug("Shooting", "ACTIVE (ends in " + (shootHoldEndMs - System.currentTimeMillis()) + "ms)");
        }
        panelsTelemetry.update(telemetry);
    }

    private void startShooting() {
        shooting = true;
        shootingStarted = false;
        shootingSpinup = false; // already running at velocity
        long now = System.currentTimeMillis();
        preShootEndMs = now + PRE_SHOOT_DELAY_MS;
        shootSpinupEndMs = preShootEndMs; // keep legacy naming; we don't use spinup delay
        shootHoldEndMs = preShootEndMs + AUTO_SHOOT_DURATION_MS;
        // Ensure deposit is at target velocity (already spinning in start())
        robot.setDepositVelocity(AUTO_DEPOSIT_VELOCITY);
        panelsTelemetry.debug("Shoot", "Waiting " + PRE_SHOOT_DELAY_MS + "ms then holding for " + AUTO_SHOOT_DURATION_MS + "ms");
    }

    private boolean updateShooting() {
        if (!shooting) return false;
        long now = System.currentTimeMillis();

        // Wait for pre-shoot settle before starting intakes/transfers
        if (!shootingStarted && now >= preShootEndMs) {
            if (robot.intake1 != null) robot.intake1.setPower(AUTO_INTAKE1_POWER);
            if (robot.intake2 != null) robot.intake2.setPower(AUTO_INTAKE2_POWER);
            setTransferOut();
            shootingStarted = true;
        }

        // Keep deposit spinning during entire auto; no stop after shooting
        if (shootingStarted && now >= shootHoldEndMs) {
            shooting = false;
            shootingSpinup = false;
            shootingStarted = false;
            // stop intakes after shooting, but leave deposit running
            if (robot.intake1 != null) robot.intake1.setPower(0.0);
            if (robot.intake2 != null) robot.intake2.setPower(0.0);
            intake2OffTimeMs = 0;
            setTransferNeutral();
            panelsTelemetry.debug("Shoot", "Completed");
            // optional recovery pause before next sequence
            if (AUTO_SHOOT_RECOVERY_MS > 0) {
                try { Thread.sleep(AUTO_SHOOT_RECOVERY_MS); } catch (InterruptedException ignored) {}
            }
            return true;
        }
        return false;
    }

    public class Paths {
        public PathChain Depo1;
        public PathChain IntakeStart1;
        public PathChain IntakeEnd1;
        public PathChain Depo2;
        public PathChain IntakeStart2;
        public PathChain IntakeEnd2;
        public PathChain Depo3;
        public PathChain IntakeStart3;
        public PathChain IntakeEnd3;
        public PathChain Depo4;
        public PathChain Gate2;
        public PathChain line12;

        public Paths(Follower follower, Team team) {
            if (team == Team.BLUE) {
                buildBluePaths(follower);
            } else {
                buildRedPaths(follower);
            }
        }

        private void buildRedPaths(Follower follower) {
            Depo1 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(117.533, 129.584),
                                    new Pose(86.121, 88.138)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(35), Math.toRadians(45))
                    .build();
            IntakeStart1 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(86.121, 88.138),
                                    new Pose(83.813, 68.495),
                                    new Pose(99.924, 85.788)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(180))
                    .build();
            IntakeEnd1 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(99.924, 85.788),
                                    new Pose(126.609, 86.823)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();
            Depo2 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(126.609, 86.823),
                                    new Pose(86.190, 88.414)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(45))
                    .build();
            IntakeStart2 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(86.190, 88.414),
                                    new Pose(84.724, 83.552),
                                    new Pose(101.241, 59.207)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(180))
                    .build();
            IntakeEnd2 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(101.241, 59.207),
                                    new Pose(135.495, 59.092)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();
            Depo3 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(135.495, 59.092),
                                    new Pose(86.017, 88.466)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(45))
                    .build();
            IntakeStart3 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(86.017, 88.466),
                                    new Pose(101.977, 38.271)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(180))
                    .build();
            IntakeEnd3 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(101.977, 38.271),
                                    new Pose(134.225, 38.294)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();
            Depo4 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(134.225, 38.294),
                                    new Pose(86.138, 88.517)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(45))
                    .build();
            Gate2 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(86.138, 88.517),
                                    new Pose(117.860, 72.035)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(90))
                    .build();
        }

        private void buildBluePaths(Follower follower) {
            Depo1 = follower.pathBuilder().addPath(
                new BezierLine(
                    new Pose(26.467, 129.584),
                    new Pose(57.879, 88.138)
                )
            ).setLinearHeadingInterpolation(Math.toRadians(145), Math.toRadians(135))
             .build();
            IntakeStart1 = follower.pathBuilder().addPath(
                new BezierCurve(
                    new Pose(57.879, 88.138),
                    new Pose(60.187, 68.495),
                    new Pose(44.076, 85.788)
                )
            ).setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(0))
             .build();
            IntakeEnd1 = follower.pathBuilder().addPath(
                new BezierLine(
                    new Pose(44.076, 85.788),
                    new Pose(17.391, 86.823)
                )
            ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
             .build();
            Depo2 = follower.pathBuilder().addPath(
                new BezierLine(
                    new Pose(17.391, 86.823),
                    new Pose(57.810, 88.414)
                )
            ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(135))
             .build();
            IntakeStart2 = follower.pathBuilder().addPath(
                new BezierCurve(
                    new Pose(57.810, 88.414),
                    new Pose(59.276, 83.552),
                    new Pose(42.759, 59.207)
                )
            ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
             .build();
            IntakeEnd2 = follower.pathBuilder().addPath(
                new BezierLine(
                    new Pose(42.759, 59.207),
                    new Pose(8.505, 59.092)
                )
            ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
             .build();
            Depo3 = follower.pathBuilder().addPath(
                new BezierLine(
                    new Pose(8.505, 59.092),
                    new Pose(57.983, 88.466)
                )
            ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(135))
             .build();
            IntakeStart3 = follower.pathBuilder().addPath(
                new BezierLine(
                    new Pose(57.983, 88.466),
                    new Pose(42.023, 38.271)
                )
            ).setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(0))
             .build();
            IntakeEnd3 = follower.pathBuilder().addPath(
                new BezierLine(
                    new Pose(42.023, 38.271),
                    new Pose(9.775, 38.294)
                )
            ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
             .build();
            Depo4 = follower.pathBuilder().addPath(
                new BezierLine(
                    new Pose(9.775, 38.294),
                    new Pose(57.862, 88.517)
                )
            ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(135))
             .build();
            Gate2 = follower.pathBuilder().addPath(
                new BezierLine(
                    new Pose(57.862, 88.517),
                    new Pose(26.140, 72.035)
                )
            ).setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(90))
             .build();
            line12 = follower.pathBuilder().addPath(
                new BezierLine(
                    new Pose(26.140, 72.035),
                    new Pose(96.594, 115.129)
                )
            ).setTangentHeadingInterpolation()
             .build();
        }
    }

    // Update the state machine to follow each path in sequence
    public int autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(paths.Depo1);
                pathState = 1;
                panelsTelemetry.debug("Transition", "Started Depo1");
                break;
            case 1:
                if (!follower.isBusy()) {
                    startShooting();
                    pathState = 14; // Shoot after Depo1
                    panelsTelemetry.debug("Transition", "Shooting after Depo1");
                }
                break;
            case 14: // Wait shooting after Depo1
                if (updateShooting()) {
                    startIntakeSequence();
                    follower.followPath(paths.IntakeStart1);
                    pathState = 2;
                    panelsTelemetry.debug("Transition", "Started IntakeStart");
                }
                break;
            case 2:
                if (!follower.isBusy()) {
                    follower.followPath(paths.IntakeEnd1);
                    pathState = 3;
                    panelsTelemetry.debug("Transition", "Started IntakeEnd");
                }
                break;
            case 3:
                if (!follower.isBusy()) {
                    stopIntakes();
                    follower.followPath(paths.Depo2);
                    pathState = 4;
                    panelsTelemetry.debug("Transition", "Started Depo2");
                }
                break;
            case 4:
                if (!follower.isBusy()) {
                    startShooting();
                    pathState = 15; // Shoot after Depo2
                    panelsTelemetry.debug("Transition", "Shooting after Depo2");
                }
                break;
            case 15:
                if (updateShooting()) {
                    startIntakeSequence();
                    follower.followPath(paths.IntakeStart2);
                    pathState = 6;
                    panelsTelemetry.debug("Transition", "Started IntakeStart2");
                }
                break;
            case 5:
                // This case is skipped now since we go directly from case 15 to case 6
                break;
            case 6:
                if (!follower.isBusy()) {
                    follower.followPath(paths.IntakeEnd2);
                    pathState = 7;
                    panelsTelemetry.debug("Transition", "Started IntakeEnd2");
                }
                break;
            case 7:
                if (!follower.isBusy()) {
                    stopIntakes();
                    follower.followPath(paths.Depo3);
                    pathState = 8;
                    panelsTelemetry.debug("Transition", "Started Depo3");
                }
                break;
            case 8:
                if (!follower.isBusy()) {
                    startShooting();
                    pathState = 16; // Shoot after Depo3
                    panelsTelemetry.debug("Transition", "Shooting after Depo3");
                }
                break;
            case 16:
                if (updateShooting()) {
                    startIntakeSequence();
                    follower.followPath(paths.IntakeStart3);
                    pathState = 9;
                    panelsTelemetry.debug("Transition", "Started IntakeStart3");
                }
                break;
            case 9:
                if (!follower.isBusy()) {
                    follower.followPath(paths.IntakeEnd3);
                    pathState = 10;
                    panelsTelemetry.debug("Transition", "Started IntakeEnd3");
                }
                break;
            case 10:
                if (!follower.isBusy()) {
                    stopIntakes();
                    follower.followPath(paths.Depo4);
                    pathState = 11;
                    panelsTelemetry.debug("Transition", "Started Depo4");
                }
                break;
            case 11:
                if (!follower.isBusy()) {
                    startShooting();
                    pathState = 17; // Shoot after Depo4
                    panelsTelemetry.debug("Transition", "Shooting after Depo4");
                }
                break;
            case 17:
                if (updateShooting()) {
                    follower.followPath(paths.Gate2);
                    pathState = 12;
                    panelsTelemetry.debug("Transition", "Started Gate2");
                }
                break;
            case 12:
                if (!follower.isBusy()) {
                    pathState = 13; // finished
                    panelsTelemetry.debug("Transition", "Finished Gate2");
                }
                break;
            case 13:
                // finished - keep idle
                break;
        }
        return pathState;
    }

    private void setTransferOut() {
        try {
            if (robot.transfer1 instanceof com.qualcomm.robotcore.hardware.CRServo) {
                ((com.qualcomm.robotcore.hardware.CRServo) robot.transfer1).setPower(1.0);
            } else if (robot.transfer1 != null) {
                robot.transfer1.setPosition(TRANSFER_OUT);
            }
        } catch (Exception ignored) {}
        try {
            if (robot.transfer2 instanceof com.qualcomm.robotcore.hardware.CRServo) {
                ((com.qualcomm.robotcore.hardware.CRServo) robot.transfer2).setPower(1.0);
            } else if (robot.transfer2 != null) {
                robot.transfer2.setPosition(TRANSFER_OUT);
            }
        } catch (Exception ignored) {}
    }

    private void setTransferNeutral() {
        try {
            if (robot.transfer1 instanceof com.qualcomm.robotcore.hardware.CRServo) {
                ((com.qualcomm.robotcore.hardware.CRServo) robot.transfer1).setPower(0.0);
            } else if (robot.transfer1 != null) {
                robot.transfer1.setPosition(TRANSFER_NEUTRAL);
            }
        } catch (Exception ignored) {}
        try {
            if (robot.transfer2 instanceof com.qualcomm.robotcore.hardware.CRServo) {
                ((com.qualcomm.robotcore.hardware.CRServo) robot.transfer2).setPower(0.0);
            } else if (robot.transfer2 != null) {
                robot.transfer2.setPosition(TRANSFER_NEUTRAL);
            }
        } catch (Exception ignored) {}
    }

    private void setTransferIn() {
        try {
            if (robot.transfer1 instanceof com.qualcomm.robotcore.hardware.CRServo) {
                ((com.qualcomm.robotcore.hardware.CRServo) robot.transfer1).setPower(-1.0);
            } else if (robot.transfer1 != null) {
                robot.transfer1.setPosition(TRANSFER_IN);
            }
        } catch (Exception ignored) {}
        try {
            if (robot.transfer2 instanceof com.qualcomm.robotcore.hardware.CRServo) {
                ((com.qualcomm.robotcore.hardware.CRServo) robot.transfer2).setPower(-1.0);
            } else if (robot.transfer2 != null) {
                robot.transfer2.setPosition(TRANSFER_IN);
            }
        } catch (Exception ignored) {}
    }

    private void startIntakeSequence() {
        if (robot.intake1 != null) robot.intake1.setPower(AUTO_INTAKE1_POWER);
        if (robot.intake2 != null) robot.intake2.setPower(AUTO_INTAKE2_POWER);
        intake2OffTimeMs = System.currentTimeMillis() + INTAKE2_BURST_MS;
    }

    private void stopIntakes() {
        if (robot.intake1 != null) robot.intake1.setPower(0.0);
        if (robot.intake2 != null) robot.intake2.setPower(0.0);
        intake2OffTimeMs = 0;
    }
}
