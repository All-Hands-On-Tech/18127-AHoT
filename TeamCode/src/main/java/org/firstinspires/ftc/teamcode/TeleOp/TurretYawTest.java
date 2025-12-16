package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@TeleOp(name = "Turret IMU RUN_TO_POSITION", group = "TEST")
public class TurretYawTest extends LinearOpMode {

    /* ================== CONSTANTS ================== */

    // Encoder pulses per motor revolution (VERIFY)
    private static final double PULSES_PER_REV = 145.1;

    // Gear reduction: motor : turret = 10 : 64
    private static final double GEAR_REDUCTION = 10.0 / 64.0;

    // Encoder ticks per turret revolution
    private static final double TICKS_PER_TURRET_REV =
            PULSES_PER_REV / GEAR_REDUCTION;

    // Degrees per turret revolution
    private static final double DEGREES_PER_REV = 360.0;

    // Encoder ticks per degree of turret rotation
    private static final double TICKS_PER_DEGREE =
            TICKS_PER_TURRET_REV / DEGREES_PER_REV;

    /* ================== HARDWARE ================== */

    private DcMotor turretMotor;
    private IMU imu;

    /* ================== TARGET ================== */

    // Yaw angle we want the turret to face (degrees)
    // Currently set to initialization position
    private double targetYawDegrees = 0.0;

    @Override
    public void runOpMode() throws InterruptedException {

        /* ---------- Hardware Mapping ---------- */
        turretMotor = hardwareMap.get(DcMotor.class, "turret_motor");
        imu = hardwareMap.get(IMU.class, "imu");

        /* ---------- Motor Setup ---------- */
        turretMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        /* ---------- IMU Init ---------- */
        //FIXME: Verify Orientation
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.FORWARD)));

        telemetry.addLine("Initialized");
        telemetry.update();

        targetYawDegrees = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

        waitForStart();

        // Optional: redefine yaw = 0 at start
        imu.resetYaw();

        /* ---------- Main Loop ---------- */
        while (opModeIsActive()) {

            // Read current robot yaw
            YawPitchRollAngles angles = imu.getRobotYawPitchRollAngles();
            double currentYaw = angles.getYaw(AngleUnit.DEGREES);

            double yawError = currentYaw - targetYawDegrees;

            // Convert yaw error to encoder ticks
            int targetTicks = (int) (targetYawDegrees * TICKS_PER_DEGREE);

            // Set turret target
            turretMotor.setTargetPosition(targetTicks);
            turretMotor.setPower(0.4);  // constant power

            telemetry.addData("Target Yaw (deg)", targetYawDegrees);
            telemetry.addData("Current Yaw (deg)", currentYaw);
            telemetry.addData("Yaw Error (deg)", yawError);
            telemetry.addData("Target Ticks", targetTicks);
            telemetry.addData("Current Ticks", turretMotor.getCurrentPosition());
            telemetry.update();

            idle();
        }
    }
}
