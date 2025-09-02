package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "Advanced Odometry Autonomous", group = "Autonomous")
public class BasicOdometryAutonomous extends LinearOpMode {

    // Drive motors
    private DcMotor frontLeft, frontRight, backLeft, backRight;
    // Odometry encoders (parallel/forward and perpendicular/strafe)
    private DcMotor odoParallel, odoPerpendicular;
    // IMU for heading
    private IMU imu;

    // Localization variables (position and heading of the robot)
    private double x = 0, y = 0, heading = 0;
    private int prevParallel = 0, prevPerpendicular = 0;

    // Constants (adjust for your robot)
    private static final double TICKS_PER_REV = 8192; // REV Through Bore Encoder
    private static final double WHEEL_DIAMETER_MM = 35.0; // Diameter of odometry wheels in mm

    // --- PID Controller Constants (THESE WILL NEED TO BE TUNED) ---
    // Drive PID
    private static final double DRIVE_KP = 0.005;
    private static final double DRIVE_KI = 0.0;
    private static final double DRIVE_KD = 0.0;
    // Strafe PID
    private static final double STRAFE_KP = 0.01;
    private static final double STRAFE_KI = 0.0;
    private static final double STRAFE_KD = 0.0;
    // Turn PID
    private static final double TURN_KP = 0.1;
    private static final double TURN_KI = 0.0;
    private static final double TURN_KD = 0.0;

    @Override
    public void runOpMode() throws InterruptedException {
        // Hardware mapping
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");

        // Configure odometry wheels. Ensure these names match your robot configuration.
        odoParallel = hardwareMap.get(DcMotor.class, "odoParallel");
        odoPerpendicular = hardwareMap.get(DcMotor.class, "odoPerpendicular");

        // Retrieve and initialize the IMU
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        imu.initialize(parameters);

        // Set motor directions
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);

        resetOdometryEncoders();

        waitForStart();

        // Example: Drive to X=0, Y=500mm, with a heading of 0 degrees
        goToPosition(0, 500, 0, 0.7);

        // Example of a more complex path:
        // goToPosition(300, 500, Math.toRadians(90), 0.7);
        // goToPosition(300, 1000, Math.toRadians(90), 0.7);

        // You can add more movements here.
    }

    /**
     * This is the main drive controller. It uses PID controllers and Mecanum Inverse Kinematics
     * to drive the robot to a target X, Y, and heading on the field.
     */
    private void goToPosition(double targetX, double targetY, double targetHeading, double powerCap) {
        // PID state variables
        double driveIntegral = 0, strafeIntegral = 0, turnIntegral = 0;
        double driveLastError = 0, strafeLastError = 0, turnLastError = 0;

        ElapsedTime pidTimer = new ElapsedTime();

        // Loop until the robot is within a tolerance of the target.
        while (opModeIsActive() && (Math.abs(targetX - x) > 10 || Math.abs(targetY - y) > 10 || Math.abs(targetHeading - heading) > Math.toRadians(2))) {
            updateOdometry(); // This is our Localization step

            double dt = pidTimer.seconds();
            pidTimer.reset();

            // Calculate errors in the robot's local coordinate frame
            double dx = targetX - x;
            double dy = targetY - y;

            double driveError = dx * Math.cos(heading) + dy * Math.sin(heading);
            double strafeError = -dx * Math.sin(heading) + dy * Math.cos(heading);
            double turnError = targetHeading - heading;
            // Normalize angle to be between -PI and PI
            while (turnError > Math.PI) turnError -= 2 * Math.PI;
            while (turnError < -Math.PI) turnError += 2 * Math.PI;

            // --- PID Calculations ---
            // Integral term
            driveIntegral += driveError * dt;
            strafeIntegral += strafeError * dt;
            turnIntegral += turnError * dt;

            // Derivative term
            double driveDerivative = (driveError - driveLastError) / dt;
            double strafeDerivative = (strafeError - strafeLastError) / dt;
            double turnDerivative = (turnError - turnLastError) / dt;

            // Proportional + Integral + Derivative
            double drivePower = (driveError * DRIVE_KP) + (driveIntegral * DRIVE_KI) + (driveDerivative * DRIVE_KD);
            double strafePower = (strafeError * STRAFE_KP) + (strafeIntegral * STRAFE_KI) + (strafeDerivative * STRAFE_KD);
            double turnPower = (turnError * TURN_KP) + (turnIntegral * TURN_KI) + (turnDerivative * TURN_KD);

            driveLastError = driveError;
            strafeLastError = strafeError;
            turnLastError = turnError;

            // --- Mecanum Inverse Kinematics ---
            // Combine powers from the three controllers to get individual wheel powers
            double frontLeftPower = drivePower + strafePower + turnPower;
            double backLeftPower = drivePower - strafePower + turnPower;
            double frontRightPower = drivePower - strafePower - turnPower;
            double backRightPower = drivePower + strafePower - turnPower;

            // Normalize powers to be within [-powerCap, powerCap]
            double max = Math.max(Math.abs(frontLeftPower), Math.abs(backLeftPower));
            max = Math.max(max, Math.abs(frontRightPower));
            max = Math.max(max, Math.abs(backRightPower));

            if (max > powerCap) {
                frontLeftPower /= (max / powerCap);
                backLeftPower /= (max / powerCap);
                frontRightPower /= (max / powerCap);
                backRightPower /= (max / powerCap);
            }

            setDrivePower(frontLeftPower, backLeftPower, frontRightPower, backRightPower);

            telemetry.addData("X", "%.2f / %.2f", x, targetX);
            telemetry.addData("Y", "%.2f / %.2f", y, targetY);
            telemetry.addData("Heading", "%.2f / %.2f", Math.toDegrees(heading), Math.toDegrees(targetHeading));
            telemetry.update();
        }
        stopDriving();
    }

    /**
     * LOCALIZATION: Updates the robot's position (x, y) and heading on the field.
     * This is "Pinpoint Odometry" using one parallel wheel, one perpendicular wheel, and the IMU.
     */
    private void updateOdometry() {
        // Encoder values might need to be negated depending on wheel orientation
        int currParallel = odoParallel.getCurrentPosition();
        int currPerpendicular = odoPerpendicular.getCurrentPosition();

        int deltaParallel = currParallel - prevParallel;
        int deltaPerpendicular = currPerpendicular - prevPerpendicular;

        double deltaY_robot = (deltaParallel / TICKS_PER_REV) * Math.PI * WHEEL_DIAMETER_MM;
        double deltaX_robot = (deltaPerpendicular / TICKS_PER_REV) * Math.PI * WHEEL_DIAMETER_MM;

        // Get heading from IMU
        heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        // Rotate the robot-centric measurements to the field-centric frame
        double deltaX_field = deltaX_robot * Math.cos(heading) - deltaY_robot * Math.sin(heading);
        double deltaY_field = deltaX_robot * Math.sin(heading) + deltaY_robot * Math.cos(heading);

        // Update global position
        x += deltaX_field;
        y += deltaY_field;

        prevParallel = currParallel;
        prevPerpendicular = currPerpendicular;
    }

    private void resetOdometryEncoders() {
        odoParallel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        odoPerpendicular.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        odoParallel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        odoPerpendicular.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        if (imu != null) {
            imu.resetYaw();
        }

        prevParallel = odoParallel.getCurrentPosition();
        prevPerpendicular = odoPerpendicular.getCurrentPosition();
        x = 0;
        y = 0;
        heading = 0;
    }

    private void setDrivePower(double fl, double bl, double fr, double br) {
        frontLeft.setPower(fl);
        backLeft.setPower(bl);
        frontRight.setPower(fr);
        backRight.setPower(br);
    }

    private void stopDriving() {
        setDrivePower(0, 0, 0, 0);
    }
}
