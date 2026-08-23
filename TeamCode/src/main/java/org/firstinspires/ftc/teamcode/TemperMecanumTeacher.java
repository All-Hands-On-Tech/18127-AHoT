package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp(name = "Mecanum Drive Field Centric + Odometry", group = "TeleOp")
public class TemperMecanumTeacher extends LinearOpMode {

	// drive motors
	private DcMotor leftBackDrive;
	private DcMotor rightBackDrive;
	private DcMotor leftFrontDrive;
	private DcMotor rightFrontDrive;

	// intake
	private DcMotor iMotor;
	private CRServo intakeLeft;
	private CRServo intakeRight;

	// dead wheel encoders
	// one points forward/backward and one points sideways
	private DcMotor odoParallel;
	private DcMotor odoPerpendicular;

	// imu for heading
	private BNO055IMU imu;

	// odometry numbers
	// change these to match the robot
	private static final double TICKS_PER_REV = 8192.0;
	private static final double ODO_WHEEL_DIAMETER_INCHES = 1.37795;
	private static final double INCHES_PER_TICK = (ODO_WHEEL_DIAMETER_INCHES * Math.PI) / TICKS_PER_REV;

	// this is how far the sideways wheel is from the center of rotation
	private static final double PERPENDICULAR_OFFSET = 5.0;

	// robot position on the field
	private double robotX = 0.0;
	private double robotY = 0.0;
	private double robotHeading = 0.0;

	// last sensor values so we can track change each loop
	private int prevParallelTicks = 0;
	private int prevPerpendicularTicks = 0;
	private double prevHeading = 0.0;

	// this lets us reset heading during the match
	private double imuHeadingOffset = 0.0;

	@Override
	public void runOpMode() {
		// hardware setup
		leftBackDrive = hardwareMap.get(DcMotor.class, "leftBackDrive");
		rightBackDrive = hardwareMap.get(DcMotor.class, "rightBackDrive");
		iMotor = hardwareMap.get(DcMotor.class, "iMotor");
		leftFrontDrive = hardwareMap.get(DcMotor.class, "leftFrontDrive");
		rightFrontDrive = hardwareMap.get(DcMotor.class, "rightFrontDrive");

		intakeLeft = hardwareMap.get(CRServo.class, "intakeLeft");
		intakeRight = hardwareMap.get(CRServo.class, "intakeRight");

		// these need to match the names you give the odometry encoders
		odoParallel = hardwareMap.get(DcMotor.class, "odoParallel");
		odoPerpendicular = hardwareMap.get(DcMotor.class, "odoPerpendicular");

		// control hub imu
		imu = hardwareMap.get(BNO055IMU.class, "imu");

		// motor directions
		leftBackDrive.setDirection(DcMotorSimple.Direction.REVERSE);
		rightBackDrive.setDirection(DcMotorSimple.Direction.FORWARD);
		leftFrontDrive.setDirection(DcMotorSimple.Direction.FORWARD);
		rightFrontDrive.setDirection(DcMotorSimple.Direction.FORWARD);
		iMotor.setDirection(DcMotorSimple.Direction.FORWARD);

		// braking x
		leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

		// reset the dead wheel encoders
		// we are only reading encoder values from these
		odoParallel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		odoPerpendicular.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		odoParallel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
		odoPerpendicular.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

		// imu setup
		BNO055IMU.Parameters imuParameters = new BNO055IMU.Parameters();
		imuParameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
		imuParameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
		imuParameters.calibrationDataFile = "BNO055IMUCalibration.json";
		imuParameters.loggingEnabled = false;
		imu.initialize(imuParameters);

		// start with intake off
		intakeLeft.setPower(0);
		intakeRight.setPower(0);

		telemetry.addLine("initialized");
		telemetry.addLine("press y to reset heading");
		telemetry.update();

		waitForStart();

		// save the starting values so the first loop has a baseline
		prevParallelTicks = odoParallel.getCurrentPosition();
		prevPerpendicularTicks = odoPerpendicular.getCurrentPosition();
		prevHeading = getIMUHeading();

		while (opModeIsActive()) {
			// update robot position from odometry and imu
			updateOdometry();

			// driver inputs
			// left stick is movement
			// right stick x is turning
			double inputY = -gamepad1.left_stick_y;
			double inputX = gamepad1.left_stick_x;
			double rx = -gamepad1.right_stick_x;

			// get current heading
			double heading = getIMUHeading();

			// rotate the stick input so movement stays field centric
			double rotatedX = inputX * Math.cos(-heading) - inputY * Math.sin(-heading);
			double rotatedY = inputX * Math.sin(-heading) + inputY * Math.cos(-heading);

			// mecanum math
			double frontLeftPower = rotatedY + rotatedX + rx;
			double frontRightPower = rotatedY - rotatedX - rx;
			double backLeftPower = rotatedY - rotatedX + rx;
			double backRightPower = rotatedY + rotatedX - rx;

			// normalize so none of the motors go over 1
			double maxPower = Math.max(
				1.0,
				Math.max(
					Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower)),
					Math.max(Math.abs(backLeftPower), Math.abs(backRightPower))
				)
			);

			frontLeftPower /= maxPower;
			frontRightPower /= maxPower;
			backLeftPower /= maxPower;
			backRightPower /= maxPower;

			leftFrontDrive.setPower(frontLeftPower);
			rightFrontDrive.setPower(frontRightPower);
			leftBackDrive.setPower(backLeftPower);
			rightBackDrive.setPower(backRightPower);

			// intake control
			if (gamepad1.right_bumper) {
				iMotor.setPower(1);
				intakeLeft.setPower(-1);
				intakeRight.setPower(1);
			} else if (gamepad1.left_bumper) {
				iMotor.setPower(-1);
				intakeLeft.setPower(1);
				intakeRight.setPower(-1);
			} else {
				iMotor.setPower(0);
				intakeLeft.setPower(0);
				intakeRight.setPower(0);
			}

			// reset heading if needed
			// this also resets x and y to zero
			if (gamepad1.y) {
				resetIMUHeading();
			}

			// telemetry
			telemetry.addData("x position", "%.2f", robotX);
			telemetry.addData("y position", "%.2f", robotY);
			telemetry.addData("heading degrees", "%.2f", Math.toDegrees(robotHeading));

			telemetry.addData("parallel ticks", odoParallel.getCurrentPosition());
			telemetry.addData("perpendicular ticks", odoPerpendicular.getCurrentPosition());

			telemetry.addData("left front power", "%.2f", frontLeftPower);
			telemetry.addData("right front power", "%.2f", frontRightPower);
			telemetry.addData("left back power", "%.2f", backLeftPower);
			telemetry.addData("right back power", "%.2f", backRightPower);

			telemetry.addData("left stick y", gamepad1.left_stick_y);
			telemetry.addData("left stick x", gamepad1.left_stick_x);
			telemetry.addData("right stick x", gamepad1.right_stick_x);
			telemetry.addLine("press y to reset heading");
			telemetry.update();
		}
	}

	// this updates the robot position using
	// one forward encoder, one sideways encoder, and the imu
	private void updateOdometry() {
		int currentParallelTicks = odoParallel.getCurrentPosition();
		int currentPerpTicks = odoPerpendicular.getCurrentPosition();
		double currentHeading = getIMUHeading();

		int deltaParallelTicks = currentParallelTicks - prevParallelTicks;
		int deltaPerpTicks = currentPerpTicks - prevPerpendicularTicks;
		double deltaHeading = normalizeAngle(currentHeading - prevHeading);

		double deltaParallelInches = deltaParallelTicks * INCHES_PER_TICK;
		double deltaPerpInches = deltaPerpTicks * INCHES_PER_TICK;

		// the sideways wheel changes a little just from turning
		// this subtracts that out
		double correctedPerpInches = deltaPerpInches - (PERPENDICULAR_OFFSET * deltaHeading);

		// using average heading for the loop gives a little better tracking
		double avgHeading = prevHeading + deltaHeading / 2.0;
		//skjdfhksjdfhksdf
		double deltaFieldX;
		double deltaFieldY;

		// if we barely turned, just treat it like straight movement
		if (Math.abs(deltaHeading) < 1e-6) {
			deltaFieldX = deltaParallelInches * Math.cos(avgHeading) - correctedPerpInches * Math.sin(avgHeading);
			deltaFieldY = deltaParallelInches * Math.sin(avgHeading) + correctedPerpInches * Math.cos(avgHeading);
		} else {
			// if we are turning, use arc math instead
			// this tracks better than pretending the robot moved in a straight line
			double strafeRadius = correctedPerpInches / deltaHeading;
			double forwardRadius = deltaParallelInches / deltaHeading;

			deltaFieldX =
				(forwardRadius * Math.sin(deltaHeading) + strafeRadius * (Math.cos(deltaHeading) - 1.0)) *
				Math.cos(prevHeading) - (forwardRadius * (1.0 - Math.cos(deltaHeading)) + strafeRadius * Math.sin(deltaHeading)) *
				Math.sin(prevHeading);

			deltaFieldY =
				(forwardRadius * Math.sin(deltaHeading) + strafeRadius * (Math.cos(deltaHeading) - 1.0)) *
				Math.sin(prevHeading) +
				(forwardRadius * (1.0 - Math.cos(deltaHeading)) + strafeRadius * Math.sin(deltaHeading)) *
				Math.cos(prevHeading);
		}

		robotX += deltaFieldX;
		robotY += deltaFieldY;
		robotHeading = currentHeading;

		prevParallelTicks = currentParallelTicks;
		prevPerpendicularTicks = currentPerpTicks;
		prevHeading = currentHeading;
	}

	// get heading from the imu and apply the reset offset
	private double getIMUHeading() {
		Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);

		return normalizeAngle(angles.firstAngle - imuHeadingOffset);
	}

	// makes the current robot direction count as zero
	// also resets the tracked x and y position
	private void resetIMUHeading() {
		Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);

		imuHeadingOffset = angles.firstAngle;

		robotX = 0.0;
		robotY = 0.0;
		robotHeading = 0.0;

		prevHeading = 0.0;
		prevParallelTicks = odoParallel.getCurrentPosition();
		prevPerpendicularTicks = odoPerpendicular.getCurrentPosition();
	}

	// keeps the angle between -pi and pi
	private double normalizeAngle(double angle) {
		while (angle > Math.PI) {
			angle -= 2.0 * Math.PI;
		}

		while (angle < -Math.PI) {
			angle += 2.0 * Math.PI;
		}

		return angle;
	}
}
