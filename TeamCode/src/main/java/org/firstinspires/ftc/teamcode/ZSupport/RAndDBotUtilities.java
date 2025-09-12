package org.firstinspires.ftc.teamcode.ZSupport;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;


import java.util.Locale;

/*
These utilites implement the goBILDA® Pinpoint Odometry Computer.
The goBILDA Odometry Computer is a device designed to solve the Pose Exponential calculation
commonly associated with Dead Wheel Odometry systems. It reads two encoders, and an integrated
system of senors to determine the robot's current heading, X position, and Y position.

it uses an ESP32-S3 as a main cpu, with an STM LSM6DSV16X IMU.
It is validated with goBILDA "Dead Wheel" Odometry pods, but should be compatible with any
quadrature rotary encoder. The ESP32 PCNT peripheral is speced to decode quadrature encoder signals
at a maximum of 40mhz per channel. Though the maximum in-application tested number is 130khz.

The device expects two perpendicularly mounted Dead Wheel pods. The encoder pulses are translated
into mm and their readings are transformed by an "offset", this offset describes how far away
the pods are from the "tracking point", usually the center of rotation of the robot.

Dead Wheel pods should both increase in count when moved forwards and to the left.
The gyro will report an increase in heading when rotated counterclockwise.

The Pose Exponential algorithm used is described on pg 181 of this book:
https://github.com/calcmogul/controls-engineering-in-frc

For support, contact tech@gobilda.com
 */

public class RAndDBotUtilities {
    final double TRACK_WIDTH = 0.0;
    DcMotor fr, fl, br, bl;
    public double frPower, flPower, brPower, blPower;

    public GoBildaPinpointDriver odo;
    final double MAX_POWER_DISTANCE = 500;
    final double MAX_POWER_HEADING_ERROR = 45;

    private double targetHeading = 0.0;

    double oldTime = 0.0;

    LinearOpMode linearOpMode;

    public RAndDBotUtilities(LinearOpMode l)
    {
        linearOpMode = l;
    }

    public void initialize(LinearOpMode l)
    {
        odo = l.hardwareMap.get(GoBildaPinpointDriver.class, "pinPoint");

        fr = l.hardwareMap.get(DcMotor.class, "fr");
        fl = l.hardwareMap.get(DcMotor.class, "fl");
        br = l.hardwareMap.get(DcMotor.class, "br");
        bl = l.hardwareMap.get(DcMotor.class, "bl");

        fr.setDirection(DcMotorSimple.Direction.REVERSE);
//        fl.setDirection(DcMotorSimple.Direction.REVERSE);
        br.setDirection(DcMotorSimple.Direction.REVERSE);
//        bl.setDirection(DcMotorSimple.Direction.REVERSE);

        configureOdo();
        resetPosAndIMU();
    }

    void configureOdo()
    {
         /*
        Set the odometry pod positions relative to the point that the odometry computer tracks around.
        The X pod offset refers to how far sideways from the tracking point the
        X (forward) odometry pod is. Left of the center is a positive number,
        right of center is a negative number. the Y pod offset refers to how far forwards from
        the tracking point the Y (strafe) odometry pod is. forward of center is a positive number,
        backwards is a negative number.
         */
        odo.setOffsets(83.75736, -138.50000, DistanceUnit.MM); //these are tuned for 3110-0002-0001 Product Insight #1

        /*
        Set the kind of pods used by your robot. If you're using goBILDA odometry pods, select either
        the goBILDA_SWINGARM_POD, or the goBILDA_4_BAR_POD.
        If you're using another kind of odometry pod, uncomment setEncoderResolution and input the
        number of ticks per unit of your odometry pod.
         */
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        //odo.setEncoderResolution(13.26291192, DistanceUnit.MM);


        /*
        Set the direction that each of the two odometry pods count. The X (forward) pod should
        increase when you move the robot forward. And the Y (strafe) pod should increase when
        you move the robot to the left.
         */
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);


        /*
        Before running the robot, recalibrate the IMU. This needs to happen when the robot is stationary
        The IMU will automatically calibrate when first powered on, but recalibrating before running
        the robot is a good idea to ensure that the calibration is "good".
        resetPosAndIMU will reset the position to 0,0,0 and also recalibrate the IMU.
        This is recommended before you run your autonomous, as a bad initial calibration can cause
        an incorrect starting value for x, y, and heading.
         */
        //odo.recalibrateIMU();
        odo.resetPosAndIMU();

        linearOpMode.telemetry.addData("Status", "Initialized");
        linearOpMode.telemetry.addData("X offset", odo.getXOffset(DistanceUnit.MM));
        linearOpMode.telemetry.addData("Y offset", odo.getYOffset(DistanceUnit.MM));
        linearOpMode.telemetry.addData("Device Version Number:", odo.getDeviceVersion());
        linearOpMode.telemetry.addData("Heading Scalar", odo.getYawScalar());
    }

    public void move(double x, double y, double r)
    {
        //represent inputs as 3D vector then normalize to ensure robot translates and turns at max speed if asked to, and if input exceeds possible power, normalize.
        double mag = Math.sqrt(x*x + y*y + r*r);

        if (mag > 1.0) {
            x /= mag;
            y /= mag;
            r /= mag;
        }
        flPower = x + y + r;
        frPower = x - y - r;
        blPower = x - y + r;
        brPower = x + y - r;


        applyDrivePower();
    }

    public void moveFieldOriented(double x, double y, double r)
    {
        double heading = odo.getHeading(AngleUnit.RADIANS);

        double cos = Math.cos(heading);
        double sin = Math.sin(heading);

        // rotate field input into robot-relative coordinates
        double xr =  x * cos + y * sin;
        double yr = -x * sin + y * cos;

        // calculate wheel powers
        flPower = xr - yr + r;
        frPower = xr + yr - r;
        blPower = xr + yr + r;
        brPower = xr - yr - r;

        // normalize so no wheel exceeds magnitude 1
        double max = Math.max(1.0, Math.max(Math.abs(flPower),
                Math.max(Math.abs(frPower),
                        Math.max(Math.abs(blPower), Math.abs(brPower)))));

        flPower /= max;
        frPower /= max;
        blPower /= max;
        brPower /= max;

        applyDrivePower();
    }



    public void zeroPower()
    {
        flPower = 0;
        blPower = 0;
        brPower = 0;
        frPower = 0;
    }

    public void applyDrivePower()
    {
        fl.setPower(flPower);
        bl.setPower(blPower);
        br.setPower(brPower);
        fr.setPower(frPower);
    }

    public void updateLocalization()
    {
        odo.update();
    }

    /**
     * @implNote  can update only the heading of the device. This takes less time to read, but will not pull any other data. Only the heading (which you can pull with getHeading() or in getPosition().
     **/
    public void updateHeading()
    {
        odo.update(GoBildaPinpointDriver.ReadData.ONLY_UPDATE_HEADING);
    }

    /**
     * @implNote  Recalibrates IMU, does not reset pos
     **/
    public void recalibrateIMU()
    {
        odo.recalibrateIMU();
    }

    public void resetPosAndIMU()
    {
        odo.resetPosAndIMU();
    }


    public void logPinpointFrequency()
    {
        linearOpMode.telemetry.addData("Pinpoint Frequency", odo.getFrequency());
    }

    public void logREVHubFrequency()
    {
        double newTime = linearOpMode.getRuntime();
        double loopTime = newTime-oldTime;
        double frequency = 1/loopTime;
        oldTime = newTime;
        linearOpMode.telemetry.addData("REV Hub Frequency", frequency);
    }

    public void logDriveData(Pose2D pos)
    {
        String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getX(DistanceUnit.MM), pos.getY(DistanceUnit.MM), pos.getHeading(AngleUnit.DEGREES));
        linearOpMode.telemetry.addData("Position", data);

        String velocity = String.format(Locale.US,"{XVel: %.3f, YVel: %.3f, HVel: %.3f}", odo.getVelX(DistanceUnit.MM), odo.getVelY(DistanceUnit.MM), odo.getHeadingVelocity(UnnormalizedAngleUnit.DEGREES));
        linearOpMode.telemetry.addData("Velocity", velocity);
    }

    public void squidToPose(Pose2D targetPos)
    {
        double errX = targetPos.getX(DistanceUnit.MM) - odo.getPosX(DistanceUnit.MM);
        double errY = targetPos.getY(DistanceUnit.MM) - odo.getPosY(DistanceUnit.MM);
        double errH = targetPos.getHeading(AngleUnit.DEGREES) - odo.getHeading(AngleUnit.DEGREES);

        double xPow = signedSqrt(errX / MAX_POWER_DISTANCE);
        double yPow = signedSqrt(errY / MAX_POWER_DISTANCE);
        double hPow = signedSqrt(errH / MAX_POWER_HEADING_ERROR);

        moveFieldOriented(xPow, yPow, hPow);
    }

    public double squidToHeading(double targetHeading)
    {
        double errH = targetHeading - odo.getHeading(UnnormalizedAngleUnit.DEGREES);

        double hPow = signedSqrt(errH / MAX_POWER_HEADING_ERROR / 5);
        return hPow;
    }
    public double squidToHeadingCumulative(double targetHeading, double current)
    {
        double errH = targetHeading - odo.getHeading(AngleUnit.DEGREES);

        double hPow = signedSqrt(errH / MAX_POWER_HEADING_ERROR / 5);
        return hPow;
    }

    private double signedSqrt(double val)
    {
        double temp = val;
        return Math.copySign(Math.sqrt(Math.abs(temp)), val);
    }

    public void setTargetHeading(double h)
    {
        targetHeading = h;
    }
}
