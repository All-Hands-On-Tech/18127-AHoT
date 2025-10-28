package org.firstinspires.ftc.teamcode.ZSupport;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
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

public class K2Utilities {
    final double TRACK_WIDTH = 0.0;
    public DcMotor fr, fl, br, bl;
    public DcMotor intakeL, intakeR;
    public DcMotorEx deliverL, deliverR;
    public double frPower, flPower, brPower, blPower;

    public GoBildaPinpointDriver odo;
    final double MAX_POWER_DISTANCE = 500;
    final double MAX_POWER_HEADING_ERROR = 45;

    private double targetHeading = 0.0;

    double oldTime = 0.0;

    LinearOpMode linearOpMode;

    public K2Utilities(LinearOpMode l)
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

//        intakeL = l.hardwareMap.get(DcMotor.class, "intakeL");
        intakeR = l.hardwareMap.get(DcMotor.class, "intakeR");

        deliverL = l.hardwareMap.get(DcMotorEx.class, "deliverL");
//        deliverR = l.hardwareMap.get(DcMotorEx.class, "deliverR");

        fr.setDirection(DcMotorSimple.Direction.REVERSE);
        fl.setDirection(DcMotorSimple.Direction.REVERSE);
        br.setDirection(DcMotorSimple.Direction.FORWARD);
        bl.setDirection(DcMotorSimple.Direction.FORWARD);

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
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.REVERSED);


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

    public void setPoseEstimate(Pose2D pose)
    {
        odo.setHeading(pose.getHeading(AngleUnit.DEGREES), AngleUnit.DEGREES);
        odo.setPosX(pose.getX(DistanceUnit.INCH), DistanceUnit.INCH);
        odo.setPosY(pose.getY(DistanceUnit.INCH), DistanceUnit.INCH);
    }

    public void move(double forward, double right, double r)
    {
        //represent inputs as 3D vector then normalize to ensure robot translates and turns at max speed if asked to, and if input exceeds possible power, normalize.
        double mag = Math.sqrt(forward*forward + right*right + r*r);

        if (mag > 1.0) {
            forward /= mag;
            right /= mag;
            r /= mag;
        }
        flPower = forward + right - r;
        frPower = -forward + right - r;
        blPower = -forward + right + r;
        brPower = forward + right + r;


        applyDrivePower();
    }
    /**
     @implNote BE AWARE OF THE ORIENTATION
     **/
    public void moveFieldOriented(double forward, double right, double r)
    {
        right*=-1;
        double heading = odo.getHeading(AngleUnit.RADIANS);

        double cos = Math.cos(heading);
        double sin = Math.sin(heading);

        // rotate field input into robot-relative coordinates
        double forwardR =  forward * cos + right * sin;
        double rightR = forward * sin - right * cos;

        linearOpMode.telemetry.addData("cosine ", cos);
        linearOpMode.telemetry.addData("sine ", sin);
        // calculate wheel powers
        flPower = forwardR + rightR - r;
        frPower = -forwardR + rightR - r;
        blPower = -forwardR + rightR + r;
        brPower = forwardR + rightR + r;

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

    /*
    public void moveFieldOriented(double forward, double right, double r) {
    double heading = odo.getHeading(AngleUnit.RADIANS);

    double cos = Math.cos(heading);
    double sin = Math.sin(heading);

    // Rotate FIELD vector by -heading to get ROBOT-relative
    // rightR =  right*cos + forward*sin
    // forwardR = -right*sin + forward*cos
    double rightR   =  right * cos + forward * sin;
    double forwardR = -right * sin + forward * cos;

    // Standard mecanum mix: y = forwardR, x = rightR, r = rotation (CCW positive)
    double fl = forwardR + rightR + r;
    double fr = forwardR - rightR - r;
    double bl = forwardR - rightR + r;
    double br = forwardR + rightR - r;

    // Normalize so no wheel exceeds 1.0
    double max = Math.max(1.0, Math.max(Math.abs(fl),
                 Math.max(Math.abs(fr), Math.max(Math.abs(bl), Math.abs(br)))));

    fl /= max; fr /= max; bl /= max; br /= max;

    flPower = fl; frPower = fr; blPower = bl; brPower = br;
    applyDrivePower();

    // Optional telemetry for sanity checks:
    // telemetry.addData("heading", heading);
    // telemetry.addData("forwardR", forwardR);
    // telemetry.addData("rightR", rightR);
    // telemetry.update();
}

     */



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
        String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getX(DistanceUnit.INCH), pos.getY(DistanceUnit.INCH), pos.getHeading(AngleUnit.DEGREES));
        linearOpMode.telemetry.addData("Position", data);

        String velocity = String.format(Locale.US,"{XVel: %.3f, YVel: %.3f, HVel: %.3f}", odo.getVelX(DistanceUnit.INCH), odo.getVelY(DistanceUnit.INCH), odo.getHeadingVelocity(UnnormalizedAngleUnit.DEGREES));
        linearOpMode.telemetry.addData("Velocity", velocity);
    }

    public void squidToPose(Pose2D targetPos)
    {

        moveFieldOriented(squidToY(targetPos.getX(DistanceUnit.MM)), -squidToX(targetPos.getX(DistanceUnit.MM)), squidToHeading(targetPos.getHeading(AngleUnit.DEGREES)));
    }

    public double squidToX(double x)
    {
        double errX = x - odo.getPosX(DistanceUnit.MM);
        double xPow = signedSqrt(errX / MAX_POWER_DISTANCE);

        return xPow;
    }

    public double squidToY(double y)
    {
        double errY = y - odo.getPosY(DistanceUnit.MM);
        double yPow = signedSqrt(errY / MAX_POWER_DISTANCE);

        return yPow;
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

//    public void intakeLeft (double pow){intakeL.setPower(pow);}
    public void intakeRight (double pow){intakeR.setPower(pow);}

    public void powerDeliver(double pow){
        deliverL.setPower(pow);
//        deliverR.setPower(pow);
    }
    public double getDeliverLVel(){
        return deliverL.getVelocity(AngleUnit.DEGREES);
        /*return deliverL.getVelocity(AngleUnit.RADIANS)/(2*Math.PI)/60;*/
    }  //rpm

    public void setDeliverVel(){
        deliverL.setVelocity(3, AngleUnit.DEGREES); // 3000 RPM roughly
//        deliverR.setVelocity(314, AngleUnit.RADIANS); // 3000 RPM roughly
    }
}
