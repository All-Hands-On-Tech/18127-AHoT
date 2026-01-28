package org.firstinspires.ftc.teamcode.ZSupport;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;

import android.graphics.Color;
import android.util.Size;

import com.pedropathing.follower.Follower;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants000;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.vision.opencv.ColorRange;
import org.firstinspires.ftc.vision.opencv.ImageRegion;

import java.util.List;

public class Utilities000 {
    LinearOpMode linearOpMode;
//    private TelemetryManager telemetryM;
//    private Follower follower;

    private static final double YAW_PULSES_PER_DEGREE = (1.0/360.0) * (70.0/10.0) * (384.5);

    private DcMotor intakeMotor;
    private DcMotorEx flywheelR;
    private DcMotorEx flywheelL;
    private DcMotor hoodYawMotor;
    private Servo hoodPitchServo;

    public DcMotor fr, fl, br, bl;
    public double frPower, flPower, brPower, blPower;

//    private VisionPortal portal;
//    private static final double resHorz = 640;
//    private static final double resVert = 480;
//    private static final double diagonalFOV = 78;
//    private Limelight3A limelight;

    private VoltageSensor voltageSensor;


    /**Initialization code*/
    public Utilities000(LinearOpMode l) {
        linearOpMode = l;
    }

    public void initialize(LinearOpMode l) {
//        follower = Constants000.createFollower(l.hardwareMap);
//        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        List<LynxModule> allHubs = l.hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        fr = l.hardwareMap.get(DcMotor.class, "fr");
        fl = l.hardwareMap.get(DcMotor.class, "fl");
        br = l.hardwareMap.get(DcMotor.class, "br");
        bl = l.hardwareMap.get(DcMotor.class, "bl");
        fr.setDirection(DcMotorSimple.Direction.REVERSE);
        fl.setDirection(DcMotorSimple.Direction.REVERSE);

        intakeMotor = l.hardwareMap.get(DcMotor.class, "intake");
        intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        flywheelR   = l.hardwareMap.get(DcMotorEx.class, "flyR");
        flywheelR.setDirection(DcMotorSimple.Direction.REVERSE);
        flywheelL   = l.hardwareMap.get(DcMotorEx.class, "flyL");
        flywheelL.setDirection(DcMotorSimple.Direction.FORWARD);
        hoodYawMotor = l.hardwareMap.get(DcMotor.class, "hoodYaw");
        hoodYawMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        hoodPitchServo = l.hardwareMap.get(Servo.class, "hoodPitch");
        hoodYawMotor.setTargetPosition(0);
        hoodYawMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        hoodYawMotor.setPower(0.5);

//        limelight = l.hardwareMap.get(Limelight3A.class, "limelight");
//        l.telemetry.setMsTransmissionInterval(10);
//        limelight.pipelineSwitch(0);
//        limelight.start();
//
//        portal = initWebcam(l);

        voltageSensor = l.hardwareMap.get(VoltageSensor.class, "Control Hub");
    }

//    private VisionPortal initWebcam(LinearOpMode l) {
//        ColorBlobLocatorProcessor colorLocatorPurple = new ColorBlobLocatorProcessor.Builder()
//                .setTargetColorRange(ColorRange.ARTIFACT_PURPLE)// Use a predefined color match
//                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)
//                .setRoi(ImageRegion.asUnityCenterCoordinates(-1, 1, 1, -1))
//                .setDrawContours(true)   // Show contours on the Stream Preview
//                .setBoxFitColor(0)       // Disable the drawing of rectangles
//                .setCircleFitColor(Color.rgb(255, 255, 0)) // Draw a circle
//                .setBlurSize(3)          // Smooth the transitions between different colors in image
//
//                // the following options have been added to fill in perimeter holes.
//                .setDilateSize(2)       // Expand blobs to fill any divots on the edges
//                .setErodeSize(2)        // Shrink blobs back to original size
//                .setMorphOperationType(ColorBlobLocatorProcessor.MorphOperationType.CLOSING)
//
//                .build();
//
//        ColorBlobLocatorProcessor colorLocatorGreen = new ColorBlobLocatorProcessor.Builder()
//                .setTargetColorRange(ColorRange.ARTIFACT_GREEN)   // Use a predefined color match
//                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)
//                .setRoi(ImageRegion.asUnityCenterCoordinates(-1, 1, 1, -1))
//                .setDrawContours(true)   // Show contours on the Stream Preview
//                .setBoxFitColor(0)       // Disable the drawing of rectangles
//                .setCircleFitColor(Color.rgb(255, 255, 0)) // Draw a circle
//                .setBlurSize(3)          // Smooth the transitions between different colors in image
//
//                // the following options have been added to fill in perimeter holes.
//                .setDilateSize(2)       // Expand blobs to fill any divots on the edges
//                .setErodeSize(2)        // Shrink blobs back to original size
//                .setMorphOperationType(ColorBlobLocatorProcessor.MorphOperationType.CLOSING)
//
//                .build();
//
//        VisionPortal portalNew = new VisionPortal.Builder()
//                .addProcessor(colorLocatorPurple)
//                .addProcessor(colorLocatorGreen)
//                .setCameraResolution(new Size((int)resHorz, (int)resVert))
//                .setCamera(l.hardwareMap.get(WebcamName.class, "Webcam 1"))
//                .build();
//
//        return portalNew;
//    }

    public void move(double forward, double right, double r) {
        //represent inputs as 3D vector then normalize to ensure robot translates and turns at max speed if asked to, and if input exceeds possible power, normalize.
        double mag = Math.sqrt(forward * forward + right * right + r * r);

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
    public void applyDrivePower() {
        fl.setPower(flPower);
        bl.setPower(blPower);
        br.setPower(brPower);
        fr.setPower(frPower);
    }

    /**Useable methods*/
//    public void move(double forward, double left, double rotateCounterclockwise, double speed) {
//        forward                = speed * deadZone(forward, 0.05);
//        left                   = speed * deadZone(left, 0.05);
//        rotateCounterclockwise = speed * deadZone(rotateCounterclockwise, 0.05);
//        follower.setTeleOpDrive(forward, left, rotateCounterclockwise, false);
//    }

    public void intakePower(double pow) {
        pow = deadZone(pow, 0.05);
        intakeMotor.setPower(pow);
    }
    public void setHoodYawAngle(double ticks) {
        hoodYawMotor.setTargetPosition((int)ticks);
    }
    public void setHoodYawAngleDegrees(double degrees) {
        hoodYawMotor.setTargetPosition((int)(degrees * YAW_PULSES_PER_DEGREE));
    }
    public double getHoodYawAngleDegrees(){
        return hoodYawMotor.getCurrentPosition() / YAW_PULSES_PER_DEGREE;
    }
    public void setHoodPitchAngle(double pos) {
        hoodPitchServo.setPosition(pos);
    }
    public void flywheelController(double targetTicksPerSec) {
        double currentTicksPerSec = getFlywheelSpeed();
        double p = 0.5; //these values need to be tuned
        double m = 0;
        double b = 0;
        double feedForward = m*targetTicksPerSec + b;

        if (currentTicksPerSec < 0.9*targetTicksPerSec) {
            setFlywheelVolts(12);
        } else if (currentTicksPerSec > 1.1*targetTicksPerSec) {
            setFlywheelVolts(0);
        } else {
            setFlywheelVolts(feedForward + p*(targetTicksPerSec-currentTicksPerSec));
        }
    }
    public double getFlywheelSpeed() {
        double average = (flywheelL.getVelocity()+flywheelR.getVelocity()) / 2;
        return average;
    }
    private void setFlywheelVolts(double volts) {
        double currentVoltage = voltageSensor.getVoltage();
        flywheelL.setPower(volts/currentVoltage);
        flywheelR.setPower(volts/currentVoltage);
    }
    public void setFlywheelSpeed_DO_NOT_USE(int tickRate) {
        flywheelL.setVelocity(tickRate);
        flywheelR.setVelocity(tickRate);
    }
//    public void setPoseEstimate(Pose pose) {
//        follower.setPose(pose);
//    }
//    public void updateFollower() {
//        follower.update();
//    }

    /**Internal utilities*/
    private double deadZone(double value, double minimum) {
        if (Math.abs(value) > minimum) {
            return value;
        } else {
            return 0;
        }
    }

}
