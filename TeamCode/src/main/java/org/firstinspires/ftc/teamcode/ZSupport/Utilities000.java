package org.firstinspires.ftc.teamcode.ZSupport;

import android.graphics.Color;
import android.util.Size;

import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathBuilder;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants000;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.opencv.Circle;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.vision.opencv.ColorRange;
import org.firstinspires.ftc.vision.opencv.ImageRegion;

import java.util.List;

public class Utilities000 {
    private static final double VELOCITY_COMPENSATION_FACTOR = 1.6;
    OpMode opMode;
    private TelemetryManager telemetryM;
    public Follower follower;
    private GoBildaPinpointDriver odo;

    private static final double YAW_PULSES_PER_DEGREE = (1.0/360.0) * (70.0/10.0) * (384.5);
    private static final double PITCH_PULSES_PER_DEGREE = 0;

    private static final double TRANSFER_MIN = 0.2;
    private static final double TRANSFER_MAX = 0.82;
    private static final double SAFE_CURRENT = 1.5;
    private static final double MAX_CURRENT = 3.0;

    public double manualFlywheelPowerConstant = 0;
    private double manualAimOffsetX = 0.0;
    private double manualAimOffsetY = 0.0;

    private DcMotorEx intakeMotor;
    private DcMotorEx flywheelR;
    private DcMotorEx flywheelL;
    public DcMotorEx hoodYawMotor;
    private Servo hoodPitchServo;
    private Servo transferServo;
    public DcMotor fr, fl, br, bl;
    public double flPower, frPower, brPower, blPower;
    private List<DcMotorEx> motors;
    private List<LynxModule> allHubs;
    private VisionPortal portal;
    ColorBlobLocatorProcessor colorLocatorPurple;
    ColorBlobLocatorProcessor colorLocatorGreen;
    private static final double resHorz = 640;
    private static final double resVert = 480;
    private static final double diagonalFOV = 78;
    private static final double webcamX = 6.5;
    private static final double webcamY = 0;
    private static final double webcamA = 120;
    private Limelight3A limelight;

    private VoltageSensor voltageSensor;

    public enum AllianceColor {RED, BLUE, UNKNOWN;}
    AllianceColor allianceColor = AllianceColor.UNKNOWN;

    public double yawShift = 0;
    public double pitchShift = 0;
    public int initialYawAfterAuto;
    /**Initialization code*/
    public Utilities000(OpMode l) {
        opMode = l;
    }

    public void initialize(OpMode l) {
        follower = Constants000.createFollower(l.hardwareMap);
        follower.startTeleOpDrive();
//        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        allHubs = l.hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
        motors = l.hardwareMap.getAll(DcMotorEx.class);

        fr = l.hardwareMap.get(DcMotor.class, "fr");
        fl = l.hardwareMap.get(DcMotor.class, "fl");
        br = l.hardwareMap.get(DcMotor.class, "br");
        bl = l.hardwareMap.get(DcMotor.class, "bl");
        bl.setDirection(DcMotorSimple.Direction.REVERSE);
        fl.setDirection(DcMotorSimple.Direction.REVERSE);

        odo = l.hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

        intakeMotor = l.hardwareMap.get(DcMotorEx.class, "intake");
        intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        flywheelR   = l.hardwareMap.get(DcMotorEx.class, "flyR");
        flywheelR.setDirection(DcMotorSimple.Direction.FORWARD);
        flywheelR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flywheelL   = l.hardwareMap.get(DcMotorEx.class, "flyL");
        flywheelL.setDirection(DcMotorSimple.Direction.REVERSE);
        flywheelL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        hoodYawMotor = l.hardwareMap.get(DcMotorEx.class, "hoodYaw");
        hoodYawMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        hoodYawMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        setHoodYawAngleTicks(0);
//        hoodYawMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hoodYawMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        hoodYawMotor.setPower(0);


        PIDFCoefficients vCoefficients = new PIDFCoefficients(5, 3, 2, 35);
        hoodYawMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, vCoefficients);

        PIDFCoefficients pCoefficients = new PIDFCoefficients(10, 0,0,0);
        hoodYawMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION, pCoefficients);

        hoodPitchServo = l.hardwareMap.get(Servo.class, "hoodPitch");
        transferServo = l.hardwareMap.get(Servo.class, "transfer");
        transferServo.scaleRange(TRANSFER_MIN, TRANSFER_MAX);

        limelight = l.hardwareMap.get(Limelight3A.class, "limelight");
        l.telemetry.setMsTransmissionInterval(10);
        limelight.pipelineSwitch(0);
        limelight.start();
//
        colorLocatorPurple = new ColorBlobLocatorProcessor.Builder()
                .setTargetColorRange(ColorRange.ARTIFACT_PURPLE)// Use a predefined color match
                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)
                .setRoi(ImageRegion.asUnityCenterCoordinates(-1, 1, 1, -1))
                .setDrawContours(true)   // Show contours on the Stream Preview
                .setBoxFitColor(0)       // Disable the drawing of rectangles
                .setCircleFitColor(Color.rgb(255, 255, 0)) // Draw a circle
                .setBlurSize(3)          // Smooth the transitions between different colors in image

                // the following options have been added to fill in perimeter holes.
                .setDilateSize(2)       // Expand blobs to fill any divots on the edges
                .setErodeSize(2)        // Shrink blobs back to original size
                .setMorphOperationType(ColorBlobLocatorProcessor.MorphOperationType.CLOSING)

                .build();

        colorLocatorGreen = new ColorBlobLocatorProcessor.Builder()
                .setTargetColorRange(ColorRange.ARTIFACT_GREEN)   // Use a predefined color match
                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)
                .setRoi(ImageRegion.asUnityCenterCoordinates(-1, 1, 1, -1))
                .setDrawContours(true)   // Show contours on the Stream Preview
                .setBoxFitColor(0)       // Disable the drawing of rectangles
                .setCircleFitColor(Color.rgb(255, 255, 0)) // Draw a circle
                .setBlurSize(3)          // Smooth the transitions between different colors in image

                // the following options have been added to fill in perimeter holes.
                .setDilateSize(2)       // Expand blobs to fill any divots on the edges
                .setErodeSize(2)        // Shrink blobs back to original size
                .setMorphOperationType(ColorBlobLocatorProcessor.MorphOperationType.CLOSING)

                .build();

        portal = new VisionPortal.Builder()
                .addProcessor(colorLocatorPurple)
                .addProcessor(colorLocatorGreen)
                .setCameraResolution(new Size((int)resHorz, (int)resVert))
                .setCamera(l.hardwareMap.get(WebcamName.class, "Webcam 1"))
                .build();

        voltageSensor = l.hardwareMap.get(VoltageSensor.class, "Control Hub");
    }

//    public void move(double forward, double right, double r, double speed) {
//        forward *= speed;
//        right *= speed;
//        r *= speed;
//        //represent inputs as 3D vector then normalize to ensure robot translates and turns at max speed if asked to, and if input exceeds possible power, normalize.
//        double mag = Math.sqrt(forward * forward + right * right + r * r);
//
//        if (mag > 1.0) {
//            forward /= mag;
//            right /= mag;
//            r /= mag;
//        }
//        flPower = forward + right - r;
//        frPower = -forward + right - r;
//        blPower = -forward + right + r;
//        brPower = forward + right + r;
//
//
//        applyDrivePower();
//    }
//    public void applyDrivePower() {
//        fl.setPower(flPower);
//        bl.setPower(blPower);
//        br.setPower(brPower);
//        fr.setPower(frPower);
//    }

    /**Useable methods*/
    public void move(double forward, double left, double rotateCounterclockwise, double speed) {
        forward                = speed * deadZone(forward, 0.05);
        left                   = speed * deadZone(left, 0.05);
        rotateCounterclockwise = speed * deadZone(rotateCounterclockwise, 0.05);
        follower.setTeleOpDrive(forward, left, rotateCounterclockwise, false);
    }

    public void setAllianceColor(AllianceColor color) {
        allianceColor = color;
    }
    public AllianceColor getAllianceColor(){
        return allianceColor;
    }

    public double computeCurrentMultiplier(double current){
        if (current <= SAFE_CURRENT) return 1.0;
        if (current >= MAX_CURRENT)  return 0.5; //FIXME

        return 1.0 - (current - SAFE_CURRENT) / (MAX_CURRENT - SAFE_CURRENT);
    }
    public void intakePower(double pow) {
        double currentMultiplier = computeCurrentMultiplier(intakeMotor.getCurrent(CurrentUnit.AMPS));
        intakeMotor.setPower(deadZone(pow*currentMultiplier, 0.05));
    }

    public void setHoodYawAngleTicks(double ticks) {hoodYawMotor.setTargetPosition((int)ticks);}
    public void setHoodYawAngleDegrees(double degrees) {
        degrees = Math.min(Math.max(degrees, -120), 120);

        setHoodYawAngleTicks((int)((degrees+yawShift) * YAW_PULSES_PER_DEGREE));
    }
    public int getHoodYawAngleTicks(){return hoodYawMotor.getCurrentPosition();}
    public double getHoodYawAngleDegrees(){return getHoodYawAngleTicks() / YAW_PULSES_PER_DEGREE;}
    public void setHoodYawPower(double power) {hoodYawMotor.setPower(power);}
    public void setHoodPitchAngleTicks(double pos) {
        hoodPitchServo.setPosition(pos+pitchShift);
    }
    public void setHoodPitchAngleDegrees(double deg) {hoodPitchServo.setPosition((100.6-deg)/54.1);}
    public double getHoodPitchAngleDegrees() {return (100.6 -  54.1 * hoodPitchServo.getPosition());}

    public double getFlywheelSpeed() {
        double average = (flywheelL.getVelocity()+flywheelR.getVelocity()) / 2;
        return average;
    }
    public void setFlywheelVolts(double volts) {
        double currentVoltage = voltageSensor.getVoltage();
        flywheelL.setPower(volts/currentVoltage);
        flywheelR.setPower(volts/currentVoltage);
    }

    public double getUnnormalizedRobotHeading(){
        return odo.getHeading(UnnormalizedAngleUnit.DEGREES);
    }

    public void turnOffCamera() {
        portal.stopLiveView();
        portal.setProcessorEnabled(colorLocatorGreen, false);
        portal.setProcessorEnabled(colorLocatorPurple, false);
    }

    public void turnOnCamera() {
        portal.setProcessorEnabled(colorLocatorGreen, true);
        portal.setProcessorEnabled(colorLocatorPurple, true);
    }

    public void setTransferBlock(){
        transferServo.setPosition(0.5); //was 0.4
    }

    public void setTransferUp(){
        transferServo.setPosition(0.0);
    }
    public void setTransferDown(){
        transferServo.setPosition(1.0);
    }
    public void setFlywheelSpeed_DO_NOT_USE(int tickRate) {
        flywheelL.setVelocity(tickRate);
        flywheelR.setVelocity(tickRate);
    }

    public void addAmpTelemetry(){
        for (DcMotorEx motor : motors) {
            String name = motor.getDeviceName();
            double current = motor.getCurrent(CurrentUnit.AMPS);

            opMode.telemetry.addData(
                    name,
                    "%.2f A",
                    current
            );
        }
        double totalCurrent = 0;
        for (LynxModule hub : allHubs) {
            totalCurrent += hub.getCurrent(CurrentUnit.AMPS);
        }
        opMode.telemetry.addData("Total Current: ", totalCurrent);
    }

    public void turrentUpdate() {
        double[] shotVector = findShot();//subtractMovement();
        flywheelController(shotVector[0]);
        setHoodPitchAngleTicks(shotVector[1]);
        setHoodYawAngleDegrees(shotVector[2]);
//        opMode.telemetry.addData("Turrent: ", shotVector[2]);
    }

    public void turrentUpdate(boolean subtractMovement) {
        double[] shotVector;
        if(subtractMovement){
            shotVector = subtractMovement();
        }else {
            shotVector = findShot();
        }

        flywheelController(shotVector[0]);
        setHoodPitchAngleTicks(shotVector[1]);
        setHoodYawAngleDegrees(shotVector[2]);
    }

    public void disarmTurrent() {
        flywheelController(0);
    }

    public boolean limelightUpdate() {
        boolean updated = false;
        LLResult result = limelight.getLatestResult();
        if (result.isValid()) {
            Pose3D pose = result.getBotpose();
            if (pose.getPosition().x!=0 || pose.getPosition().y!=0) {
                updated = true;
                follower.setPose(new Pose(72+pose.getPosition().y/0.0254, 72-pose.getPosition().x/0.0254, pose.getOrientation().getYaw(AngleUnit.RADIANS)+Math.PI/2));
            }
        }
        return updated;
    }

    public void updateOdo(){
        follower.update();
    }

    /**Internal utilities*/
    private double deadZone(double value, double minimum) {
        if (Math.abs(value) > minimum) {
            return value;
        } else {
            return 0;
        }
    }

    public double aimAtPoint(Pose point) {
        Pose currentPose = follower.getPose();
        Pose path = point.minus(currentPose);

        double dX = path.getX();
        double dY = path.getY();

        double deg = Math.toDegrees(Math.atan2(dY, dX));


        double turretDeg = deg - (180+Math.toDegrees(odo.getHeading(UnnormalizedAngleUnit.RADIANS)));
//        double turretDeg = - odo.getHeading(AngleUnit.DEGREES);

//        turretDeg = Math.min(170, Math.max(-170, turretDeg));

        turretDeg = Math.toDegrees(
                Math.atan2(
                        Math.sin(Math.toRadians(turretDeg)),
                        Math.cos(Math.toRadians(turretDeg))
                )
        );

        return turretDeg;
    }

//    public double aimAtPoint(Pose point) {
//
//        Pose currentPose = follower.getPose();
//
//        // Compute vector to target
//        double dX = point.getX() - currentPose.getX();
//        double dY = point.getY() - currentPose.getY();
//
//        // Field-relative angle to target (degrees)
//        double fieldDeg = Math.toDegrees(Math.atan2(dY, dX));
//
//        // Continuous robot heading (degrees)
//        double robotDeg = Math.toDegrees(
//                odo.getHeading(UnnormalizedAngleUnit.RADIANS)
//        );
//
//        // Step 1: Compute pure relative error
//        double raw = fieldDeg - robotDeg;
//
//        // Step 2: Normalize relative error
//        raw = Math.toDegrees(
//                Math.atan2(
//                        Math.sin(Math.toRadians(raw)),
//                        Math.cos(Math.toRadians(raw))
//                )
//        );
////        // Step 3: Apply turret mounting offset (if zero faces backwards)
//        raw += 180.0;
//        opMode.telemetry.addData("RAW YAW CALCULATION", raw);
////
////        // Step 4: Normalize again after offset
////        raw = Math.toDegrees(
////                Math.atan2(
////                        Math.sin(Math.toRadians(raw)),
////                        Math.cos(Math.toRadians(raw))
////                )
////        );
////
////        double limit = 170.0;
////        double wrapThreshold = 180.0 + (180.0 - limit); // 190
////
////        // ---- Dead Zone Handling ----
////
////        if (raw > limit && raw <= wrapThreshold) {
////            return limit;
////        }
////
////        if (raw < -limit && raw >= -wrapThreshold) {
////            return -limit;
////        }
////
////        // ---- Actual Wrap After Clearing Dead Zone ----
////
////        if (raw > wrapThreshold) {
////            return raw - 360.0;
////        }
////
////        if (raw < -wrapThreshold) {
////            return raw + 360.0;
////        }
//
//        return raw;
//    }

    public double getAngleRelativeToPoint(int x, int y) {
        Pose currentPose = follower.getPose();
        double currentX = currentPose.getX();
        double currentY = currentPose.getY();

        double dX = x - currentX;
        double dY = y - currentY;

        double deg = Math.toDegrees(Math.atan2(dY, dX));

        return deg;
    }

    public Pose getPosition(){
        return follower.getPose();
    }

    public void flywheelController(double targetTicksPerSec) {
        double currentTicksPerSec = getFlywheelSpeed();
        double p = 0.005;
        double m = 0.00456;
        double b = 0.675;
        double feedForward = m*targetTicksPerSec + b;

        if (targetTicksPerSec < 0){
            setFlywheelVolts(-2);
        } else if (currentTicksPerSec < 0.9*targetTicksPerSec) {
            setFlywheelVolts(12);
        } else if (currentTicksPerSec > 1.1*targetTicksPerSec) {
            setFlywheelVolts(0);
        } else {
            setFlywheelVolts(feedForward + p*(targetTicksPerSec-currentTicksPerSec));
        }
    }

    public Pose[] getArtifactPoses(){
        Pose currentPose = follower.getPose();
        List<ColorBlobLocatorProcessor.Blob> blobs = colorLocatorPurple.getBlobs();
        blobs.addAll(colorLocatorGreen.getBlobs());

        ColorBlobLocatorProcessor.Util.filterByCriteria(
                ColorBlobLocatorProcessor.BlobCriteria.BY_CONTOUR_AREA,
                500, resHorz*resVert*10, blobs);  // filter out very small blobs.

        ColorBlobLocatorProcessor.Util.filterByCriteria(
                ColorBlobLocatorProcessor.BlobCriteria.BY_CIRCULARITY,
                0.5, 1, blobs);
        //focal length = apparent radius (px) * known distance
        double focalLength = 1570;
        //Field Of View Scaling
        double fovScaling = diagonalFOV / Math.sqrt(resHorz*resHorz + resVert*resVert);
        Pose[] artifactPoses = new Pose[blobs.size()];
        opMode.telemetry.addData("say hi", blobs.size());
        for (int i=0; i<blobs.size(); i++) {

            ColorBlobLocatorProcessor.Blob b = blobs.get(i);
            Circle circleFit = b.getCircle();

            double range = focalLength / circleFit.getRadius();
            double theta = webcamA + ((circleFit.getY() - resVert / 2) * fovScaling);
            double phi = Math.toDegrees(currentPose.getHeading()) - ((circleFit.getX() - resHorz / 2) * fovScaling);

            double blobX = currentPose.getX() + range * Math.sin(Math.toRadians(theta)) * Math.cos(Math.toRadians(phi));
            double blobY = currentPose.getY() + range * Math.sin(Math.toRadians(theta)) * Math.sin(Math.toRadians(phi));
            artifactPoses[i] = new Pose(blobX, blobY);

            //debugging lines
            opMode.telemetry.addLine(String.format("r theta phi (%5.1f, %5.1f,%5.1f)", range, theta, phi));
            opMode.telemetry.addLine(String.format("artifact location: (%5.1f,%5.1f)", blobX, blobY));
            opMode.telemetry.addLine(String.format("artifact location: (%5.1f,%5.1f)", artifactPoses[i].getX(), artifactPoses[i].getY()));
            // Display the Blob's circularity, and the size (radius) and center location of its circleFit.
            //telemetry.addLine(String.format("%5.3f      %3d     (%3d,%3d)",
            //b.getCircularity(), (int) circleFit.getRadius(), (int) circleFit.getX(), (int) circleFit.getY()));
        }

        return artifactPoses;
    }

    public Pose getGoal(){
        if (allianceColor==AllianceColor.BLUE) {
            return new Pose(0 + manualAimOffsetX, 144 + manualAimOffsetY);
        } else if (allianceColor==AllianceColor.RED) {
            return new Pose(144 + manualAimOffsetX, 144 + manualAimOffsetY);
        } else {
            return new Pose(0, 0);
        }
    }

    public void setManualAimOffsets(double x, double y){
        manualAimOffsetX = x;
        manualAimOffsetY = y;
    }

    public double flywheelSpeedFit() {
        double distance = getGoal().distanceFrom(follower.getPose());
        double speed = distance * 7.05 + 945;
        return speed + manualFlywheelPowerConstant; //HUY ADJUSTMENT REMOVE LATER
    }
    private double flywheelPitchFit() {
        double distance = getGoal().distanceFrom(follower.getPose());
//        double pitch = distance * 0.0034 + 0.331;
        double pitch = distance * 0.00554285714286 + 0.3; // Khai-tuned line
        return pitch;
    }

    /**This finds the shot vector when stationary
     * @return [speed, pitch, yaw]
     */
    private double[] findShot() {
        double[] values = new double[3];
        values[0] = flywheelSpeedFit();
        values[1] = flywheelPitchFit();
        values[2] = aimAtPoint(getGoal());
        return values;
    }

    private double[] subtractMovement() {
        Vector movement = follower.getVelocity();
        Pose currentPose = follower.getPose();
        double[] prevShot = findShot();
        double shotFactor = 2.83*3.14/56;
        prevShot[0] *= shotFactor;
        prevShot[2] += (180+Math.toDegrees(currentPose.getHeading()));

        double X = prevShot[0] * Math.cos(Math.toRadians(100.6 -  54.1 * prevShot[1])) * Math.cos(Math.toRadians(prevShot[2]));
        double Y = prevShot[0] * Math.cos(Math.toRadians(100.6 -  54.1 * prevShot[1])) * Math.sin(Math.toRadians(prevShot[2]));
        double Z = prevShot[0] * Math.sin(Math.toRadians(100.6 -  54.1 * prevShot[1]));

        X -= movement.getXComponent() * VELOCITY_COMPENSATION_FACTOR;
        Y -= movement.getYComponent() * VELOCITY_COMPENSATION_FACTOR;

        double[] newShot = new double[3];
        newShot[0] = Math.sqrt(X*X + Y*Y + Z*Z) / shotFactor;
        newShot[1] = (90-Math.toDegrees(Math.atan(Math.sqrt(X*X + Y*Y)/Z))-100.6)/(-54.1);
        newShot[2] = Math.toDegrees(Math.atan2(Y, X))-(180+Math.toDegrees(odo.getHeading(UnnormalizedAngleUnit.RADIANS)));
//        double turretDeg = deg - (180+Math.toDegrees(odo.getHeading(UnnormalizedAngleUnit.RADIANS)));

        newShot[2] = Math.toDegrees(
                Math.atan2(
                        Math.sin(Math.toRadians(newShot[2])),
                        Math.cos(Math.toRadians(newShot[2]))
                )
        );


        return newShot;
    }

    public PathChain pathToArtifacts(Pose[] artifactPoses) {
        Pose currentPose = follower.getPose();

        PathBuilder robotPath = new PathBuilder(follower);
        Path a = new Path(new BezierLine(currentPose, artifactPoses[0]));
        a.setTangentHeadingInterpolation();
        robotPath.addPath(a);
        return robotPath.build();
    }

    public static class RobotStateAfterAuto {
        public static Pose postAutoPose;
        public static int postAutoYawAngle;
        public static boolean wasAuto = false;

        private static void setPostAutoPose(Pose p) {
            postAutoPose = p;
            wasAuto = true;
        }

        private static void setPostAutoYawTicks(int t) {
            postAutoYawAngle = t;
            wasAuto = true;
        }

        public static void setPostAutoState(Pose p, int t) {
            setPostAutoPose(p);
            setPostAutoYawTicks(t);
            wasAuto = true;
        }
    }

}
