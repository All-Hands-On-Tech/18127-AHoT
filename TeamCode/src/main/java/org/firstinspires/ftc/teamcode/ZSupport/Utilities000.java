package org.firstinspires.ftc.teamcode.ZSupport;

import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.List;

public class Utilities000 {
    LinearOpMode linearOpMode;
    private TelemetryManager telemetryM;
    private Follower follower;

    private static final double YAW_PULSES_PER_DEGREE = (1.0/360.0) * (70.0/10.0) * (384.5);
    private static final double PITCH_PULSES_PER_DEGREE = 0;

    private static final double TRANSFER_MIN = 0.2;
    private static final double TRANSFER_MAX = 0.82;


    private DcMotor intakeMotor;
    private DcMotorEx flywheelR;
    private DcMotorEx flywheelL;
    private DcMotorEx hoodYawMotor;
    private Servo hoodPitchServo;
    private Servo transferServo;
    public DcMotor fr, fl, br, bl;
    private List<DcMotorEx> motors;
    private List<LynxModule> allHubs;
    public double frPower, flPower, brPower, blPower;

    public GoBildaPinpointDriver odo;

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
        allHubs = l.hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
        motors = l.hardwareMap.getAll(DcMotorEx.class);
        odo = l.hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

        fr = l.hardwareMap.get(DcMotor.class, "fr");
        fl = l.hardwareMap.get(DcMotor.class, "fl");
        br = l.hardwareMap.get(DcMotor.class, "br");
        bl = l.hardwareMap.get(DcMotor.class, "bl");
        fr.setDirection(DcMotorSimple.Direction.REVERSE);
        fl.setDirection(DcMotorSimple.Direction.REVERSE);

        intakeMotor = l.hardwareMap.get(DcMotor.class, "intake");
        intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        flywheelR   = l.hardwareMap.get(DcMotorEx.class, "flyR");
        flywheelR.setDirection(DcMotorSimple.Direction.FORWARD);
        flywheelR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flywheelL   = l.hardwareMap.get(DcMotorEx.class, "flyL");
        flywheelL.setDirection(DcMotorSimple.Direction.REVERSE);
        flywheelL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        hoodYawMotor = l.hardwareMap.get(DcMotorEx.class, "hoodYaw");
        hoodYawMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        hoodYawMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hoodYawMotor.setTargetPosition(0);
        hoodYawMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        PIDFCoefficients vCoefficients = new PIDFCoefficients(5, 3, 2, 35);
        hoodYawMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, vCoefficients);

        PIDFCoefficients pCoefficients = new PIDFCoefficients(10, 0,0,0);
        hoodYawMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION, pCoefficients);

        hoodPitchServo = l.hardwareMap.get(Servo.class, "hoodPitch");
        transferServo = l.hardwareMap.get(Servo.class, "transfer");
        transferServo.scaleRange(TRANSFER_MIN, TRANSFER_MAX);

        odo.initialize();
        odo.setOffsets(80.025, 176.475, DistanceUnit.MM);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
//        limelight = l.hardwareMap.get(Limelight3A.class, "limelight");
//        l.telemetry.setMsTransmissionInterval(10);
//        limelight.pipelineSwitch(0);
//        limelight.start();
//
//        portal = initWebcam(l);

        voltageSensor = l.hardwareMap.get(VoltageSensor.class, "Control Hub");
    }

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

    public void intakePower(double pow) {intakeMotor.setPower(deadZone(pow, 0.05));}

    public void setHoodYawAngleTicks(double ticks) {hoodYawMotor.setTargetPosition((int)ticks);}
    public void setHoodYawAngleDegrees(double degrees) {hoodYawMotor.setTargetPosition((int)(degrees * YAW_PULSES_PER_DEGREE));}
    public double getHoodYawAngleDegrees(){return hoodYawMotor.getCurrentPosition() / YAW_PULSES_PER_DEGREE;}
    public void setHoodYawPower(double power) {hoodYawMotor.setPower(power);}
    public void setHoodPitchAngleTicks(double pos) {
        hoodPitchServo.setPosition(pos);
    }
    public void setHoodPitchAngleDegrees(double pos) {hoodPitchServo.setPosition(pos * PITCH_PULSES_PER_DEGREE);}
    public double getHoodPitchAngleDegrees() {return (10000 - hoodPitchServo.getPosition()/ PITCH_PULSES_PER_DEGREE);}

    public double getFlywheelSpeed() {
        double average = (flywheelL.getVelocity()+flywheelR.getVelocity()) / 2;
        return average;
    }
    public void setFlywheelVolts(double volts) {
        double currentVoltage = voltageSensor.getVoltage();
        flywheelL.setPower(volts/currentVoltage);
        flywheelR.setPower(volts/currentVoltage);
    }

    public void setTransferBlock(){
        transferServo.setPosition(0.5);
    }

    public void setTransferUp(){
        transferServo.setPosition(0);
    }
    public void setTransferDown(){
        transferServo.setPosition(1);
    }
    public void setFlywheelSpeed_DO_NOT_USE(int tickRate) {
        flywheelL.setVelocity(tickRate);
        flywheelR.setVelocity(tickRate);
    }

    public void addAmpTelemetry(){
        for (DcMotorEx motor : motors) {
            String name = motor.getDeviceName();
            double current = motor.getCurrent(CurrentUnit.AMPS);

            linearOpMode.telemetry.addData(
                    name,
                    "%.2f A",
                    current
            );
        }
        double totalCurrent = 0;
        for (LynxModule hub : allHubs) {
            totalCurrent += hub.getCurrent(CurrentUnit.AMPS);
        }
        linearOpMode.telemetry.addData("Total Current: ", totalCurrent);
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

    private void flywheelController(double targetTicksPerSec) {
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


//    private double speedFunction() {
//        Pose current = follower.getPose();
//        double distance
//    }

}
