package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.ZSupport.Utilities000;

import java.util.ArrayList;
import java.util.TreeMap;


@TeleOp(name="Lookup Table Data Collection", group="A")
public class DepoLookUpTableDataCollection extends LinearOpMode {

    private Utilities000 bot = new Utilities000(this);
    private ElapsedTime limeLightStaller = new ElapsedTime(10);
    private TreeMap<Double, RobotState> tempLookupList = new TreeMap<>();

    private double flywheelSpeed;
    private double hoodPitch;

    @Override
    public void runOpMode() {
        flywheelSpeed = 0;
        hoodPitch = 0;

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        bot.initialize(this);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        while (opModeIsActive()) {


            if(gamepad1.dpadDownWasPressed()){
                flywheelSpeed -= 50;
            }

            if(gamepad1.dpadUpWasPressed()){
                flywheelSpeed += 50;
            }

            if(gamepad1.right_trigger > 0.1){
                bot.intakePower(gamepad1.right_trigger);
            } else if(gamepad1.left_trigger > 0.1){
                bot.intakePower(-gamepad1.left_trigger);
            } else{
                bot.intakePower(0);
            }

            if(gamepad1.rightBumperWasPressed()){
                hoodPitch += 1;
                bot.setHoodPitchAngleDegrees(hoodPitch);
            }
            if(gamepad1.leftBumperWasPressed()){
                hoodPitch -= 1;
                bot.setHoodPitchAngleDegrees(hoodPitch);
            }

            if(gamepad1.aWasPressed()){
                double distance = bot.getGoal().distanceFrom(bot.follower.getPose());
                tempLookupList.put(distance, new RobotState(flywheelSpeed, hoodPitch));
            }

            bot.flywheelController(flywheelSpeed);



            //MANAGE POSITION ESTIMATE
            manageBotPose();

            telemetry.addData("Flywheel RPM: ", flywheelSpeed);
            telemetry.addData("Pitch: ", hoodPitch);

            addLookupTableTelemetry();
            telemetry.update();
        }

    }

    private void manageBotPose(){
        bot.updateOdo();
        if(gamepad1.yWasPressed()){
            bot.limelightUpdate();
        }
    }

    private void addLookupTableTelemetry(){
        for (TreeMap.Entry<Double, RobotState> entry : tempLookupList.entrySet()) {
            Double dist = entry.getKey();
            RobotState state = entry.getValue();

            telemetry.addLine("Distance: " + dist);
            telemetry.addData("Pitch: ", state.getPitch());
            telemetry.addData("RPM: ", state.getRPM());
            telemetry.addLine();
        }
    }

    @Config
    public static class DepoLookupTable {
        //Distance, RobotState -- sorted into treemap
        private static TreeMap<Double, RobotState> LOOKUP_TABLE = new TreeMap<>();

        public void initTable(){
//            LOOKUP_TABLE.put(); // Distance, new RobotState(rpm, pitch)
//            LOOKUP_TABLE.put(92.7, new RobotState(1200,48));
//            LOOKUP_TABLE.put(113.4, new RobotState(1050,54));
//            LOOKUP_TABLE.put(115.9, new RobotState(900,66));
//            LOOKUP_TABLE.put(120.97, new RobotState(750,80));
            LOOKUP_TABLE.put(0.0, new RobotState(0,80));

        }

        public RobotState getInterpolatedState(double distance){
            if (distance <= LOOKUP_TABLE.firstKey()) return LOOKUP_TABLE.firstEntry().getValue();
            if (distance >= LOOKUP_TABLE.lastKey()) return LOOKUP_TABLE.lastEntry().getValue();

            // 3. Get the two surrounding points
            java.util.Map.Entry<Double, RobotState> low = LOOKUP_TABLE.floorEntry(distance);
            java.util.Map.Entry<Double, RobotState> high = LOOKUP_TABLE.ceilingEntry(distance);

            double x0 = low.getKey();
            double x1 = high.getKey();
            RobotState y0 = low.getValue();
            RobotState y1 = high.getValue();

            // 4. Calculate the "t" value (how far we are between the two points, 0.0 to 1.0)
            double t = (distance - x0) / (x1 - x0);

            // 5. Interpolate Pitch and Speed
            double interpolatedPitch = y0.getPitch() + t * (y1.getPitch() - y0.getPitch());
            double interpolatedSpeed = y0.getRPM() + t * (y1.getRPM() - y0.getRPM());

            return new RobotState(interpolatedPitch, interpolatedSpeed);
        }
    }

    public static class RobotState {
        double rpm;
        double pitch;

        public RobotState(double rpm, double pitch){
            this.rpm = rpm;
            this.pitch = pitch;
        }

        public double getRPM(){
            return rpm;
        }
        public double getPitch(){
            return pitch;
        }
    }

}
