package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.common.RobotHardware;
import org.firstinspires.ftc.teamcode.common.RobotConfig;
import org.firstinspires.ftc.teamcode.common.Odometry;
import org.firstinspires.ftc.teamcode.common.PanelsPublisher;

@TeleOp(name = "Two Wheel Odometry TeleOp FieldCentric", group = "TeleOp")
public class Robot1TeleFieldCentric extends LinearOpMode {
    private RobotHardware hw;
    private RobotConfig config;
    private Odometry odometry;
    private PanelsPublisher panels;
    private boolean prevResetStart = false;
    private double lastLoopTime = 0;

    @Override
    public void runOpMode() {
        hw = new RobotHardware(hardwareMap);
        config = new RobotConfig();
        hw.initPinpoint();
        panels = new PanelsPublisher();
        panels.init();

        telemetry.addLine("Init complete - waiting start");
        telemetry.addData("Pinpoint", hw.pinpoint == null ? "NOT FOUND (expect name '"+config.pinpointName+"')" : hw.pinpoint.getDeviceStatus());
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;
        lastLoopTime = getRuntime();

        odometry = new Odometry(hw, hw.pinpoint);

        while (opModeIsActive()) {
            hw.updatePinpoint();
            odometry.update();
            Odometry.Position pos = odometry.getPosition();

            // Get joystick values
            double y = Math.abs(gamepad1.left_stick_y) > 0.05 ? gamepad1.left_stick_y : 0;
            double x = Math.abs(gamepad1.left_stick_x) > 0.05 ? -gamepad1.left_stick_x : 0;
            double r = Math.abs(gamepad1.right_stick_x) > 0.05 ? -gamepad1.right_stick_x : 0;
            double speedMul = gamepad1.left_bumper ? 0.5 : 1.0;
            y*=speedMul; x*=speedMul; r*=speedMul; x*=1.1;
            y=Math.copySign(y*y*y,y); x=Math.copySign(x*x*x,x); r=Math.copySign(r*r*r,r);

            // Field-centric transformation
            double headingRad = pos.getHeadingRad(); // Heading in radians
            double tempX = x * Math.cos(-headingRad) - y * Math.sin(-headingRad);
            double tempY = x * Math.sin(-headingRad) + y * Math.cos(-headingRad);
            x = tempX;
            y = tempY;

            double denom = Math.max(Math.abs(y)+Math.abs(x)+Math.abs(r),1);
            double fl=(y+x+r)/denom, bl=(y - x + r)/denom, fr=(y - x - r)/denom, br=(y + x - r)/denom;
            hw.setDrivePowers(fl,fr,bl,br);
    
            if (gamepad1.a && gamepad1.x) hw.pinpoint.resetPosAndIMU();
            if (gamepad1.b && hw.pinpoint!=null) hw.pinpoint.recalibrateIMU();
            boolean resetPressed = gamepad2.start;
            if (resetPressed && !prevResetStart) {
                if (hw.pinpoint!=null) hw.pinpoint.resetPosAndIMU();
                else if (hw.imu!=null) hw.imu.resetYaw();
            }
            prevResetStart = resetPressed;

            double intakePower=0;
            if (gamepad1.right_trigger>0.1) intakePower=0.8; else if (gamepad1.left_trigger>0.1) intakePower=-0.8;
            if (hw.intakeMotor!=null) hw.intakeMotor.setPower(intakePower);

            telemetry.addLine("=== DRIVE ===");
            telemetry.addData("Joy","Y%.2f X%.2f R%.2f",y,x,r);
            telemetry.addData("Pow","FL%.2f FR%.2f BL%.2f BR%.2f",fl,fr,bl,br);
            telemetry.addData("Odo", pos.toString());
            if (hw.pinpoint!=null) {
                telemetry.addData("PP Stat", hw.pinpoint.getDeviceStatus());
            } else {
                telemetry.addLine("Pinpoint MISSING - name '"+config.pinpointName+"'");
            }
            telemetry.addData("IMU Yaw°", hw.imu!=null? Math.toDegrees(hw.imu.getRobotYawPitchRollAngles().getYaw()):"null");
            telemetry.addData("Intake", intakePower);
            double now = getRuntime(); double dt = now - lastLoopTime; lastLoopTime = now;
            telemetry.addData("REV Hz", dt>0? String.format("%.1f",1.0/dt):"-");
            telemetry.update();

            // Panels publishing (inches)
            double xIn = pos.getXmm()/25.4;
            double yIn = pos.getYmm()/25.4;
            panels.putText("teleop/pinpoint/status", hw.pinpoint!=null? hw.pinpoint.getDeviceStatus().name():"NONE");
            panels.publishPose("teleop/pose", xIn, yIn, pos.getHeadingDeg(), 50);
        }
    }
}

