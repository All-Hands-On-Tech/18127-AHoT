package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.common.RobotHardware;
import org.firstinspires.ftc.teamcode.pedroPathing.math.Vector;

/**
 * MecanumDrivetrain - Adapter for Pedro Pathing to control mecanum drive
 */
public class MecanumDrivetrain extends Drivetrain {
    private final RobotHardware hw;

    public MecanumDrivetrain(RobotHardware hw) {
        this.hw = hw;
        this.maxPowerScaling = 1.0;
        this.voltageCompensation = false;
        this.nominalVoltage = 12.0;

        // Mecanum drive movement vectors
        this.vectors = new Vector[]{
            new Vector(1, 1),   // Front Left
            new Vector(1, -1),  // Front Right
            new Vector(1, -1),  // Back Left
            new Vector(1, 1)    // Back Right
        };
    }

    @Override
    public double[] calculateDrive(Vector correctivePower, Vector headingPower, Vector pathingPower, double robotHeading) {
        // Combine all power vectors
        Vector totalPower = new Vector();
        totalPower.setOrthogonalComponents(
            correctivePower.getXComponent() + pathingPower.getXComponent(),
            correctivePower.getYComponent() + pathingPower.getYComponent()
        );

        double x = totalPower.getXComponent();
        double y = totalPower.getYComponent();
        double turn = headingPower.getXComponent(); // Use heading power for rotation

        // Mecanum drive calculations - MATCH TELEOP SETUP
        // Your teleop uses: fl=(y+x+r), bl=(y-x+r), fr=(y-x-r), br=(y+x-r)
        double frontLeft = y + x + turn;
        double backLeft = y - x + turn;
        double frontRight = y - x - turn;
        double backRight = y + x - turn;

        // Normalize powers
        double max = Math.max(Math.abs(frontLeft), Math.max(Math.abs(frontRight),
                     Math.max(Math.abs(backLeft), Math.abs(backRight))));
        if (max > 1.0) {
            frontLeft /= max;
            frontRight /= max;
            backLeft /= max;
            backRight /= max;
        }

        // Apply max power scaling
        frontLeft *= maxPowerScaling;
        frontRight *= maxPowerScaling;
        backLeft *= maxPowerScaling;
        backRight *= maxPowerScaling;

        return new double[]{frontLeft, frontRight, backLeft, backRight};
    }

    @Override
    public void updateConstants() {
        // No constants to update for basic mecanum drive
    }

    @Override
    public void breakFollowing() {
        hw.setDrivePowers(0, 0, 0, 0);
    }

    @Override
    public void runDrive(double[] drivePowers) {
        if (drivePowers.length >= 4) {
            hw.setDrivePowers(drivePowers[0], drivePowers[1], drivePowers[2], drivePowers[3]);
        }
    }

    @Override
    public void startTeleopDrive() {
        startTeleopDrive(true);
    }

    @Override
    public void startTeleopDrive(boolean brakeMode) {
        DcMotor.ZeroPowerBehavior behavior = brakeMode ?
            DcMotor.ZeroPowerBehavior.BRAKE : DcMotor.ZeroPowerBehavior.FLOAT;

        if (hw.frontLeft != null) hw.frontLeft.setZeroPowerBehavior(behavior);
        if (hw.frontRight != null) hw.frontRight.setZeroPowerBehavior(behavior);
        if (hw.backLeft != null) hw.backLeft.setZeroPowerBehavior(behavior);
        if (hw.backRight != null) hw.backRight.setZeroPowerBehavior(behavior);
    }

    @Override
    public double xVelocity() {
        // Not implemented for basic drivetrain
        return 0;
    }

    @Override
    public double yVelocity() {
        // Not implemented for basic drivetrain
        return 0;
    }

    @Override
    public void setXVelocity(double xMovement) {
        // Not implemented for basic drivetrain
    }

    @Override
    public void setYVelocity(double yMovement) {
        // Not implemented for basic drivetrain
    }

    @Override
    public String debugString() {
        return "MecanumDrivetrain - Hardware initialized";
    }

    @Override
    public double getVoltage() {
        // Return nominal voltage (12V) - could be improved with actual battery reading
        return 12.0;
    }
}
