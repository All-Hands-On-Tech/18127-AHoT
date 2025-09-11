package org.firstinspires.ftc.teamcode.ZSupport;

import com.qualcomm.robotcore.hardware.DcMotor;

public class Claw {
    private DcMotor lowerLeft, lowerRight, gripLeft, gripRight;

    public Claw(DcMotor lowerLeft, DcMotor lowerRight, DcMotor gripLeft, DcMotor gripRight) {
        this.lowerLeft = lowerLeft;
        this.lowerRight = lowerRight;
        this.gripLeft = gripLeft;
        this.gripRight = gripRight;
    }

    /**
     * Lowers the claw.
     */
    public void lower() {
        if (lowerLeft != null) lowerLeft.setPower(-1);
        if (lowerRight != null) lowerRight.setPower(-1);
    }

    /**
     * Raises the claw.
     */
    public void raise() {
        if (lowerLeft != null) lowerLeft.setPower(1);
        if (lowerRight != null) lowerRight.setPower(1);
    }

    /**
     * Stops vertical movement of the claw.
     */
    public void stopVertical() {
        if (lowerLeft != null) lowerLeft.setPower(0);
        if (lowerRight != null) lowerRight.setPower(0);
    }

    /**
     * Opens the claw grip.
     */
    public void open() {
        if (gripLeft != null) gripLeft.setPower(1);
        if (gripRight != null) gripRight.setPower(1);
    }

    /**
     * Closes the claw grip.
     */
    public void close() {
        if (gripLeft != null) gripLeft.setPower(-1);
        if (gripRight != null) gripRight.setPower(-1);
    }

    /**
     * Stops the claw grip motors.
     */
    public void stopGrip() {
        if (gripLeft != null) gripLeft.setPower(0);
        if (gripRight != null) gripRight.setPower(0);
    }
}

