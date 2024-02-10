/* Copyright (c) 2017 FIRST. All rights reserved.
 * Rohan copyrighted dis
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;


import org.firstinspires.ftc.teamcode.DeliveryFunctions;
import org.firstinspires.ftc.teamcode.DrivetrainFunctions;
import org.firstinspires.ftc.teamcode.DroneLauncherFunctions;
import org.firstinspires.ftc.teamcode.IntakeFunctions;
import org.firstinspires.ftc.teamcode.RoadRunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.RoboMom;
import org.firstinspires.ftc.teamcode.VisionFunctions;

import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name="CenterStage Teleop", group="AAA")


public class CenterStageTeleOp extends RoboMom {

    double deadZone = 0.05;

    double speedScalar = 1;

    private double scoreSpeedScalar = 0.2;


    private float TARGET_DISTANCE_TO_TAG = 12;

    final double STRAFE_GAIN = 0.0271;
    final double FORWARD_GAIN = 0.0288;

    final double ROTATION_GAIN = 0.022;

    double rightTriggerPull;

    private VisionFunctions aprilTagsFunctions;

    private boolean controlsRelinquished = false;

    public enum DeliveryState{
        DELIVERY_START,
        DELIVERY_LIFT,
        DELIVERY_DUMP,
        DELIVERY_RETRACT
    }

    private final double DEADZONE = 0.1;

    private DeliveryFunctions deliveryFunctions;
    private IntakeFunctions intakeFunctions;

    private DrivetrainFunctions drivetrainFunctions;

    private DroneLauncherFunctions droneLauncherFunctions;

    private DeliveryState deliveryState = DeliveryState.DELIVERY_START;

    private final int LIFT_HIGH = 750;

    private final int SET_1_HEIGHT = 750;
    private final int SET_2_HEIGHT = 1000;

    private final int SET_3_HEIGHT = 1800;

    private final int LIFT_LOW = 0;

    private final double DUMP_TIME = 1;

    private final int LIFT_MAX = 1800;
    private final int LIFT_MIN = 0;

    private final double QUEUE_BUFFER = 0.25;

    private int tempTarget;

    private int leftMotorPosition;
    private int rightMotorPosition;

    private int targetPosition;

    private boolean dumped = false;
    private boolean secondDumped = false;
    private boolean isRunToPosition = true;

    private boolean dumping = false;
    private boolean secondDumping = false;

    private  boolean queuedB = false;

    private boolean retracting = false;

    ElapsedTime deliveryTimer = new ElapsedTime();
    ElapsedTime queueBufferTimer = new ElapsedTime();
    static DcMotor[] motors;

    static SampleMecanumDrive drive;

     private static final double HARDWARECHECK_DELAY = 1;

    private ElapsedTime hardwareCheckTimer = new ElapsedTime();

    int targetTagIDRed;
    int targetTagIDBlue;

    @Override
    public void runOpMode() {
        super.runOpMode();

        drive = new SampleMecanumDrive(hardwareMap);

        aprilTagsFunctions = new VisionFunctions(this);
        deliveryFunctions = new DeliveryFunctions(this, true);
        intakeFunctions = new IntakeFunctions(this);
        drivetrainFunctions = new DrivetrainFunctions(this);
        droneLauncherFunctions = new DroneLauncherFunctions(this);

        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        aprilTagsFunctions.startDetectingApriltags();


        waitForStart();

        targetPosition = LIFT_LOW;
        deliveryFunctions.setSlidesTargetPosition(LIFT_LOW);

        String LEDExtras = "";

        while (opModeIsActive()) {

            if (intakeFunctions.isDisabled && hardwareCheckTimer.seconds() == HARDWARECHECK_DELAY)
                intakeFunctions.Reinitialize();
            if (deliveryFunctions.isDisabled && hardwareCheckTimer.seconds() == HARDWARECHECK_DELAY)
                deliveryFunctions.Reinitialize();
            if (drivetrainFunctions.isDisabled && hardwareCheckTimer.seconds() == HARDWARECHECK_DELAY)
                drivetrainFunctions.Reinitialize();
            if (droneLauncherFunctions.isDisabled && hardwareCheckTimer.seconds() == HARDWARECHECK_DELAY)
                droneLauncherFunctions.Reinitialize();

            if (hardwareCheckTimer.seconds() >= HARDWARECHECK_DELAY)
                hardwareCheckTimer.reset();

            /**GAMEPAD 1**/

            //Store inputted desired velocity (left_stick_x, left_stick_y)
            if (!controlsRelinquished) {

                if (Math.abs(gamepad1.left_stick_x) > deadZone || Math.abs(gamepad1.left_stick_y) > deadZone || Math.abs(gamepad1.right_stick_x) > deadZone || Math.abs(gamepad1.right_stick_y) > deadZone*2) {
                    if(Math.abs(gamepad1.right_stick_y) > deadZone*2) {
                        drivetrainFunctions.Move(gamepad1.left_stick_x, gamepad1.right_stick_y, gamepad1.right_stick_x, scoreSpeedScalar);
                    }else{
                        drivetrainFunctions.Move(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x, speedScalar);
                    }
                } else {
                    drivetrainFunctions.Stop();
                }
                if (gamepad1.b)
                    droneLauncherFunctions.ReleaseDrone();
            }


            if(gamepad1.x) {
                LEDExtras = "BACK";
            }
            if(gamepad1.y) {
                LEDExtras = "PARTY";
            }
            if (gamepad1.a){
                LEDExtras = "";
            }
            drivetrainFunctions.setLEDColor(deliveryFunctions.detectFrontPixelColor(), deliveryFunctions.detectBackPixelColor(), LEDExtras);
            telemetry.addData("Color sensors", "%s %s %s", deliveryFunctions.detectFrontPixelColor(), deliveryFunctions.detectBackPixelColor(), LEDExtras);
            telemetry.addData("Current Pattern", "%s", drivetrainFunctions.getLEDColor());
            if(gamepad1.dpad_left){
                targetTagIDRed = aprilTagsFunctions.RED_1_TAG;
                targetTagIDBlue = aprilTagsFunctions.BLUE_1_TAG;
            }
            if(gamepad1.dpad_down){
                targetTagIDRed = aprilTagsFunctions.RED_2_TAG;
                targetTagIDBlue = aprilTagsFunctions.BLUE_2_TAG;
            }
            if(gamepad1.dpad_right){
                targetTagIDRed = aprilTagsFunctions.RED_3_TAG;
                targetTagIDBlue = aprilTagsFunctions.BLUE_3_TAG;
            }


            if (aprilTagsFunctions.DetectAprilTag(targetTagIDBlue) || aprilTagsFunctions.DetectAprilTag(targetTagIDRed)) {
                telemetry.addData("Found", "ID %d (%s)", aprilTagsFunctions.detectedTag.id, aprilTagsFunctions.detectedTag.metadata.name);
                telemetry.addData("Range", "%5.1f inches", aprilTagsFunctions.detectedTag.ftcPose.range);
                telemetry.addData("Bearing", "%3.0f degrees", aprilTagsFunctions.detectedTag.ftcPose.bearing);
                telemetry.addData("Yaw", "%3.0f degrees", aprilTagsFunctions.detectedTag.ftcPose.yaw);
                telemetry.addData("X delta", "%3.0f inches", aprilTagsFunctions.detectedTag.ftcPose.x);

                if (gamepad1.dpad_left || gamepad1.dpad_down || gamepad1.dpad_right) {

                    double x = STRAFE_GAIN * aprilTagsFunctions.detectedTag.ftcPose.yaw;
                    double y = -FORWARD_GAIN * aprilTagsFunctions.detectedTag.ftcPose.range;
                    double bearing = -ROTATION_GAIN * aprilTagsFunctions.detectedTag.ftcPose.bearing;

                    telemetry.addData("x: ", x);
                    telemetry.addData("y: ", y);
                    telemetry.addData("bearing: ", bearing);

                    drivetrainFunctions.Move((float) x, (float) y, (float) bearing, 1);
                } else {
                    controlsRelinquished = false;
                }

            } else {          //align to point (pose of aprilTag)

            }

            telemetry.update();


            //slow down power if bumper is pressed
            if (gamepad1.left_bumper) {
                speedScalar = 0.5;
            } else if (gamepad1.right_bumper) {
                speedScalar = 0.8;
            } else {
                speedScalar = 1;
            }

            //Gamepad 2

            targetPosition = deliveryFunctions.getMotorTargetPosition();

            leftMotorPosition = deliveryFunctions.getMotorPositionByIndex(0);
            rightMotorPosition = deliveryFunctions.getMotorPositionByIndex(1);


            switch (deliveryState) {
                case DELIVERY_START:
                    if (gamepad2.a) {
                        deliveryState = DeliveryState.DELIVERY_LIFT;
                        targetPosition = SET_1_HEIGHT;
                    }
                    if (gamepad2.x) {
                        deliveryState = DeliveryState.DELIVERY_LIFT;
                        targetPosition = SET_2_HEIGHT;
                    }
                    if (gamepad2.y) {
                        deliveryState = DeliveryState.DELIVERY_LIFT;
                        targetPosition = SET_3_HEIGHT;
                    }

                    if (gamepad2.b && !dumped && deliveryFunctions.getMotorPositionByIndex(0) > deliveryFunctions.CARRIAGE_OUTSIDE_CHASSIS) {
                        deliveryState = DeliveryState.DELIVERY_DUMP;
                        dumped = true;
                        deliveryTimer.reset();
                        deliveryFunctions.Dump(1);
                        secondDumped = false;
                    }
                    break;

                case DELIVERY_LIFT:

                    //if both motors are within stop threshold
                    if
                    (targetPosition - leftMotorPosition <= deliveryFunctions.TICK_STOP_THRESHOLD
                            &&
                            targetPosition - rightMotorPosition <= deliveryFunctions.TICK_STOP_THRESHOLD) {
                        deliveryState = DeliveryState.DELIVERY_DUMP;
                        dumped = false;
                        secondDumped = false;
                    } else {
                        //Still going
                    }
                    break;


                case DELIVERY_DUMP:

                    if (gamepad2.b && queueBufferTimer.seconds() > QUEUE_BUFFER) {
                        queueBufferTimer.reset();
                        queuedB = true;
                    }

                    if(dumping){
                        if(deliveryTimer.seconds() < DUMP_TIME/3){
                            deliveryFunctions.OpenHolderServoByIndex(0);
                        }
                        //CLOSE 1
                        else if(deliveryTimer.seconds() < DUMP_TIME){
                            deliveryFunctions.CloseHolderServoByIndex(0);
                        }
                        //OPEN 2
                        else if(deliveryTimer.seconds() < DUMP_TIME * 1.5){
                            deliveryFunctions.OpenHolderServoByIndex(1);
                        } else if(deliveryTimer.seconds() > DUMP_TIME * 1.5){
                            dumped = true;
                            dumping = false;
                        }
                    }

                    if(secondDumping){
                        if(deliveryTimer.seconds() < DUMP_TIME/2){
                            deliveryFunctions.OpenHolderServoByIndex(0);
                        } else if(deliveryTimer.seconds() > DUMP_TIME){
                            deliveryFunctions.CloseHolderServoByIndex(0);
                            secondDumped = true;
                            secondDumping = false;
                        }
                    }

                    if ((gamepad2.b || queuedB) && !dumped && !dumping) {
                        queuedB = false;
                        dumping = true;
                        deliveryTimer.reset();
                        queueBufferTimer.reset();
                    }

                    if (dumped && deliveryTimer.seconds() >= DUMP_TIME && (gamepad2.b || queuedB) && !secondDumped && !secondDumping) {
                        queuedB = false;
                        secondDumping = true;
                        deliveryTimer.reset();
                        queueBufferTimer.reset();
                    }

                    if (dumped && deliveryTimer.seconds() >= DUMP_TIME && (gamepad2.b || queuedB) && secondDumped) {
                        queuedB = false;
                        dumped = false;
                        secondDumped = false;
                        deliveryTimer.reset();
                        retracting = true;
                        deliveryState = DeliveryState.DELIVERY_RETRACT;
                        tempTarget = deliveryFunctions.getMotorPositionByIndex(1);
                        deliveryFunctions.SetWristPosition(deliveryFunctions.servoDodge);
                    }

                    break;
                case DELIVERY_RETRACT:
                    retracting = true;
                    if(deliveryTimer.seconds() <= DUMP_TIME + 1){
                        targetPosition = deliveryFunctions.CARRIAGE_OUTSIDE_CHASSIS - 250;
                        deliveryFunctions.SetWristPosition(deliveryFunctions.servoDodge);
                        break;
                    }

                    if (deliveryFunctions.getMotorPositionByIndex(0) < deliveryFunctions.CARRIAGE_DODGE) {
                        deliveryFunctions.SetWristPosition(deliveryFunctions.servoIn);
                    } else {
                        deliveryFunctions.SetWristPosition(deliveryFunctions.servoDodge);
                    }

                    //if both motors are within stop threshold
                    if
                    (targetPosition + leftMotorPosition <= deliveryFunctions.TICK_STOP_THRESHOLD
                            &&
                            targetPosition + rightMotorPosition <= deliveryFunctions.TICK_STOP_THRESHOLD) {

                        deliveryState = DeliveryState.DELIVERY_START;
                        retracting = false;
                    } else {
                        //Still Moving
                    }
                    break;
            }

            telemetry.addData("Delivery State: ", deliveryState);

            //MANUAL
            if (Math.abs(gamepad2.right_stick_y) >= DEADZONE || Math.abs(gamepad2.left_stick_y) >= DEADZONE) {
                deliveryState = DeliveryState.DELIVERY_START;
                retracting = false;

                if (leftMotorPosition > deliveryFunctions.CARRIAGE_OUTSIDE_CHASSIS) {
                    targetPosition -= gamepad2.left_stick_y * 15;
                } else {
                    targetPosition -= gamepad2.left_stick_y * 10;
                }
            }

            targetPosition = Math.max(LIFT_MIN, Math.min(LIFT_MAX, targetPosition));
            deliveryFunctions.setSlidesTargetPosition(targetPosition);

            deliveryFunctions.PControlPower();

            if (!retracting) {
                deliveryFunctions.WristMovementByLiftPosition();
            }


            //telemetry.addData("Target Position: ", targetPosition);
            telemetry.addData("Target Position in DeliveryFunctions: ", deliveryFunctions.getMotorTargetPosition());
            telemetry.addData("Left Motor Position: ", leftMotorPosition);
            telemetry.addData("Right Motor Position: ", rightMotorPosition);

            if (gamepad2.left_bumper) {
                intakeFunctions.OutakeFromIntake(-1f);

            } else if (gamepad2.left_trigger >= 0.05) {
                intakeFunctions.RunIntakeMotor(gamepad2.left_trigger);
                deliveryFunctions.OpenHolderServoByIndex(0);
                deliveryFunctions.OpenHolderServoByIndex(1);
            } else if(!dumping && !secondDumping){
                intakeFunctions.StopIntakeMotor();
                deliveryFunctions.CloseHolderServoByIndex(0);
                deliveryFunctions.CloseHolderServoByIndex(1);
            }

            if (gamepad2.right_bumper){
                deliveryFunctions.OpenHolderServoByIndex(0);
                deliveryFunctions.OpenHolderServoByIndex(1);
            }

        }
        //when opmode is NOT active
        safeStop();

    }





    private void safeStop(){
        drivetrainFunctions.Stop();
        deliveryFunctions.setSlidesPower(0);
        intakeFunctions.RunIntakeMotor(0);
    }

}
