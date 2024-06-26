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

package org.firstinspires.ftc.teamcode.George;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;


import org.firstinspires.ftc.teamcode.DeliveryFunctions;
import org.firstinspires.ftc.teamcode.DrivetrainFunctions;
import org.firstinspires.ftc.teamcode.DroneLauncherFunctions;
import org.firstinspires.ftc.teamcode.IntakeFunctions;
import org.firstinspires.ftc.teamcode.LEDFunctions;
import org.firstinspires.ftc.teamcode.RoadRunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.RoboMom;
import org.firstinspires.ftc.teamcode.VisionFunctions;
import org.firstinspires.ftc.teamcode.XDrivetrainFunctions;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name="GEORGE Teleop", group="Summer")


public class GeorgeTeleop extends RoboMom {

    private final double DEADZONE = 0.05;

    double speedScalar = 1;

    private double scoreSpeedScalar = 0.2;

    private DrivetrainFunctions drivetrainFunctions;

    static SampleMecanumDrive drive;

    private static final double HARDWARECHECK_DELAY = 1;

    private ElapsedTime hardwareCheckTimer = new ElapsedTime();

    @Override
    public void runOpMode() {
        super.runOpMode();


        drive = new SampleMecanumDrive(hardwareMap);


        drivetrainFunctions = new XDrivetrainFunctions(this);

        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);


        waitForStart();


        while (opModeIsActive()) {

            if (drivetrainFunctions.isDisabled && hardwareCheckTimer.seconds() == HARDWARECHECK_DELAY)
                drivetrainFunctions.Reinitialize();

            if (hardwareCheckTimer.seconds() >= HARDWARECHECK_DELAY)
                hardwareCheckTimer.reset();

            /**GAMEPAD 1**/

            //Store inputted desired velocity (left_stick_x, left_stick_y)

            if (Math.abs(gamepad1.left_stick_x) > DEADZONE || Math.abs(gamepad1.left_stick_y) > DEADZONE || Math.abs(gamepad1.right_stick_x) > DEADZONE || Math.abs(gamepad1.right_stick_y) > DEADZONE * 2) {
                if (Math.abs(gamepad1.right_stick_y) > DEADZONE * 2) {
                    drivetrainFunctions.Move(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x, scoreSpeedScalar);
                } else {
                    drivetrainFunctions.Move(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x, speedScalar);
                }
            } else {
                drivetrainFunctions.Stop();
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


            //MANUAL




        }
        //when opmode is NOT active
    }

}


