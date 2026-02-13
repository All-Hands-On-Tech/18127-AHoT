///*
//Copyright (c) 2024 Limelight Vision
//
//All rights reserved.
//
//Redistribution and use in source and binary forms, with or without modification,
//are permitted (subject to the limitations in the disclaimer below) provided that
//the following conditions are met:
//
//Redistributions of source code must retain the above copyright notice, this list
//of conditions and the following disclaimer.
//
//Redistributions in binary form must reproduce the above copyright notice, this
//list of conditions and the following disclaimer in the documentation and/or
//other materials provided with the distribution.
//
//Neither the name of FIRST nor the names of its contributors may be used to
//endorse or promote products derived from this software without specific prior
//written permission.
//
//NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
//LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
//"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
//THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
//ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
//FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
//DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
//SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
//CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
//TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
//THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//*/
//package org.firstinspires.ftc.teamcode.TeleOp;
//
//import com.pedropathing.follower.Follower;
//import com.pedropathing.ftc.FTCCoordinates;
//import com.pedropathing.geometry.PedroCoordinates;
//import com.pedropathing.geometry.Pose;
//import com.qualcomm.hardware.limelightvision.LLResult;
//import com.qualcomm.hardware.limelightvision.LLResultTypes;
//import com.qualcomm.hardware.limelightvision.LLStatus;
//import com.qualcomm.hardware.limelightvision.Limelight3A;
//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//
//import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
//import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
//import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
//import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
////import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
//
//import java.util.List;
//
///*
// * This OpMode illustrates how to use the Limelight3A Vision Sensor.
// *
// * @see <a href="https://limelightvision.io/">Limelight</a>
// *
// * Notes on configuration:
// *
// *   The device presents itself, when plugged into a USB port on a Control Hub as an ethernet
// *   interface.  A DHCP server running on the Limelight automatically assigns the Control Hub an
// *   ip address for the new ethernet interface.
// *
// *   Since the Limelight is plugged into a USB port, it will be listed on the top level configuration
// *   activity along with the Control Hub Portal and other USB devices such as webcams.  Typically
// *   serial numbers are displayed below the device's names.  In the case of the Limelight device, the
// *   Control Hub's assigned ip address for that ethernet interface is used as the "serial number".
// *
// *   Tapping the Limelight's name, transitions to a new screen where the user can rename the Limelight
// *   and specify the Limelight's ip address.  Users should take care not to confuse the ip address of
// *   the Limelight itself, which can be configured through the Limelight settings page via a web browser,
// *   and the ip address the Limelight device assigned the Control Hub and which is displayed in small text
// *   below the name of the Limelight on the top level configuration screen.
// */
//@Disabled
//@TeleOp(name = "Limelight Testing", group = "ZTesting")
//public class LimelightTesting extends LinearOpMode {
//
//    private Limelight3A limelight;
//    public static Follower follower;
//
//    @Override
//    public void runOpMode() throws InterruptedException
//    {
//        follower = Constants.createFollower(hardwareMap);
//        limelight = hardwareMap.get(Limelight3A.class, "limelight");
//
//        telemetry.setMsTransmissionInterval(11);
//
//        limelight.pipelineSwitch(0);
//
//        /*
//         * Starts polling for data.  If you neglect to call start(), getLatestResult() will return null.
//         */
//        limelight.start();
//
//        follower.setStartingPose(new Pose(72,72,0));
//        follower.startTeleopDrive();
//        follower.update();
//
//        telemetry.addData(">", "Robot Ready.  Press Play.");
//        telemetry.update();
//        waitForStart();
//
//        while (opModeIsActive()) {
////            LLStatus status = limelight.getStatus();
////            telemetry.addData("Name", "%s",
////                    status.getName());
////            telemetry.addData("LL", "Temp: %.1fC, CPU: %.1f%%, FPS: %d",
////                    status.getTemp(), status.getCpu(),(int)status.getFps());
////            telemetry.addData("Pipeline", "Index: %d, Type: %s",
////                    status.getPipelineIndex(), status.getPipelineType());
//
//            LLResult result = limelight.getLatestResult();
//            if (result.isValid()) {
//                // Access general information
//                Pose3D botpose = result.getBotpose();
//                Pose tagPose = getRobotPoseFromCamera(botpose);
//                //NEEDS TO BE FIXED ANGLE WRONG
//                telemetry.addLine(String.format("limelight location: (%5.1f,%5.1f,%5.1f)", tagPose.getX(), tagPose.getY(), tagPose.getHeading()));
////                double captureLatency = result.getCaptureLatency();
////                double targetingLatency = result.getTargetingLatency();
//////                double parseLatency = result.getParseLatency();
////                telemetry.addData("LL Latency", captureLatency + targetingLatency);
////                telemetry.addData("Parse Latency", parseLatency);
////                telemetry.addData("PythonOutput", java.util.Arrays.toString(result.getPythonOutput()));
////
////                telemetry.addData("Botpose", botpose.toString());
////
////                // Access April Tag results
////                List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();
////                for (LLResultTypes.FiducialResult fr : fiducialResults) {
////                    telemetry.addData("Fiducial", "ID: %d, Family: %s, X: %.2f, Y: %.2f", fr.getFiducialId(), fr.getFamily(), fr.getTargetXDegrees(), fr.getTargetYDegrees());
////                }
//            } else {
//                telemetry.addData("Limelight", "No data available");
//                telemetry.addData("num tags: ", result.getBotposeTagCount());
//            }
//
//            Pose followerPose = follower.getPose();
//            telemetry.addLine(String.format("follower location:  (%5.1f,%5.1f, %5.1f)", followerPose.getX(), followerPose.getY(), followerPose.getHeading()));
//
//
//            //follower.setTeleOpDrive(-0.4*gamepad1.left_stick_y, -0.4*gamepad1.left_stick_x, -0.4*gamepad1.right_stick_x, true);
//            follower.update();
//
//            telemetry.update();
//        }
//        limelight.stop();
//    }
//
//    private Pose getRobotPoseFromCamera(Pose3D limeLightPose) {
//        //Fill this out to get the robot Pose from the camera's output (apply any filters if you need to using follower.getPose() for fusion)
//        //Pedro Pathing has built-in KalmanFilter and LowPassFilter classes you can use for this
//
//        //Use this to convert standard FTC coordinates to standard Pedro Pathing coordinates
//        return new Pose(72+limeLightPose.getPosition().y/0.0254, 72-limeLightPose.getPosition().x/0.0254, limeLightPose.getOrientation().getYaw());
//    }
//}
