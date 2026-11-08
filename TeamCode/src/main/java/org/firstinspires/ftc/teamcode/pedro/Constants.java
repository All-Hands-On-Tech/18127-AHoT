package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.controllers.Controller;
import com.pedropathing.follower.Follower;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import org.firstinspires.ftc.teamcode.config.RobotConstants;

/**
 * Pedro Pathing configuration. Tuner OpModes (see Tuning.java) generate
 * snippets for this file: run them and paste the output over the configs below.
 */
public class Constants {

    // Wheel directions match RobotHardware: both right motors REVERSED.
    public static final MecanumConfig drivetrainConfig = new MecanumConfig(c -> {
        c.frontLeftName.set(RobotConstants.FRONT_LEFT_DRIVE);
        c.backLeftName.set(RobotConstants.BACK_LEFT_DRIVE);
        c.frontRightName.set(RobotConstants.FRONT_RIGHT_DRIVE);
        c.backRightName.set(RobotConstants.BACK_RIGHT_DRIVE);
        c.frontLeftDirection.set(DcMotorSimple.Direction.FORWARD);
        c.backLeftDirection.set(DcMotorSimple.Direction.FORWARD);
        c.frontRightDirection.set(DcMotorSimple.Direction.REVERSE);
        c.backRightDirection.set(DcMotorSimple.Direction.REVERSE);
    });

    public static final PinpointConfig localizerConfig = new PinpointConfig(c -> {
        c.name.set(RobotConstants.PINPOINT);
        c.podType.set(RobotConstants.PINPOINT_POD_TYPE);
        c.xPodDirection.set(RobotConstants.PINPOINT_X_POD_DIRECTION);
        c.yPodDirection.set(RobotConstants.PINPOINT_Y_POD_DIRECTION);
        c.xPodOffset.set(RobotConstants.PINPOINT_X_POD_OFFSET_MM);
        c.yPodOffset.set(RobotConstants.PINPOINT_Y_POD_OFFSET_MM);
        c.offsetUnits.set(DistanceUnit.MM);
        c.globalDistanceUnit.set(DistanceUnit.INCH);
    });

    // STARTER gains only (the five required fields; rest is library-default).
    // TODO: run the Foresight Tuner and replace this lambda with its output.
    public static final ForesightConfig foresightConfig = new ForesightConfig(c -> {
        c.headingFeedback.set(Controller.proportional(0.03));
        c.forwardTranslational.set(Controller.piecewise(Controller.proportional(0.02)).put(2.5, Controller.proportional(0.1)));
        c.strafeTranslational.set(Controller.piecewise(Controller.proportional(0.02)).put(2.5, Controller.proportional(0.1)));
        c.coast.set(Controller.proportionalFeedforward(0.033));
        c.brake.set(Controller.proportionalFeedforward(0.028));
    });

    public static Follower create(HardwareMap hardwareMap) {
        return new Follower(
                new PinpointLocalizer(hardwareMap, localizerConfig),
                new Mecanum(hardwareMap, drivetrainConfig),
                new Foresight(foresightConfig));
    }
}
