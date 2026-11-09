package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.controllers.Controller;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Vector2D;
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

    public static MecanumConfig drivetrainConfig = new MecanumConfig(c -> {
        c.frontLeftName.set("front_left_drive");
        c.frontRightName.set("front_right_drive");
        c.backLeftName.set("back_left_drive");
        c.backRightName.set("back_right_drive");
        c.frontLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.frontRightDirection.set(DcMotorSimple.Direction.FORWARD);
        c.backLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.backRightDirection.set(DcMotorSimple.Direction.FORWARD);
    });

    public static PinpointConfig localizerConfig = new PinpointConfig(c -> {
        c.name.set("pinpoint");
        c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        c.xPodOffset.set(-4.176374420406312);
        c.yPodOffset.set(5.121319387841413);
        c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.REVERSED);
        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.globalDistanceUnit.set(DistanceUnit.INCH);
        c.offsetUnits.set(DistanceUnit.INCH);
    });

    public static ForesightConfig foresightConfig = new ForesightConfig(
            c -> {
                Controller primaryTranslationalForward = Controller.proportional(0.35979756485420683);
                Controller secondaryTranslationalForward = Controller.proportional(0.13293558624563354);
                Controller primaryTranslationalLateral = Controller.proportional(0.4938541141060367);
                Controller secondaryTranslationalLateral = Controller.proportional(0.18246589913721697);

                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

                c.coast.set(Controller.proportionalFeedforward(0.016848169468211504));
                c.brake.set(Controller.proportionalFeedforward(0.014320944047979779));

                c.headingFeedback.set(Controller.proportional(4.343946914157177));
                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.043105816545582036, 0.005989098007638411));

                c.linearBrakeCoefficients.set(Matrix.diag(0.05653834474887369, 0.05540956503868474));
                c.quadraticBrakeCoefficients.set(Matrix.diag(0.0014240920626589402, 0.0016700060842905985));

                c.maxAchievableForwardVelocity.set(70.37120240258619);
                c.maxAchievableStrafeVelocity.set(58.5072347415458);
                c.naturalForwardDeceleration.set(78.93134671579475);
                c.naturalStrafeDeceleration.set(95.97511057332368);
            }
    );

    public static Follower create(HardwareMap h) {
        return new Follower(
                new PinpointLocalizer(h, localizerConfig),
                new Mecanum(h, drivetrainConfig),
                new Foresight(foresightConfig)
        );
    }
}
