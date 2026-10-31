package org.firstinspires.ftc.teamcode.auxiliary;

import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import org.firstinspires.ftc.teamcode.drive.Alliance;

/**
 * Alliance selection during init: X button = BLUE, B button = RED, pressed on
 * either driver's gamepad. The selection lives in a static field so it
 * persists from an Autonomous OpMode's selection into the TeleOp (and across
 * OpMode restarts) within the same robot boot.
 * <p>
 * Reuses the existing {@link Alliance} enum (drive package) that Drivetrain
 * already consumes rather than introducing a duplicate type.
 */
public class AllianceSelector {

    private static Alliance selected = null;

    private AllianceSelector() {
    }

    /**
     * Poll for X/B presses. Call every init_loop iteration (callers running a
     * LinearOpMode should loop this until isStarted() while !isStopRequested()).
     */
    public static void update(Gamepad gamepad1, Gamepad gamepad2) {
        if ((gamepad1 != null && gamepad1.x) || (gamepad2 != null && gamepad2.x)) {
            selected = Alliance.BLUE;
        }
        if ((gamepad1 != null && gamepad1.b) || (gamepad2 != null && gamepad2.b)) {
            selected = Alliance.RED;
        }
    }

    /** The currently selected alliance, or null if none was ever selected. */
    public static Alliance get() {
        return selected;
    }

    /** Shows the selection (or a loud warning) plus the selection instructions. */
    public static void addTelemetry(Telemetry telemetry) {
        if (selected == null) {
            telemetry.addData("Alliance", "NO ALLIANCE SELECTED - aim and relocalize are disabled");
        } else {
            telemetry.addData("Alliance", selected.getDisplayName());
        }
        telemetry.addData("Alliance select", "X = blue, B = red (either gamepad)");
    }
}
