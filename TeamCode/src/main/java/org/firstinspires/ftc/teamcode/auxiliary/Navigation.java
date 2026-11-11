package org.firstinspires.ftc.teamcode.auxiliary;

import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public final class Navigation {
    private Navigation() {
    }

    public enum Alliance {
        RED("Red"),
        BLUE("Blue");

        private final String displayName;

        Alliance(String displayName) {
            this.displayName = displayName;
        }

        public String getDisplayName() {
            return displayName;
        }
    }

    /** Init-time selection (X = blue, B = red, either gamepad). Static, so it survives Auto -> TeleOp. */
    public static final class AllianceSelector {
        private static Alliance selected = null;

        private AllianceSelector() {
        }

        public static void update(Gamepad g1, Gamepad g2) {
            if ((g1 != null && g1.x) || (g2 != null && g2.x)) selected = Alliance.BLUE;
            if ((g1 != null && g1.b) || (g2 != null && g2.b)) selected = Alliance.RED;
        }

        public static Alliance get() {
            return selected;
        }

        public static void addTelemetry(Telemetry telemetry) {
            telemetry.addData("Alliance", selected == null
                    ? "NO ALLIANCE SELECTED - aim disabled" : selected.getDisplayName());
            telemetry.addData("Alliance select", "X = blue, B = red (either gamepad)");
        }
    }
}