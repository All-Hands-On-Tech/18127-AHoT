package org.firstinspires.ftc.teamcode.drive;

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
