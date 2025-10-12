package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

public final class TelemetryHelper {
    private final OpMode opMode;
    private final boolean isEnabled;

    public TelemetryHelper(OpMode opMode, boolean isEnabled) {
        this.opMode = opMode;
        this.isEnabled = isEnabled;
    }

    public TelemetryHelper addLine(String text) {
        if (isEnabled) opMode.telemetry.addLine(text);
        return this;
    }

    public TelemetryHelper addData(String caption, String format, Object... args) {
        if (isEnabled) opMode.telemetry.addData(caption, format, args);
        return this;
    }
}
