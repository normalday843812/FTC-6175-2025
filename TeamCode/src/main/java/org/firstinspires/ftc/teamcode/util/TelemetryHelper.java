package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;

public final class TelemetryHelper {
    private final Telemetry telemetry;
    private final boolean isEnabled;

    public TelemetryHelper(OpMode opMode, boolean isEnabled) {
        this.isEnabled = isEnabled;
        Telemetry ftc = opMode.telemetry;
        Telemetry panels = PanelsTelemetry.INSTANCE.getFtcTelemetry();
        this.telemetry = new JoinedTelemetry(ftc, panels);
    }

    public TelemetryHelper addLine(String text) {
        if (isEnabled) telemetry.addLine(text);
        return this;
    }

    public TelemetryHelper addData(String caption, String format, Object... args) {
        if (isEnabled) telemetry.addData(caption, format, args);
        return this;
    }

    public void update() {
        if (isEnabled) telemetry.update();
    }
}
