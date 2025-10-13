package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.Func;

import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;

import java.util.HashMap;
import java.util.Map;

public final class TelemetryHelper {
    private static Telemetry SHARED;
    private static boolean GLOBAL_ENABLED = true;
    private static final Map<String, Long> LAST = new HashMap<>();

    private final boolean isEnabled;

    public TelemetryHelper(OpMode opMode, boolean isEnabled) {
        this.isEnabled = isEnabled;
        if (SHARED == null) {
            Telemetry ftc = opMode.telemetry;
            Telemetry panels = PanelsTelemetry.INSTANCE.getFtcTelemetry();
            SHARED = new JoinedTelemetry(ftc, panels);
            SHARED.setAutoClear(true);
            SHARED.setMsTransmissionInterval(50);
        }
    }

    private static Telemetry t() {
        return SHARED;
    }

    public TelemetryHelper addLine(String text) {
        if (GLOBAL_ENABLED && isEnabled) t().addLine(text);
        return this;
    }

    public TelemetryHelper addData(String caption, String format, Object... args) {
        if (GLOBAL_ENABLED && isEnabled) t().addData(caption, format, args);
        return this;
    }

    public <T> TelemetryHelper addData(String caption, Func<T> value) {
        if (GLOBAL_ENABLED && isEnabled) t().addData(caption, value);
        return this;
    }

    public static void setGlobalEnabled(boolean enabled) {
        GLOBAL_ENABLED = enabled;
    }

    // Call once per loop
    public static void update() {
        if (GLOBAL_ENABLED && SHARED != null) SHARED.update();
    }

    public TelemetryHelper once(String key, String text) {
        if (GLOBAL_ENABLED && isEnabled && !LAST.containsKey(key)) {
            t().addLine(text);
            LAST.put(key, System.currentTimeMillis());
        }
        return this;
    }

    public <T> TelemetryHelper every(String key, long periodMs, Func<T> value) {
        if (!(GLOBAL_ENABLED && isEnabled)) return this;
        long now = System.currentTimeMillis();
        Long prevObj = LAST.get(key);
        long prev = (prevObj != null) ? prevObj : 0L;
        if (now - prev >= periodMs) {
            t().addData(key, value);
            LAST.put(key, now);
        }
        return this;
    }

    public static void reset() {
        SHARED = null;
        GLOBAL_ENABLED = true;
    }
}
