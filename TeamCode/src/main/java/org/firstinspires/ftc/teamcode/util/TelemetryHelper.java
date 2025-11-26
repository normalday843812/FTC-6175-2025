package org.firstinspires.ftc.teamcode.util;

import static org.firstinspires.ftc.teamcode.config.GlobalConfig.ENABLE_TELEMETRY;

import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.HashMap;
import java.util.Map;

/**
 * Multiplexed telemetry for FTC Driver Station and Panels.
 *
 * <p>Creates a process-wide {@link Telemetry} that fans out to both targets via {@link JoinedTelemetry}.
 * Construct per-subsystem helpers to guard writes with a local flag. Flush once per loop with {@link #update()}.
 * Call {@link #reset()} when an OpMode ends.</p>
 *
 * <h3>Usage</h3>
 * <pre>{@code
 * TelemetryHelper tele = new TelemetryHelper(this, SubsystemConfig.TELEMETRY_ENABLED);
 * tele.addData("x", "%.2f", pose.x)
 *     .addData("y", "%.2f", pose.y);
 * TelemetryHelper.update(); // once per loop
 * }</pre>
 */
public final class TelemetryHelper {
    private static Telemetry SHARED;
    private static boolean GLOBAL_ENABLED = ENABLE_TELEMETRY;
    private static final Map<String, Long> LAST = new HashMap<>();

    private final boolean isEnabled;

    /**
     * Creates or reuses the shared telemetry and sets per-helper enable state.
     *
     * <p>The first constructed helper builds a {@link JoinedTelemetry} from the OpMode's
     * {@code opMode.telemetry} and Panels' {@link PanelsTelemetry#INSTANCE}, then enables auto-clear
     * and sets a 50 ms transmission interval.</p>
     *
     * @param opMode    current OpMode; used only on first construction to seed the shared telemetry
     * @param isEnabled per-helper gate for all add calls
     */
    public TelemetryHelper(OpMode opMode, boolean isEnabled) {
        this.isEnabled = isEnabled;
        if (SHARED == null) {
            Telemetry ftc = opMode.telemetry;
            Telemetry panels = PanelsTelemetry.INSTANCE.getFtcTelemetry();
            SHARED = new JoinedTelemetry(panels, ftc);
            SHARED.setAutoClear(true);
            SHARED.setMsTransmissionInterval(250);
        }
    }

    private static Telemetry t() {
        return SHARED;
    }

    /**
     * Adds a line to telemetry if globally and locally enabled.
     *
     * @param text line content
     * @return this helper for chaining
     */
    public TelemetryHelper addLine(String text) {
        if (GLOBAL_ENABLED && isEnabled) t().addLine(text);
        return this;
    }

    /**
     * Adds a formatted datum. Equivalent to {@link Telemetry#addData(String, String, Object...)}.
     *
     * @param caption label
     * @param format  printf-style format string
     * @param args    format arguments
     * @return this helper for chaining
     */
    public TelemetryHelper addData(String caption, String format, Object... args) {
        if (GLOBAL_ENABLED && isEnabled) t().addData(caption, format, args);
        return this;
    }

    /**
     * Adds a lazily evaluated datum. {@code value} is invoked at {@link #update()} time.
     * Reduces per-loop formatting cost.
     *
     * @param caption label
     * @param value   supplier evaluated when the frame transmits
     * @param <T>     value type
     * @return this helper for chaining
     */
    public <T> TelemetryHelper addData(String caption, Func<T> value) {
        if (GLOBAL_ENABLED && isEnabled) t().addData(caption, value);
        return this;
    }

    /**
     * Globally enables or disables all telemetry writes from all helpers.
     * Use during shutdown to prevent stray writes from background tasks.
     *
     * @param enabled {@code true} to allow writes, {@code false} to turn all writes into no-ops
     */
    public static void setGlobalEnabled(boolean enabled) {
        GLOBAL_ENABLED = enabled;
    }

    /**
     * Flushes the shared telemetry to both targets. Call exactly once per loop.
     * No effect if globally disabled or not yet initialized.
     */
    // Call once per loop
    public static void update() {
        if (GLOBAL_ENABLED && SHARED != null) SHARED.update();
    }

    /**
     * Emits a line once per unique key for the lifetime of the process or until {@link #reset()}.
     *
     * @param key  uniqueness key
     * @param text line content
     * @return this helper for chaining
     */
    public TelemetryHelper once(String key, String text) {
        if (GLOBAL_ENABLED && isEnabled && !LAST.containsKey(key)) {
            t().addLine(text);
            LAST.put(key, System.currentTimeMillis());
        }
        return this;
    }

    /**
     * Emits a lazily evaluated datum at most once per period for the given key.
     * Useful for throttling chatty values while preserving freshness.
     *
     * @param key      throttle key
     * @param periodMs minimum interval between emissions in milliseconds
     * @param value    supplier evaluated when the interval elapses
     * @param <T>      value type
     * @return this helper for chaining
     */
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

    /**
     * Clears shared state between OpModes.
     *
     * <p>Drops the shared telemetry reference, re-enables global writes, and clears
     * one-time/throttle bookkeeping so keys can be used again in the next OpMode.</p>
     */
    public static void reset() {
        SHARED = null;
        GLOBAL_ENABLED = true;
    }
}
