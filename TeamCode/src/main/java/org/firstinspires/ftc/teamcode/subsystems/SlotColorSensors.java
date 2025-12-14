package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.config.ColorCal.*;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.teamcode.util.TelemetryHelper;

/**
 * Ball detection using simple V/A thresholds.
 */
public class SlotColorSensors {

    public enum BallColor {NONE, PURPLE, GREEN, UNKNOWN}

    private final NormalizedColorSensor[] devices;
    private final TelemetryHelper tele;

    private boolean enabled = true;

    // Store last readings for telemetry
    private final double[][] lastHSVA = new double[3][4];

    public SlotColorSensors(NormalizedColorSensor[] sensors, OpMode opmode) {
        this.devices = sensors;
        this.tele = new TelemetryHelper(opmode, TELEMETRY_ENABLED);
    }

    public void setEnabled(boolean on) {
        enabled = on;
    }

    public void update() {
        if (!enabled) return;

        // Read all sensors and update telemetry
        boolean anyBall = false;
        boolean frontBall;
        for (int idx = 0; idx < devices.length && idx < 3; idx++) {
            double[] sample = readSensor(idx);
            if (sample != null) {
                lastHSVA[idx] = sample;
                if (classifyAt(idx, sample) != BallColor.NONE) {
                    anyBall = true;
                }
            }
        }

        frontBall = hasFrontBall();
        addBallTelemetry(anyBall, frontBall);
    }

    /**
     * Returns true if ANY sensor detects a ball.
     */
    public boolean hasBall() {
        for (int idx = 0; idx < devices.length && idx < 3; idx++) {
            double[] sample = readSensor(idx);
            if (sample != null && classifyAt(idx, sample) != BallColor.NONE) {
                return true;
            }
        }
        return false;
    }

    /**
     * Returns true if the "front" (slot-0 / shooter-feed) sensors detect a ball.
     * This intentionally ignores the separate intake sensor (index 2).
     */
    public boolean hasFrontBall() {
        int count = 0;
        int limit = Math.min(devices.length, Math.max(0, FRONT_SENSOR_COUNT));
        for (int idx = 0; idx < limit; idx++) {
            double[] sample = readSensor(idx);
            if (sample != null && classifyAt(idx, sample) != BallColor.NONE) {
                count++;
            }
        }
        return count >= Math.max(1, FRONT_REQUIRED_COUNT);
    }

    /**
     * Returns true if the dedicated intake sensor detects a ball.
     * This intentionally ignores the slot-0 "front" sensors.
     */
    public boolean hasIntakeBall() {
        int idx = Math.max(0, FRONT_SENSOR_COUNT);
        if (idx >= devices.length) return false;
        double[] sample = readSensor(idx);
        return sample != null && classifyAt(idx, sample) != BallColor.NONE;
    }

    /**
     * Gets the color (BALL or NONE) at a specific sensor position.
     * Used by SpindexerModel for verification.
     */
    public BallColor getColor(int slot) {
        if (slot < 0 || slot >= devices.length) {
            return BallColor.NONE;
        }
        double[] sample = readSensor(slot);
        return sample == null ? BallColor.NONE : classifyAt(slot, sample);
    }

    /**
     * Classify a sample into a ball color (or NONE).
     */
    private BallColor classifyAt(int idx, double[] sample) {
        if (sample == null) return BallColor.NONE;
        if (idx < 0 || idx >= 3) return BallColor.NONE;

        if (matchesInterval(idx, sample,
                PURPLE_H_MIN, PURPLE_H_MAX,
                PURPLE_S_MIN, PURPLE_S_MAX,
                PURPLE_V_MIN, PURPLE_V_MAX,
                PURPLE_A_MIN, PURPLE_A_MAX)) {
            return BallColor.PURPLE;
        }

        if (matchesInterval(idx, sample,
                GREEN_H_MIN, GREEN_H_MAX,
                GREEN_S_MIN, GREEN_S_MAX,
                GREEN_V_MIN, GREEN_V_MAX,
                GREEN_A_MIN, GREEN_A_MAX)) {
            return BallColor.GREEN;
        }

        return BallColor.NONE;
    }

    /**
     * Returns the best-effort color at the front (slot-0) using the two front sensors.
     *
     * <p>If only one sensor reports a color, returns that color. If the two sensors disagree,
     * returns the color from the sensor with the stronger signal.</p>
     */
    public BallColor getFrontColor() {
        int limit = Math.min(devices.length, Math.max(0, FRONT_SENSOR_COUNT));
        if (limit == 0) return BallColor.NONE;

        double[] s0 = readSensor(0);
        double[] s1 = (limit >= 2) ? readSensor(1) : null;

        BallColor a = (s0 == null) ? BallColor.NONE : classifyAt(0, s0);
        BallColor b = (s1 == null) ? BallColor.NONE : classifyAt(1, s1);

        if (a == BallColor.NONE && b == BallColor.NONE) return BallColor.NONE;
        if (a == BallColor.NONE) return b;
        if (b == BallColor.NONE) return a;
        if (a == b) return a;

        // If the two front sensors disagree, prefer the one with a stronger signal.
        return (strength(s0) >= strength(s1)) ? a : b;
    }

    /**
     * Returns the best-effort color at the intake sensor (index {@code FRONT_SENSOR_COUNT}).
     */
    public BallColor getIntakeColor() {
        int idx = Math.max(0, FRONT_SENSOR_COUNT);
        return getColor(idx);
    }

    private static boolean matchesInterval(
            int idx,
            double[] hsva,
            double[] hMin, double[] hMax,
            double[] sMin, double[] sMax,
            double[] vMin, double[] vMax,
            double[] aMin, double[] aMax
    ) {
        if (idx < 0) return false;
        if (hsva == null || hsva.length < 4) return false;
        if (hMin == null || hMax == null || sMin == null || sMax == null || vMin == null || vMax == null || aMin == null || aMax == null) {
            return false;
        }
        if (idx >= hMin.length || idx >= hMax.length || idx >= sMin.length || idx >= sMax.length
                || idx >= vMin.length || idx >= vMax.length || idx >= aMin.length || idx >= aMax.length) {
            return false;
        }

        double h = normalizeHueDeg(hsva[H]);
        return inHueRange(h, hMin[idx], hMax[idx])
                && inRange(hsva[S], sMin[idx], sMax[idx])
                && inRange(hsva[V], vMin[idx], vMax[idx])
                && inRange(hsva[A], aMin[idx], aMax[idx]);
    }

    private static boolean inRange(double v, double min, double max) {
        return v >= min && v <= max;
    }

    /**
     * Hue range helper that supports wrap-around when min > max.
     */
    private static boolean inHueRange(double hueDeg, double minDeg, double maxDeg) {
        double h = normalizeHueDeg(hueDeg);
        double min = normalizeHueDeg(minDeg);
        double max = normalizeHueDeg(maxDeg);
        if (min <= max) {
            return h >= min && h <= max;
        }
        return h >= min || h <= max;
    }

    private static double normalizeHueDeg(double hueDeg) {
        double h = hueDeg % 360.0;
        if (h < 0) h += 360.0;
        return h;
    }

    private static double strength(double[] hsva) {
        if (hsva == null || hsva.length < 4) return 0.0;
        return clamp01(hsva[A]) * clamp01(hsva[V]);
    }

    private static double clamp01(double v) {
        return Math.max(0.0, Math.min(1.0, v));
    }

    /**
     * Reads H, S, V, A from a sensor.
     */
    private double[] readSensor(int idx) {
        if (idx < 0 || idx >= devices.length) {
            return null;
        }

        try {
            NormalizedColorSensor sensor = devices[idx];
            sensor.setGain(SENSOR_GAIN[idx]);
            NormalizedRGBA rgba = sensor.getNormalizedColors();

            int r = clamp255(Math.round(rgba.red * 255f));
            int g = clamp255(Math.round(rgba.green * 255f));
            int b = clamp255(Math.round(rgba.blue * 255f));
            float[] hsv = new float[3];
            Color.RGBToHSV(r, g, b, hsv);

            return new double[]{
                    hsv[0],  // H
                    hsv[1],  // S
                    hsv[2],  // V
                    clamp01(rgba.alpha)  // A
            };
        } catch (Throwable t) {
            return null;
        }
    }

    private void addBallTelemetry(boolean anyBall) {
        tele.addLine("=== BALL SENSOR ===")
                .addData("Ball", "%s", anyBall ? "YES" : "no");

        for (int i = 0; i < Math.min(devices.length, 3); i++) {
            double[] hsva = lastHSVA[i];
            if (hsva != null) {
                BallColor c = classifyAt(i, hsva);
                tele.addData("S" + i, "H=%.0f S=%.2f V=%.3f A=%.3f [%s]",
                        hsva[H], hsva[S], hsva[V], hsva[A], c.name());
            }
        }
    }

    private void addBallTelemetry(boolean anyBall, boolean frontBall) {
        boolean intakeBall = hasIntakeBall();
        tele.addLine("=== BALL SENSOR ===")
                .addData("Ball", "%s", anyBall ? "YES" : "no")
                .addData("FrontBall", "%s", frontBall ? "YES" : "no")
                .addData("IntakeBall", "%s", intakeBall ? "YES" : "no");

        for (int i = 0; i < Math.min(devices.length, 3); i++) {
            double[] hsva = lastHSVA[i];
            if (hsva != null) {
                BallColor c = classifyAt(i, hsva);
                tele.addData("S" + i, "H=%.0f S=%.2f V=%.3f A=%.3f [%s]",
                        hsva[H], hsva[S], hsva[V], hsva[A], c.name());
            }
        }
    }

    private static int clamp255(int v) {
        return Math.max(0, Math.min(255, v));
    }

    private static float clamp01(float v) {
        return Math.max(0f, Math.min(1f, v));
    }
}
