package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.config.RgbIndicatorConfig.*;

import com.qualcomm.robotcore.hardware.Servo;

public class RgbIndicator {
    // Effects
    public enum Fx {SOLID, BLINK, STROBE, SWEEP}

    public enum OffBehavior {OFF, WHITE}

    public static final class Pattern {
        public final double hueDeg;
        public final Fx fx;
        public final int periodMs;
        public final double duty;
        public final double sweepSpanDeg;
        public final OffBehavior offBehavior;

        public Pattern(double hueDeg, Fx fx, int periodMs, double duty, double sweepSpanDeg, OffBehavior offBehavior) {
            this.hueDeg = hueDeg;
            this.fx = fx;
            this.periodMs = periodMs;
            this.duty = duty;
            this.sweepSpanDeg = sweepSpanDeg;
            this.offBehavior = offBehavior;
        }

        public static Pattern solid(double hueDeg) {
            return new Pattern(hueDeg, Fx.SOLID, 0, 1.0, 0.0, OffBehavior.OFF);
        }
    }

    private final Servo rgb;

    private Pattern base = Pattern.solid(120);
    private Pattern overlay = null;
    private Double overlayRawPos = null;
    private long overlayUntilMs = 0;

    public RgbIndicator(Servo rgbIndicator) {
        this.rgb = rgbIndicator;
    }

    public void setBasePattern(Pattern p) {
        if (p != null) base = p;
    }

    public void notifyPattern(Pattern p, long durationMs) {
        if (p == null || durationMs <= 0) return;
        overlay = p;
        overlayRawPos = null;
        overlayUntilMs = nowMs() + durationMs;
        renderPattern(p, nowMs());
    }

    public void flash(int r, int g, int b) {
        flash(r, g, b, 300);
    }

    public void flash(int r, int g, int b, long durationMs) {
        overlay = null;
        overlayRawPos = rgbToPos(r, g, b);
        overlayUntilMs = nowMs() + Math.max(1, durationMs);
        writePos(overlayRawPos); // immediate show
    }

    public void update() {
        final long now = nowMs();
        if ((overlay != null || overlayRawPos != null) && now > overlayUntilMs) {
            overlay = null;
            overlayRawPos = null;
        }
        if (overlayRawPos != null) {
            writePos(overlayRawPos);
            return;
        }
        renderPattern(overlay != null ? overlay : base, now);
    }

    public void setColor(int r, int g, int b) {
        overlay = null;
        overlayRawPos = null;
        writePos(rgbToPos(r, g, b));
    }

    public void setHueDeg(double hue) {
        overlay = null;
        overlayRawPos = null;
        writePos(hueToPos(hue));
    }

    public void setWhite() {
        overlay = null;
        overlayRawPos = null;
        writePos(1.0);
    }

    public void setOff() {
        overlay = null;
        overlayRawPos = null;
        writePos(OFF_POS);
    }

    public void setPos(double pos01) {
        overlay = null;
        overlayRawPos = null;
        writePos(clamp01(pos01));
    }

    public void setGreen() {
        rgb.setPosition(GREEN_POS);
    }

    public void setRed() {
        rgb.setPosition(RED_POS);
    }

    private void renderPattern(Pattern p, long now) {
        switch (p.fx) {
            case SOLID:
                writePos(hueToPos(p.hueDeg));
                break;

            case BLINK: {
                final int T = Math.max(1, p.periodMs);
                final boolean on = (now % T) < (int) (T * clamp01(p.duty));
                if (on) writePos(hueToPos(p.hueDeg));
                else writePos(p.offBehavior == OffBehavior.WHITE ? 1.0 : OFF_POS);
                break;
            }

            case STROBE: {
                final int T = Math.max(1, p.periodMs);
                final boolean on = (now % T) < Math.max(30, (int) (T * clamp01(p.duty)));
                if (on) writePos(hueToPos(p.hueDeg));
                else writePos(p.offBehavior == OffBehavior.WHITE ? 1.0 : OFF_POS);
                break;
            }

            case SWEEP: {
                final int T = Math.max(1, p.periodMs);
                final double phase = (now % T) / (double) T;     // 0..1
                final double h = p.hueDeg + p.sweepSpanDeg * Math.sin(2 * Math.PI * phase);
                writePos(hueToPos(h));
                break;
            }
        }
    }

    private void writePos(double pos01) {
        rgb.setPosition(clamp01(pos01));
    }

    private static long nowMs() {
        return System.nanoTime() / 1_000_000L;
    }

    private static double clamp01(double x) {
        return x < 0 ? 0 : (x > 1 ? 1 : x);
    }

    private double hueToPos(double hueDeg) {
        double h = Math.max(0.0, Math.min(300.0, hueDeg));
        return getComputedPosition(h);
    }

    public double rgbToPos(int red8bit, int green8bit, int blue8bit) {
        red8bit = Math.max(0, Math.min(255, red8bit));
        green8bit = Math.max(0, Math.min(255, green8bit));
        blue8bit = Math.max(0, Math.min(255, blue8bit));

        double redNorm = red8bit / 255.0;
        double greenNorm = green8bit / 255.0;
        double blueNorm = blue8bit / 255.0;

        double maxChannel = Math.max(redNorm, Math.max(greenNorm, blueNorm));
        double minChannel = Math.min(redNorm, Math.min(greenNorm, blueNorm));
        double deltaChannel = maxChannel - minChannel;

        double hueDegrees;
        double saturationFraction = (maxChannel == 0) ? 0.0 : (deltaChannel / maxChannel);

        if (deltaChannel == 0) {
            hueDegrees = 0.0;
        } else if (maxChannel == redNorm) {
            hueDegrees = 60.0 * ((greenNorm - blueNorm) / deltaChannel);
        } else if (maxChannel == greenNorm) {
            hueDegrees = 60.0 * (2.0 + (blueNorm - redNorm) / deltaChannel);
        } else {
            hueDegrees = 60.0 * (4.0 + (redNorm - greenNorm) / deltaChannel);
        }
        if (hueDegrees < 0) hueDegrees += 360.0;

        if (maxChannel < 0.05) return OFF_POS;
        if (saturationFraction < 0.12 && maxChannel > 0.6) return 1.0;

        double clampedHue = Math.max(0.0, Math.min(300.0, hueDegrees));
        return getComputedPosition(clampedHue);
    }

    private static double getComputedPosition(double clampedHue) {
        int i = 0;
        while (i < hueBreakpointsDeg.length - 2 && clampedHue > hueBreakpointsDeg[i + 1]) i++;

        double frac = (clampedHue - hueBreakpointsDeg[i]) /
                (hueBreakpointsDeg[i + 1] - hueBreakpointsDeg[i]);

        double pos = positionBreakpoints[i] +
                frac * (positionBreakpoints[i + 1] - positionBreakpoints[i]);

        if (pos < 0.0) pos = 0.0;
        if (pos > 1.0) pos = 1.0;
        return pos;
    }
}
