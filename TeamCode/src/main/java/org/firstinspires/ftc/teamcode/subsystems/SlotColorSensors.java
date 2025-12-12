package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.config.ColorCal.A;
import static org.firstinspires.ftc.teamcode.config.ColorCal.A_THRESHOLD;
import static org.firstinspires.ftc.teamcode.config.ColorCal.H;
import static org.firstinspires.ftc.teamcode.config.ColorCal.REQUIRE_BOTH_VA;
import static org.firstinspires.ftc.teamcode.config.ColorCal.S;
import static org.firstinspires.ftc.teamcode.config.ColorCal.SENSOR_GAIN;
import static org.firstinspires.ftc.teamcode.config.ColorCal.TELEMETRY_ENABLED;
import static org.firstinspires.ftc.teamcode.config.ColorCal.V;
import static org.firstinspires.ftc.teamcode.config.ColorCal.V_THRESHOLD;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.teamcode.util.TelemetryHelper;

/**
 * Ball detection using simple V/A thresholds.
 */
public class SlotColorSensors {

    public enum BallColor {NONE, BALL}

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
        for (int idx = 0; idx < devices.length && idx < 3; idx++) {
            double[] sample = readSensor(idx);
            if (sample != null) {
                lastHSVA[idx] = sample;
                if (hasBallAt(idx, sample)) {
                    anyBall = true;
                }
            }
        }

        addBallTelemetry(anyBall);
    }

    /**
     * Returns true if ANY sensor detects a ball.
     */
    public boolean hasBall() {
        for (int idx = 0; idx < devices.length && idx < 3; idx++) {
            double[] sample = readSensor(idx);
            if (sample != null && hasBallAt(idx, sample)) {
                return true;
            }
        }
        return false;
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
        if (sample != null && hasBallAt(slot, sample)) {
            return BallColor.BALL;
        }
        return BallColor.NONE;
    }

    /**
     * Checks if a specific sensor has a ball based on V/A thresholds.
     */
    private boolean hasBallAt(int idx, double[] sample) {
        if (idx < 0 || idx >= V_THRESHOLD.length || sample == null) {
            return false;
        }
        boolean vPass = sample[V] > V_THRESHOLD[idx];
        boolean aPass = sample[A] > A_THRESHOLD[idx];
        return REQUIRE_BOTH_VA ? (vPass && aPass) : (vPass || aPass);
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
                boolean vPass = hsva[V] > V_THRESHOLD[i];
                boolean aPass = hsva[A] > A_THRESHOLD[i];
                tele.addData("S" + i, "H=%.0f S=%.2f V=%.3f A=%.3f [V:%b A:%b]",
                        hsva[H], hsva[S], hsva[V], hsva[A], vPass, aPass);
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
