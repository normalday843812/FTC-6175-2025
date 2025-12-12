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

    public static final class Observation {
        public final BallColor color;
        public final double ballConfidence;
        public final boolean valid;

        public Observation(BallColor color, double ballConfidence, boolean valid) {
            this.color = color;
            this.ballConfidence = ballConfidence;
            this.valid = valid;
        }
    }

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

        Observation obs = hasBallObservation();
        addBallTelemetry(obs);
    }

    public boolean hasBall() {
        return hasBallObservation().color == BallColor.BALL;
    }

    @Deprecated
    public Observation getObservation(int slot) {
        return hasBallObservation();
    }

    @Deprecated
    public BallColor getColor(int slot) {
        return hasBallObservation().color;
    }

    @Deprecated
    public boolean hasAnyBall(int slot) {
        return hasBall();
    }

    @Deprecated
    public boolean isFull() {
        return false;
    }

    /**
     * Detects ball using V and A thresholds.
     */
    private Observation hasBallObservation() {
        double maxConf = 0.0;
        boolean anyValid = false;
        boolean ballDetected = false;

        for (int idx = 0; idx < devices.length && idx < 3; idx++) {
            double[] sample = readSensor(idx);
            if (sample != null) {
                anyValid = true;
                lastHSVA[idx] = sample;

                boolean vPass = sample[V] > V_THRESHOLD[idx];
                boolean aPass = sample[A] > A_THRESHOLD[idx];

                boolean sensorDetected = REQUIRE_BOTH_VA ? (vPass && aPass) : (vPass || aPass);

                if (sensorDetected) {
                    ballDetected = true;
                    maxConf = Math.max(maxConf, Math.max(sample[V], sample[A]));
                }
            }
        }

        BallColor color = ballDetected ? BallColor.BALL : BallColor.NONE;
        return new Observation(color, maxConf, anyValid);
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

    private void addBallTelemetry(Observation obs) {
        tele.addLine("=== BALL SENSOR ===")
                .addData("Ball", "%s", obs.color == BallColor.BALL ? "YES" : "no");

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
