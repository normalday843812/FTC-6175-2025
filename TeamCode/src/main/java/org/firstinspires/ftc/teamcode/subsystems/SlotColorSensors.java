package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.config.ColorCal.A;
import static org.firstinspires.ftc.teamcode.config.ColorCal.EWMA_ALPHA;
import static org.firstinspires.ftc.teamcode.config.ColorCal.GREEN_HOLE;
import static org.firstinspires.ftc.teamcode.config.ColorCal.GREEN_SOLID;
import static org.firstinspires.ftc.teamcode.config.ColorCal.H;
import static org.firstinspires.ftc.teamcode.config.ColorCal.HOLE_LEAK;
import static org.firstinspires.ftc.teamcode.config.ColorCal.K_SCALE;
import static org.firstinspires.ftc.teamcode.config.ColorCal.MU;
import static org.firstinspires.ftc.teamcode.config.ColorCal.MIN_BALL_CONF;
import static org.firstinspires.ftc.teamcode.config.ColorCal.S;
import static org.firstinspires.ftc.teamcode.config.ColorCal.SENSOR_GAIN;
import static org.firstinspires.ftc.teamcode.config.ColorCal.SIGMA;
import static org.firstinspires.ftc.teamcode.config.ColorCal.SIGMA_MIN;
import static org.firstinspires.ftc.teamcode.config.ColorCal.PURPLE_HOLE;
import static org.firstinspires.ftc.teamcode.config.ColorCal.PURPLE_SOLID;
import static org.firstinspires.ftc.teamcode.config.ColorCal.TELEMETRY_ENABLED;
import static org.firstinspires.ftc.teamcode.config.ColorCal.V;
import static org.firstinspires.ftc.teamcode.config.ColorCal.W;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.teamcode.util.TelemetryHelper;

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

    private final double[] ewmaBall = new double[3];
    private final boolean[] ewmaInit = new boolean[3];

    public SlotColorSensors(NormalizedColorSensor[] sensors, OpMode opmode) {
        this.devices = sensors;
        this.tele = new TelemetryHelper(opmode, TELEMETRY_ENABLED);
    }

    public void setEnabled(boolean on) {
        enabled = on;
    }

    public void update() {
        if (!enabled) return;

        // Show telemetry for each SLOT with both sensors' data
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
     * Detects ball presence using all 3 sensors with fusion.
     * Ball = max(purple, green) confidence from any sensor.
     */
    private Observation hasBallObservation() {
        double maxConf = 0.0;
        boolean anyValid = false;

        // Check all 3 sensors and take max confidence
        for (int idx = 0; idx < devices.length && idx < 3; idx++) {
            double conf = classifySensor(idx);
            if (conf >= 0) {
                anyValid = true;
                maxConf = Math.max(maxConf, conf);
            }
        }

        BallColor color = (maxConf >= MIN_BALL_CONF) ? BallColor.BALL : BallColor.NONE;
        return new Observation(color, maxConf, anyValid);
    }

    /**
     * Classifies a single sensor and returns ball confidence.
     * Ball confidence = max(purple, green) since we don't care about color.
     * @return Ball confidence (0.0-1.0), or -1 if invalid
     */
    private double classifySensor(int idx) {
        if (idx < 0 || idx >= devices.length) {
            return -1;
        }

        NormalizedColorSensor sensor = devices[idx];
        float gain = SENSOR_GAIN[idx];

        double[] sample = new double[4]; // H,S,V,A

        try {
            sensor.setGain(gain);
            NormalizedRGBA rgba = sensor.getNormalizedColors();
            int r = clamp255(Math.round(rgba.red * 255f));
            int g = clamp255(Math.round(rgba.green * 255f));
            int b = clamp255(Math.round(rgba.blue * 255f));
            float[] hsv = new float[3];
            Color.RGBToHSV(r, g, b, hsv);

            sample[H] = hsv[0];
            sample[S] = hsv[1];
            sample[V] = hsv[2];
            sample[A] = clamp01(rgba.alpha);
        } catch (Throwable t) {
            if (!ewmaInit[idx]) {
                ewmaBall[idx] = 0.0;
                ewmaInit[idx] = true;
            }
            return ewmaBall[idx];
        }

        double purpleSolid = classConfidence(idx, PURPLE_SOLID, sample);
        double purpleHole = classConfidence(idx, PURPLE_HOLE, sample);
        double greenSolid = classConfidence(idx, GREEN_SOLID, sample);
        double greenHole = classConfidence(idx, GREEN_HOLE, sample);

        double purple = Math.max(purpleSolid, purpleHole * HOLE_LEAK);
        double green = Math.max(greenSolid, greenHole * HOLE_LEAK);
        double ball = Math.max(purple, green);

        if (!ewmaInit[idx]) {
            ewmaBall[idx] = ball;
            ewmaInit[idx] = true;
        } else {
            ewmaBall[idx] = EWMA_ALPHA * ball + (1.0 - EWMA_ALPHA) * ewmaBall[idx];
        }

        return ewmaBall[idx];
    }

    private static double classConfidence(int sensorIndex, int cls, double[] x) {
        double dH = hueDiff(x[H], MU[sensorIndex][cls][H]) / denom(sensorIndex, cls, H);
        double dS = (x[S] - MU[sensorIndex][cls][S]) / denom(sensorIndex, cls, S);
        double dV = (x[V] - MU[sensorIndex][cls][V]) / denom(sensorIndex, cls, V);
        double dA = (x[A] - MU[sensorIndex][cls][A]) / denom(sensorIndex, cls, A);

        double D = W[H] * Math.abs(dH)
                + W[S] * Math.abs(dS)
                + W[V] * Math.abs(dV)
                + W[A] * Math.abs(dA);

        return Math.exp(-D / K_SCALE);
    }

    private static double denom(int s, int cls, int ch) {
        return Math.max(SIGMA_MIN, SIGMA[s][cls][ch]);
    }

    private static double hueDiff(double aDeg, double bDeg) {
        double d = Math.abs(aDeg - bDeg) % 360.0;
        return (d > 180.0) ? 360.0 - d : d;
    }

    private void addBallTelemetry(Observation obs) {
        tele.addLine("=== BALL SENSOR ===")
                .addData("Ball", "%s", obs.color == BallColor.BALL ? "YES" : "no")
                .addData("Confidence", "%.2f", obs.ballConfidence)
                .addData("Valid", "%b", obs.valid);

        // Show individual sensor confidences for debugging
        if (TELEMETRY_ENABLED) {
            for (int i = 0; i < Math.min(devices.length, 3); i++) {
                tele.addData("Sensor" + i, "%.2f", ewmaBall[i]);
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