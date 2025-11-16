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
import static org.firstinspires.ftc.teamcode.config.ColorCal.COLOR_MARGIN;
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

    public enum BallColor {NONE, PURPLE, GREEN}

    public static final class Observation {
        public final BallColor color;
        public final double ballConfidence;
        public final double purpleConfidence;
        public final double greenConfidence;
        public final boolean valid;

        public Observation(BallColor color,
                           double ballConfidence,
                           double purpleConfidence,
                           double greenConfidence,
                           boolean valid) {
            this.color = color;
            this.ballConfidence = ballConfidence;
            this.purpleConfidence = purpleConfidence;
            this.greenConfidence = greenConfidence;
            this.valid = valid;
        }
    }

    private final NormalizedColorSensor[] devices = new NormalizedColorSensor[3];
    private final TelemetryHelper tele;

    private boolean enabled = true;

    private final double[] ewmaPurple = new double[3];
    private final double[] ewmaGreen = new double[3];
    private final double[] ewmaBall = new double[3];
    private final boolean[] ewmaInit = new boolean[3];

    public SlotColorSensors(NormalizedColorSensor s0,
                            NormalizedColorSensor s1,
                            NormalizedColorSensor s2,
                            OpMode opmode) {
        devices[0] = s0;
        devices[1] = s1;
        devices[2] = s2;
        tele = new TelemetryHelper(opmode, TELEMETRY_ENABLED);
    }

    public void setEnabled(boolean on) {
        enabled = on;
    }

    public void update() {
        if (!enabled) {
            for (int i = 0; i < 3; i++) {
                addTelemetry(i, new Observation(BallColor.NONE, 0, 0, 0, false));
            }
            return;
        }
        for (int i = 0; i < 3; i++) {
            Observation observation = classify(i);
            addTelemetry(i, observation);
        }
    }

    public Observation getObservation(int slot) {
        return classify(slot);
    }

    public BallColor getColor(int slot) {
        return classify(slot).color;
    }

    public boolean hasAnyBall(int slot) {
        Observation o = classify(slot);
        return o.valid && o.ballConfidence >= MIN_BALL_CONF && o.color != BallColor.NONE;
    }

    public boolean isFull() {
        return hasAnyBall(0) && hasAnyBall(1) && hasAnyBall(2);
    }

    public int findBallSlot(BallColor target) {
        if (target == BallColor.NONE) return -1;
        for (int i = 0; i < 3; i++) {
            if (getColor(i) == target) return i;
        }
        return -1;
    }

    private Observation classify(int idx) {
        NormalizedColorSensor sensor = devices[idx];
        float gain = SENSOR_GAIN[idx];
        boolean valid = true;

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
            valid = false;
        }

        if (!valid) {
            if (!ewmaInit[idx]) {
                ewmaPurple[idx] = 0.0;
                ewmaGreen[idx] = 0.0;
                ewmaBall[idx] = 0.0;
                ewmaInit[idx] = true;
            }
            return new Observation(BallColor.NONE, ewmaBall[idx], ewmaPurple[idx], ewmaGreen[idx], false);
        }

        double purpleSolid = classConfidence(idx, PURPLE_SOLID, sample);
        double purpleHole = classConfidence(idx, PURPLE_HOLE, sample);
        double greenSolid = classConfidence(idx, GREEN_SOLID, sample);
        double greenHole = classConfidence(idx, GREEN_HOLE, sample);

        double purple = Math.max(purpleSolid, purpleHole * HOLE_LEAK);
        double green = Math.max(greenSolid, greenHole * HOLE_LEAK);
        double ball = Math.max(purple, green);

        if (!ewmaInit[idx]) {
            ewmaPurple[idx] = purple;
            ewmaGreen[idx] = green;
            ewmaBall[idx] = ball;
            ewmaInit[idx] = true;
        } else {
            ewmaPurple[idx] = EWMA_ALPHA * purple + (1.0 - EWMA_ALPHA) * ewmaPurple[idx];
            ewmaGreen[idx] = EWMA_ALPHA * green + (1.0 - EWMA_ALPHA) * ewmaGreen[idx];
            ewmaBall[idx] = EWMA_ALPHA * ball + (1.0 - EWMA_ALPHA) * ewmaBall[idx];
        }

        BallColor color = BallColor.NONE;
        if (ewmaBall[idx] >= MIN_BALL_CONF) {
            double diff = Math.abs(ewmaPurple[idx] - ewmaGreen[idx]);
            if (diff >= COLOR_MARGIN) {
                color = ewmaPurple[idx] > ewmaGreen[idx] ? BallColor.PURPLE : BallColor.GREEN;
            }
        }

        return new Observation(color, ewmaBall[idx], ewmaPurple[idx], ewmaGreen[idx], true);
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

    private void addTelemetry(int idx, Observation observation) {
        tele.addLine("--- SLOT " + idx + " ---")
                .addData("Color", "%s", observation.color.name())
                .addData("BallConf", "%.2f", observation.ballConfidence)
                .addData("PurpleConf", "%.2f", observation.purpleConfidence)
                .addData("GreenConf", "%.2f", observation.greenConfidence)
                .addData("Valid", "%b", observation.valid);
    }

    private static int clamp255(int v) {
        return Math.max(0, Math.min(255, v));
    }

    private static float clamp01(float v) {
        return Math.max(0f, Math.min(1f, v));
    }
}