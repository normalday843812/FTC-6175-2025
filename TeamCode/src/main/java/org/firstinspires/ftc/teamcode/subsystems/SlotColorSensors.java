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

    private final NormalizedColorSensor[] devices;
    private final TelemetryHelper tele;

    private boolean enabled = true;

    private final double[] ewmaPurple = new double[6];
    private final double[] ewmaGreen = new double[6];
    private final double[] ewmaBall = new double[6];
    private final boolean[] ewmaInit = new boolean[6];

    public SlotColorSensors(NormalizedColorSensor[] sensors, OpMode opmode) {
        this.devices = sensors;
        this.tele = new TelemetryHelper(opmode, TELEMETRY_ENABLED);
    }

    public void setEnabled(boolean on) {
        enabled = on;
    }

    public void update() {
        if (!enabled) return;

        // FIX #3: Show telemetry for each SLOT with both sensors' data
        for (int slot = 0; slot < 3; slot++) {
            Observation obs = classifySlot(slot);
            addSlotTelemetry(slot, obs);
        }
    }

    // FIX #2: Public API now calls classifySlot() instead of classifySensor()
    public Observation getObservation(int slot) {
        return classifySlot(slot);
    }

    // FIX #2: Public API now calls classifySlot() instead of classifySensor()
    public BallColor getColor(int slot) {
        return classifySlot(slot).color;
    }

    // FIX #2: Public API now calls classifySlot() instead of classifySensor()
    public boolean hasAnyBall(int slot) {
        Observation o = classifySlot(slot);
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

    /**
     * Classifies a SLOT (0-2) by merging data from both sensors in that slot.
     * Slot 0 = sensors 0 & 1
     * Slot 1 = sensors 2 & 3
     * Slot 2 = sensors 4 & 5
     */
    private Observation classifySlot(int slotIdx) {
        int sensorA = slotIdx * 2;
        int sensorB = slotIdx * 2 + 1;

        Observation obsA = classifySensor(sensorA);
        Observation obsB = classifySensor(sensorB);

        // Merge logic: pick the sensor with stronger ball detection
        boolean aHasBall = obsA.valid && obsA.color != BallColor.NONE;
        boolean bHasBall = obsB.valid && obsB.color != BallColor.NONE;

        if (aHasBall && !bHasBall) return obsA;
        if (!aHasBall && bHasBall) return obsB;
        if (aHasBall && bHasBall) {
            return (obsA.ballConfidence > obsB.ballConfidence) ? obsA : obsB;
        }

        // Neither has ball - return higher confidence (noise floor)
        return (obsA.ballConfidence > obsB.ballConfidence) ? obsA : obsB;
    }

    /**
     * Classifies a single SENSOR (0-5) - internal use only.
     */
    private Observation classifySensor(int idx) {
        if (idx < 0 || idx >= devices.length) {
            return new Observation(BallColor.NONE, 0, 0, 0, false);
        }

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

    // FIX #3: New telemetry method that shows slot-level data with both sensors
    private void addSlotTelemetry(int slot, Observation obs) {
        int sensorA = slot * 2;
        int sensorB = slot * 2 + 1;

        tele.addLine("--- SLOT " + slot + " (sensors " + sensorA + "," + sensorB + ") ---")
                .addData("Color", "%s", obs.color.name())
                .addData("BallConf", "%.2f", obs.ballConfidence)
                .addData("PurpleConf", "%.2f", obs.purpleConfidence)
                .addData("GreenConf", "%.2f", obs.greenConfidence)
                .addData("Valid", "%b", obs.valid);

        // Show raw HSV from both sensors for debugging
        if (TELEMETRY_ENABLED && devices[sensorA] != null && devices[sensorB] != null) {
            try {
                NormalizedRGBA rgbaA = devices[sensorA].getNormalizedColors();
                NormalizedRGBA rgbaB = devices[sensorB].getNormalizedColors();

                int rA = clamp255(Math.round(rgbaA.red * 255f));
                int gA = clamp255(Math.round(rgbaA.green * 255f));
                int bA = clamp255(Math.round(rgbaA.blue * 255f));
                float[] hsvA = new float[3];
                Color.RGBToHSV(rA, gA, bA, hsvA);

                int rB = clamp255(Math.round(rgbaB.red * 255f));
                int gB = clamp255(Math.round(rgbaB.green * 255f));
                int bB = clamp255(Math.round(rgbaB.blue * 255f));
                float[] hsvB = new float[3];
                Color.RGBToHSV(rB, gB, bB, hsvB);

                tele.addData("Sensor" + sensorA + " H/S/V", "%.0f/%.2f/%.2f", hsvA[0], hsvA[1], hsvA[2])
                        .addData("Sensor" + sensorB + " H/S/V", "%.0f/%.2f/%.2f", hsvB[0], hsvB[1], hsvB[2]);
            } catch (Throwable t) {
                // Ignore telemetry errors
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