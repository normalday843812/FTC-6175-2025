package org.firstinspires.ftc.teamcode.tuning;

import static org.firstinspires.ftc.teamcode.config.ColorCal.A;
import static org.firstinspires.ftc.teamcode.config.ColorCal.BLANK;
import static org.firstinspires.ftc.teamcode.config.ColorCal.GREEN_HOLE;
import static org.firstinspires.ftc.teamcode.config.ColorCal.GREEN_SOLID;
import static org.firstinspires.ftc.teamcode.config.ColorCal.H;
import static org.firstinspires.ftc.teamcode.config.ColorCal.PURPLE_HOLE;
import static org.firstinspires.ftc.teamcode.config.ColorCal.PURPLE_SOLID;
import static org.firstinspires.ftc.teamcode.config.ColorCal.S;
import static org.firstinspires.ftc.teamcode.config.ColorCal.SENSOR_GAIN;
import static org.firstinspires.ftc.teamcode.config.ColorCal.V;
import static org.firstinspires.ftc.teamcode.config.ColorCal.applyClassStats;
import static org.firstinspires.ftc.teamcode.config.ColorCal.applyGain;

import java.util.Arrays;

import android.graphics.Color;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

@TeleOp(name = "Color Sensor Tuning (6x)", group = "Tuning")
public class ColorSensorTuning extends LinearOpMode {

    private static final int[] CLASS_ORDER = new int[]{
            PURPLE_HOLE, PURPLE_SOLID, GREEN_HOLE, GREEN_SOLID, BLANK
    };
    private static final String[] CLASS_NAMES = new String[]{
            "Purple HOLE", "Purple SOLID", "Green HOLE", "Green SOLID", "Blank"
    };

    // 6 Sensors total now
    private static final int SENSOR_COUNT = 6;

    // Session buffers (resized for 6)
    private final float[] sessionGain = new float[SENSOR_COUNT];
    private final double[][][] sessionMu = new double[SENSOR_COUNT][5][4];
    private final double[][][] sessionSigma = new double[SENSOR_COUNT][5][4];

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize 6 sensors
        // Mapping: 0=Slot0_1, 1=Slot0_2, 2=Slot1_1, 3=Slot1_2, 4=Slot2_1, 5=Slot2_2
        NormalizedColorSensor[] sensors = new NormalizedColorSensor[]{
                hardwareMap.get(NormalizedColorSensor.class, "spindex_color_0_1"),
                hardwareMap.get(NormalizedColorSensor.class, "spindex_color_0_2"),
                hardwareMap.get(NormalizedColorSensor.class, "spindex_color_1_1"),
                hardwareMap.get(NormalizedColorSensor.class, "spindex_color_1_2"),
                hardwareMap.get(NormalizedColorSensor.class, "spindex_color_2_1"),
                hardwareMap.get(NormalizedColorSensor.class, "spindex_color_2_2")
        };

        // Load existing config (make sure ColorCal is updated to size 6!)
        System.arraycopy(SENSOR_GAIN, 0, sessionGain, 0, SENSOR_COUNT);

        waitForStart();
        TelemetryManager.TelemetryWrapper panels = PanelsTelemetry.INSTANCE.getFtcTelemetry();
        if (isStopRequested()) return;

        // Tune each sensor sequentially
        for (int sIdx = 0; sIdx < SENSOR_COUNT && opModeIsActive(); sIdx++) {
            tuneSensor(sIdx, sensors[sIdx], panels);
        }

        // Summary Screen
        telemetry.clearAll();
        panels.clearAll();
        telemetry.addData("Gain", Arrays.toString(sessionGain));
        panels.addData("Gain", Arrays.toString(sessionGain));
        telemetry.addLine("Ready to save. Press Y to Apply, B to Discard.");
        panels.addLine("Ready to save. Press Y to Apply, B to Discard.");
        telemetry.update();
        panels.update();

        while (opModeIsActive()) {
            if (gamepad1.y) {
                // Apply to static config
                for (int s = 0; s < SENSOR_COUNT; s++) {
                    applyGain(s, sessionGain[s]);
                    for (int cls : CLASS_ORDER) {
                        applyClassStats(s, cls, sessionMu[s][cls], sessionSigma[s][cls]);
                    }
                }
                telemetry.clearAll();
                telemetry.addLine("Applied to Config.");
                telemetry.update();
                sleep(1000);
                break;
            }
            if (gamepad1.b) {
                telemetry.clearAll();
                telemetry.addLine("Discarded.");
                telemetry.update();
                sleep(1000);
                break;
            }
            idle();
        }
    }

    private void tuneSensor(int sensorIndex, NormalizedColorSensor sensor,
                            TelemetryManager.TelemetryWrapper panels) {
        float gain = sessionGain[sensorIndex];

        // Determine human-readable name for this sensor
        int slotNum = sensorIndex / 2;
        int sensorSubNum = (sensorIndex % 2) + 1;
        String sensorName = "Slot " + slotNum + " (Sensor " + sensorSubNum + ")";

        for (int p = 0; p < CLASS_ORDER.length && opModeIsActive(); p++) {
            int cls = CLASS_ORDER[p];
            String className = CLASS_NAMES[p];

            Stats h = new Stats();
            Stats s = new Stats();
            Stats v = new Stats();
            Stats a = new Stats();

            boolean finishedPrompt = false;

            while (opModeIsActive() && !finishedPrompt) {
                // Gain adjustment
                if (gamepad1.dpad_up) {
                    gain += 1f;
                    sleep(140);
                }
                if (gamepad1.dpad_down) {
                    gain -= 1f;
                    sleep(140);
                }
                if (gamepad1.dpad_right) {
                    gain += 5f;
                    sleep(140);
                }
                if (gamepad1.dpad_left) {
                    gain -= 5f;
                    sleep(140);
                }
                if (gain < 1f) gain = 1f;
                if (gain > 128f) gain = 128f;

                sensor.setGain(gain);

                // Sampling
                if (gamepad1.a) {
                    NormalizedRGBA rgba = sensor.getNormalizedColors();
                    int ri = clamp255(Math.round(rgba.red * 255f));
                    int gi = clamp255(Math.round(rgba.green * 255f));
                    int bi = clamp255(Math.round(rgba.blue * 255f));
                    float[] hsv = new float[3];
                    Color.RGBToHSV(ri, gi, bi, hsv);

                    h.add(hsv[0]);
                    s.add(hsv[1]);
                    v.add(hsv[2]);
                    a.add(clamp01(rgba.alpha));
                }

                // Finish this class
                if (gamepad1.b) {
                    sessionGain[sensorIndex] = gain;
                    sessionMu[sensorIndex][cls][H] = h.mean();
                    sessionMu[sensorIndex][cls][S] = s.mean();
                    sessionMu[sensorIndex][cls][V] = v.mean();
                    sessionMu[sensorIndex][cls][A] = a.mean();

                    // Minimum sigmas to prevent overfitting
                    sessionSigma[sensorIndex][cls][H] = Math.max(h.std(), 1.0);
                    sessionSigma[sensorIndex][cls][S] = Math.max(s.std(), 0.05);
                    sessionSigma[sensorIndex][cls][V] = Math.max(v.std(), 0.05);
                    sessionSigma[sensorIndex][cls][A] = Math.max(a.std(), 0.05);

                    finishedPrompt = true;
                    sleep(250);
                }

                // Telemetry
                String status = "TUNING: " + sensorName + " -> " + className;

                telemetry.clearAll();
                telemetry.addLine(status);
                telemetry.addData("Gain", "%.1f", gain);
                telemetry.addData("Samples", "%d", h.n);
                telemetry.addData("H", "%.1f (%.1f)", h.mean(), h.std());
                telemetry.addData("S", "%.3f (%.3f)", s.mean(), s.std());
                telemetry.addData("V", "%.3f (%.3f)", v.mean(), v.std());
                telemetry.addLine("[A] Hold to Sample | [B] Save & Next");
                telemetry.update();

                panels.clearAll();
                panels.addLine(status);
                panels.addData("Gain", "%.1f", gain);
                panels.addData("Samples", "%d", h.n);
                panels.update();

                idle();
            }
        }
    }

    private static final class Stats {
        int n = 0;
        double sum = 0;
        double sumSq = 0;

        void add(double v) {
            n++;
            sum += v;
            sumSq += v * v;
        }

        double mean() {
            return n > 0 ? sum / n : 0.0;
        }

        double std() {
            if (n <= 1) return 0.0;
            double m = mean();
            double var = (sumSq - n * m * m) / (n - 1);
            return var > 0 ? Math.sqrt(var) : 0.0;
        }
    }

    private static int clamp255(int v) {
        return Math.max(0, Math.min(255, v));
    }

    private static float clamp01(float v) {
        return Math.max(0f, Math.min(1f, v));
    }
}