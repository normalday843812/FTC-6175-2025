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

import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp(name = "Color Sensor Tuning", group = "Tuning")
public class ColorSensorTuning extends LinearOpMode {

    private static final int[] CLASS_ORDER = new int[]{
            PURPLE_HOLE, PURPLE_SOLID, GREEN_HOLE, GREEN_SOLID, BLANK
    };
    private static final String[] CLASS_NAMES = new String[]{
            "Purple HOLE", "Purple SOLID", "Green HOLE", "Green SOLID", "Blank"
    };

    // Session buffers
    private final float[] sessionGain = new float[3];
    private final double[][][] sessionMu = new double[3][5][4];
    private final double[][][] sessionSigma = new double[3][5][4];



    @Override
    public void runOpMode() throws InterruptedException {
        NormalizedColorSensor[] sensors = new NormalizedColorSensor[]{
                hardwareMap.get(NormalizedColorSensor.class, "spindex_color_0"),
                hardwareMap.get(NormalizedColorSensor.class, "spindex_color_1"),
                hardwareMap.get(NormalizedColorSensor.class, "spindex_color_2")
        };

        System.arraycopy(SENSOR_GAIN, 0, sessionGain, 0, 3);

        waitForStart();
        TelemetryManager.TelemetryWrapper panels = PanelsTelemetry.INSTANCE.getFtcTelemetry();
        if (isStopRequested()) return;

        for (int sIdx = 0; sIdx < 3 && opModeIsActive(); sIdx++) {
            tuneSensor(sIdx, sensors[sIdx], panels);
        }

        telemetry.clearAll();
        panels.clearAll();
        telemetry.addData("Gain", Arrays.toString(sessionGain));
        panels.addData("Gain", Arrays.toString(sessionGain));
        telemetry.addData("Mu", Arrays.deepToString(sessionMu));
        panels.addData("Mu", Arrays.deepToString(sessionMu));
        telemetry.addData("Sigma", Arrays.deepToString(sessionSigma));
        panels.addData("Sigma", Arrays.deepToString(sessionSigma));
        telemetry.addLine("Apply calibration?");
        panels.addLine("Apply calibration?");
        telemetry.addLine("[Y] Apply  |  [B] Discard");
        panels.addLine("[Y] Apply  |  [B] Discard");
        telemetry.update();
        panels.update();


        while (opModeIsActive()) {
            if (gamepad1.y) {
                for (int s = 0; s < 3; s++) {
                    applyGain(s, sessionGain[s]);
                    for (int cls : CLASS_ORDER) {
                        applyClassStats(s, cls, sessionMu[s][cls], sessionSigma[s][cls]);
                    }
                }
                telemetry.clearAll();
                panels.clearAll();
                telemetry.addLine("Applied.");
                panels.addLine("Applied.");
                telemetry.update();
                panels.update();
                sleep(500);
                break;
            }
            if (gamepad1.b) {
                telemetry.clearAll();
                panels.clearAll();
                telemetry.addLine("Discarded.");
                panels.addLine("Discarded.");
                telemetry.update();
                panels.update();
                sleep(500);
                break;
            }
            idle();
        }
    }

    private void tuneSensor(int sensorIndex, NormalizedColorSensor sensor,
                            TelemetryManager.TelemetryWrapper panels) {
        float gain = sessionGain[sensorIndex];

        for (int p = 0; p < CLASS_ORDER.length && opModeIsActive(); p++) {
            int cls = CLASS_ORDER[p];
            String name = CLASS_NAMES[p];

            Stats h = new Stats();
            Stats s = new Stats();
            Stats v = new Stats();
            Stats a = new Stats();

            boolean sampling;
            boolean finishedPrompt = false;

            while (opModeIsActive() && !finishedPrompt) {
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

                sampling = gamepad1.a;

                if (sampling) {
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

                if (gamepad1.b) {
                    sessionGain[sensorIndex] = gain;
                    sessionMu[sensorIndex][cls][H] = h.mean();
                    sessionMu[sensorIndex][cls][S] = s.mean();
                    sessionMu[sensorIndex][cls][V] = v.mean();
                    sessionMu[sensorIndex][cls][A] = a.mean();

                    sessionSigma[sensorIndex][cls][H] = Math.max(h.std(), 1.0);
                    sessionSigma[sensorIndex][cls][S] = Math.max(s.std(), 0.05);
                    sessionSigma[sensorIndex][cls][V] = Math.max(v.std(), 0.05);
                    sessionSigma[sensorIndex][cls][A] = Math.max(a.std(), 0.05);

                    finishedPrompt = true;
                    sleep(200);
                }

                telemetry.clearAll();
                telemetry.addLine("Sensor " + sensorIndex + " — " + name);
                telemetry.addData("Gain", "%.1f", gain);
                telemetry.addData("Frames", "%d", h.n);
                telemetry.addData("H mean/std", "%.1f / %.1f", h.mean(), h.std());
                telemetry.addData("S mean/std", "%.3f / %.3f", s.mean(), s.std());
                telemetry.addData("V mean/std", "%.3f / %.3f", v.mean(), v.std());
                telemetry.addData("A mean/std", "%.3f / %.3f", a.mean(), a.std());
                telemetry.addLine("[A] Hold to sample  |  [B] Accept");
                telemetry.update();

                panels.clearAll();
                panels.addLine("Sensor " + sensorIndex + " — " + name);
                panels.addData("Gain", "%.1f", gain);
                panels.addData("Frames", "%d", h.n);
                panels.addData("H mean/std", "%.1f / %.1f", h.mean(), h.std());
                panels.addData("S mean/std", "%.3f / %.3f", s.mean(), s.std());
                panels.addData("V mean/std", "%.3f / %.3f", v.mean(), v.std());
                panels.addData("A mean/std", "%.3f / %.3f", a.mean(), a.std());
                panels.addLine("[A] Hold to sample  |  [B] Accept");
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