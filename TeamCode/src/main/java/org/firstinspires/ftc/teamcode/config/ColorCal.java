package org.firstinspires.ftc.teamcode.config;

import com.bylazar.configurables.annotations.Configurable;

@Configurable
public class ColorCal {

    public static final int BLANK = 0;
    public static final int PURPLE_HOLE = 1;
    public static final int PURPLE_SOLID = 2;
    public static final int GREEN_HOLE = 3;
    public static final int GREEN_SOLID = 4;

    public static final int H = 0;
    public static final int S = 1;
    public static final int V = 2;
    public static final int A = 3;

    public static float[] SENSOR_GAIN = {45f, 45f, 45f};

    public static double[][][] MU = new double[3][5][4];
    public static double[][][] SIGMA = new double[3][5][4];

    public static double[] W = {1.0, 0.6, 0.25, 0.4};
    public static double SIGMA_MIN = 1e-3;
    public static double K_SCALE = 3.0;
    public static double HOLE_LEAK = 0.7;

    public static double MIN_BALL_CONF = 0.60;
    public static double COLOR_MARGIN = 0.15;
    public static double EWMA_ALPHA = 0.6;

    public static boolean TELEMETRY_ENABLED = true;

    static {
        for (int s = 0; s < 3; s++) {
            MU[s][BLANK][H] = 0;
            SIGMA[s][BLANK][H] = 180;
            MU[s][BLANK][S] = 0.1;
            SIGMA[s][BLANK][S] = 0.3;
            MU[s][BLANK][V] = 0.1;
            SIGMA[s][BLANK][V] = 0.3;
            MU[s][BLANK][A] = 0.05;
            SIGMA[s][BLANK][A] = 0.2;

            MU[s][PURPLE_HOLE][H] = 270;
            SIGMA[s][PURPLE_HOLE][H] = 25;
            MU[s][PURPLE_HOLE][S] = 0.4;
            SIGMA[s][PURPLE_HOLE][S] = 0.2;
            MU[s][PURPLE_HOLE][V] = 0.3;
            SIGMA[s][PURPLE_HOLE][V] = 0.2;
            MU[s][PURPLE_HOLE][A] = 0.25;
            SIGMA[s][PURPLE_HOLE][A] = 0.2;

            MU[s][PURPLE_SOLID][H] = 285;
            SIGMA[s][PURPLE_SOLID][H] = 20;
            MU[s][PURPLE_SOLID][S] = 0.6;
            SIGMA[s][PURPLE_SOLID][S] = 0.2;
            MU[s][PURPLE_SOLID][V] = 0.4;
            SIGMA[s][PURPLE_SOLID][V] = 0.2;
            MU[s][PURPLE_SOLID][A] = 0.35;
            SIGMA[s][PURPLE_SOLID][A] = 0.2;

            MU[s][GREEN_HOLE][H] = 165;
            SIGMA[s][GREEN_HOLE][H] = 25;
            MU[s][GREEN_HOLE][S] = 0.5;
            SIGMA[s][GREEN_HOLE][S] = 0.2;
            MU[s][GREEN_HOLE][V] = 0.3;
            SIGMA[s][GREEN_HOLE][V] = 0.2;
            MU[s][GREEN_HOLE][A] = 0.25;
            SIGMA[s][GREEN_HOLE][A] = 0.2;

            MU[s][GREEN_SOLID][H] = 170;
            SIGMA[s][GREEN_SOLID][H] = 20;
            MU[s][GREEN_SOLID][S] = 0.7;
            SIGMA[s][GREEN_SOLID][S] = 0.2;
            MU[s][GREEN_SOLID][V] = 0.4;
            SIGMA[s][GREEN_SOLID][V] = 0.2;
            MU[s][GREEN_SOLID][A] = 0.35;
            SIGMA[s][GREEN_SOLID][A] = 0.2;
        }
    }

    public static void applyGain(int sensorIndex, float gain) {
        SENSOR_GAIN[sensorIndex] = gain;
    }

    public static void applyClassStats(int sensorIndex, int classIndex, double[] muHSVA, double[] sigmaHSVA) {
        for (int c = 0; c < 4; c++) {
            MU[sensorIndex][classIndex][c] = muHSVA[c];
            SIGMA[sensorIndex][classIndex][c] = Math.max(SIGMA_MIN, sigmaHSVA[c]);
        }
    }
}