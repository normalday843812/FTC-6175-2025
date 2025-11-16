package org.firstinspires.ftc.teamcode.sensors;

import static org.firstinspires.ftc.teamcode.config.IntakeColorSensorConfig.*;

import java.util.Arrays;

public class ColorClassifier {
    private final float[] hsv = new float[3];
    private final boolean[] purpleBuf = new boolean[WINDOW_SIZE];
    private final boolean[] greenBuf = new boolean[WINDOW_SIZE];
    private int idx = 0;
    private boolean purple, green;

    public void pushSample(int r, int g, int b, float alpha) {
        android.graphics.Color.RGBToHSV(r, g, b, hsv);
        float alpha255 = alpha * 255f;
        boolean present = alpha255 >= ALPHA_MIN_BALL && alpha255 <= ALPHA_MAX_BALL;

        boolean pNow = present && alpha255 >= ALPHA_MIN_PURPLE &&
                alpha255 <= ALPHA_MAX_PURPLE &&
                hsv[0] >= HUE_MIN_PURPLE && hsv[0] <= HUE_MAX_PURPLE &&
                hsv[1] >= SATURATION_MIN_PURPLE && hsv[2] >= VALUE_MIN_PURPLE;
        boolean gNow = present && alpha255 >= ALPHA_MIN_GREEN &&
                alpha255 <= ALPHA_MAX_GREEN &&
                hsv[0] >= HUE_MIN_GREEN && hsv[0] <= HUE_MAX_GREEN &&
                hsv[1] >= SATURATION_MIN_GREEN && hsv[2] >= VALUE_MIN_GREEN;

        if (pNow && gNow) {
            // Ambiguous frame, drop to avoid false positives.
            pNow = false;
            gNow = false;
        }

        purpleBuf[idx] = pNow;
        greenBuf[idx] = gNow;
        idx = (idx + 1) % WINDOW_SIZE;

        purple = consistent(purpleBuf, N);
        green = consistent(greenBuf, N);
    }

    public void reset() {
        Arrays.fill(purpleBuf, false);
        Arrays.fill(greenBuf, false);
        purple = false;
        green = false;
        idx = 0;
    }

    public boolean isPurple() {
        return purple && !green;
    }

    public boolean isGreen() {
        return green && !purple;
    }

    private static boolean consistent(boolean[] buf, int threshold) {
        int c = 0;
        for (boolean b : buf) if (b) c++;
        return c >= threshold;
    }
}
