package org.firstinspires.ftc.teamcode.subsystems;

import static android.graphics.Color.RGBToHSV;
import static org.firstinspires.ftc.teamcode.config.IntakeColorSensorConfig.GAIN;
import static org.firstinspires.ftc.teamcode.config.IntakeColorSensorConfig.HUE_MAX_GREEN;
import static org.firstinspires.ftc.teamcode.config.IntakeColorSensorConfig.HUE_MAX_PURPLE;
import static org.firstinspires.ftc.teamcode.config.IntakeColorSensorConfig.HUE_MIN_GREEN;
import static org.firstinspires.ftc.teamcode.config.IntakeColorSensorConfig.HUE_MIN_PURPLE;
import static org.firstinspires.ftc.teamcode.config.IntakeColorSensorConfig.WINDOW_SIZE;
import static org.firstinspires.ftc.teamcode.config.IntakeColorSensorConfig.N;
import static org.firstinspires.ftc.teamcode.config.IntakeColorSensorConfig.SATURATION_MIN_GREEN;
import static org.firstinspires.ftc.teamcode.config.IntakeColorSensorConfig.SATURATION_MIN_PURPLE;
import static org.firstinspires.ftc.teamcode.config.IntakeColorSensorConfig.TELEMETRY_ENABLED;
import static org.firstinspires.ftc.teamcode.config.IntakeColorSensorConfig.VALUE_MIN_GREEN;
import static org.firstinspires.ftc.teamcode.config.IntakeColorSensorConfig.VALUE_MIN_PURPLE;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.teamcode.util.TelemetryHelper;

import java.util.Arrays;

public class IntakeColorSensor {
    public enum BallColor { NONE, PURPLE, GREEN }

    private final NormalizedColorSensor intakeColorSensor;
    private final TelemetryHelper tele;
    private boolean isPurple;
    private boolean isGreen;
    private final float[] hsv = new float[3];
    private int r, g, b;

    private final boolean[] purpleBuf = new boolean[WINDOW_SIZE];
    private final boolean[] greenBuf  = new boolean[WINDOW_SIZE];
    private int idx = 0;

    public IntakeColorSensor(NormalizedColorSensor intakeColorSensor, OpMode opmode) {
        this.intakeColorSensor = intakeColorSensor;
        this.tele = new TelemetryHelper(opmode, TELEMETRY_ENABLED);
    }

    public void update() {
        intakeColorSensor.setGain(GAIN);
        NormalizedRGBA colors = intakeColorSensor.getNormalizedColors();
        r = Math.round(colors.red * 255f);
        g = Math.round(colors.green * 255f);
        b = Math.round(colors.blue * 255f);
        RGBToHSV(r, g, b, hsv);

        boolean purpleNow =
                hsv[0] >= HUE_MIN_PURPLE && hsv[0] <= HUE_MAX_PURPLE &&
                        hsv[1] >= SATURATION_MIN_PURPLE &&
                        hsv[2] >= VALUE_MIN_PURPLE;

        boolean greenNow =
                hsv[0] >= HUE_MIN_GREEN && hsv[0] <= HUE_MAX_GREEN &&
                        hsv[1] >= SATURATION_MIN_GREEN &&
                        hsv[2] >= VALUE_MIN_GREEN;

        purpleBuf[idx] = purpleNow && !greenNow;
        greenBuf[idx]  = greenNow && !purpleNow;
        idx = (idx + 1) % WINDOW_SIZE;

        isPurple = consistent(purpleBuf, N);
        isGreen  = consistent(greenBuf, N);

        addTelemetry();
    }

    public boolean isPurple() { return isPurple; }
    public boolean isGreen() { return isGreen; }

    public boolean isConsistentlyPurple() {
        return isPurple && !isGreen;
    }

    public boolean isConsistentlyGreen() {
        return isGreen && !isPurple;
    }

    public BallColor getBallColor() {
        if (isConsistentlyPurple()) return BallColor.PURPLE;
        if (isConsistentlyGreen()) return BallColor.GREEN;
        return BallColor.NONE;
    }

    /** @noinspection SameParameterValue*/
    private boolean consistent(boolean[] buf, int threshold) {
        int count = 0;
        for (boolean b : buf) if (b) count++;
        return count >= threshold;
    }

    private void addTelemetry() {
        tele.addLine("=== INTAKE COLOR SENSOR ===")
                .addData("Red", "%d", r)
                .addData("Green", "%d", g)
                .addData("Blue", "%d", b)
                .addData("HSV", Arrays.toString(hsv))
                .addData("Is Purple?", "%b", isPurple)
                .addData("Is Green?", "%b", isGreen)
                .addData("Stable", getBallColor()::name);
    }
}