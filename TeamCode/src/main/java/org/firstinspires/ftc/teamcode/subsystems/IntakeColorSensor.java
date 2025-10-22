package org.firstinspires.ftc.teamcode.subsystems;

import static android.graphics.Color.RGBToHSV;
import static org.firstinspires.ftc.teamcode.config.IntakeColorSensorConfig.GAIN;
import static org.firstinspires.ftc.teamcode.config.IntakeColorSensorConfig.HUE_MAX_GREEN;
import static org.firstinspires.ftc.teamcode.config.IntakeColorSensorConfig.HUE_MAX_PURPLE;
import static org.firstinspires.ftc.teamcode.config.IntakeColorSensorConfig.HUE_MIN_GREEN;
import static org.firstinspires.ftc.teamcode.config.IntakeColorSensorConfig.HUE_MIN_PURPLE;
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
    private final NormalizedColorSensor intakeColorSensor;
    private final TelemetryHelper tele;
    private boolean isPurple;
    private boolean isGreen;
    private final float[] hsv = new float[3];
    int r, g, b;
    public IntakeColorSensor (NormalizedColorSensor intakeColorSensor, OpMode opmode) {
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
        isPurple =
                hsv[0] >= HUE_MIN_PURPLE && hsv[0] <= HUE_MAX_PURPLE &&
                hsv[1] >= SATURATION_MIN_PURPLE &&
                hsv[2] >= VALUE_MIN_PURPLE;
        isGreen =
                hsv[0] >= HUE_MIN_GREEN && hsv[0] <= HUE_MAX_GREEN &&
                hsv[1] >= SATURATION_MIN_GREEN &&
                hsv[2] >= VALUE_MIN_GREEN;

        addTelemetry();
    }

    public boolean isPurple() {
        return isPurple;
    }

    public boolean isGreen() {
        return isGreen;
    }

    private void addTelemetry() {
        tele.addLine("=== INTAKE COLOR SENSOR ===")
                .addData("Red", "%d", r)
                .addData("Green", "%d", g)
                .addData("Blue", "%d", b)
                .addData("HSV", Arrays.toString(hsv))
                .addData("Is Purple?", "%b", isPurple)
                .addData("Is Green?", "%b", isGreen);
    }
}
