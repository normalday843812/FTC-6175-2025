package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.config.AutoUnifiedConfig.TELEMETRY_ENABLED;
import static org.firstinspires.ftc.teamcode.config.IntakeColorSensorConfig.GAIN;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.teamcode.sensors.ColorClassifier;
import org.firstinspires.ftc.teamcode.util.TelemetryHelper;

public class SpindexSlotsColor {
    public enum BallColor {NONE, PURPLE, GREEN}

    private final NormalizedColorSensor s0, s1, s2;
    private final ColorClassifier c0 = new ColorClassifier();
    private final ColorClassifier c1 = new ColorClassifier();
    private final ColorClassifier c2 = new ColorClassifier();
    private final TelemetryHelper tele;

    public SpindexSlotsColor(NormalizedColorSensor slot0,
                             NormalizedColorSensor slot1,
                             NormalizedColorSensor slot2,
                             OpMode opmode) {
        this.s0 = slot0;
        this.s1 = slot1;
        this.s2 = slot2;
        this.tele = new TelemetryHelper(opmode, TELEMETRY_ENABLED);
    }

    public void update() {
        readOne(s0, c0, 0);
        readOne(s1, c1, 1);
        readOne(s2, c2, 2);
    }

    private void readOne(NormalizedColorSensor dev, ColorClassifier cls, int idx) {
        boolean dead;
        try {
            dev.setGain(GAIN);
            NormalizedRGBA x = dev.getNormalizedColors();
            int r = Math.round(x.red * 255f);
            int g = Math.round(x.green * 255f);
            int b = Math.round(x.blue * 255f);
            dead = (r == 0 && g == 0 && b == 0);
            cls.pushRgb(r, g, b);
        } catch (Throwable t) {
            dead = true;
        }

        tele.addLine("--- SLOT " + idx + " ---")
                .addData("Color", getColor(idx)::name)
                .addData("deadFrame", "%b", dead);
    }

    public BallColor getColor(int slot) {
        switch (slot) {
            case 0:
                return c0.isPurple() ? BallColor.PURPLE : c0.isGreen() ? BallColor.GREEN : BallColor.NONE;
            case 1:
                return c1.isPurple() ? BallColor.PURPLE : c1.isGreen() ? BallColor.GREEN : BallColor.NONE;
            case 2:
                return c2.isPurple() ? BallColor.PURPLE : c2.isGreen() ? BallColor.GREEN : BallColor.NONE;
            default:
                return BallColor.NONE;
        }
    }

    public boolean hasAnyBall(int slot) {
        BallColor v = getColor(slot);
        return v == BallColor.PURPLE || v == BallColor.GREEN;
    }
}