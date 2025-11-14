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
    private boolean enabled = true;

    public SpindexSlotsColor(NormalizedColorSensor slot0,
                             NormalizedColorSensor slot1,
                             NormalizedColorSensor slot2,
                             OpMode opmode) {
        this.s0 = slot0;
        this.s1 = slot1;
        this.s2 = slot2;
        this.tele = new TelemetryHelper(opmode, TELEMETRY_ENABLED);
    }

    public void setEnabled(boolean enabled) {
        if (this.enabled != enabled) {
            this.enabled = enabled;
            if (!enabled) {
                c0.reset();
                c1.reset();
                c2.reset();
            }
        }
    }

    public void update() {
        if (!enabled) {
            disabledTelemetry(0);
            disabledTelemetry(1);
            disabledTelemetry(2);
            return;
        }

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
            cls.pushSample(r, g, b, x.alpha);
        } catch (Throwable t) {
            dead = true;
            cls.reset();
        }

        tele.addLine("--- SLOT " + idx + " ---")
                .addData("Color", getColor(idx)::name)
                .addData("deadFrame", "%b", dead);
    }

    private void disabledTelemetry(int idx) {
        tele.addLine("--- SLOT " + idx + " ---")
                .addData("Color", BallColor.NONE::name)
                .addData("deadFrame", "%b", true)
                .addData("enabled", "%b", false);
    }

    public BallColor getColor(int slot) {
        if (!enabled) return BallColor.NONE;
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
        if (!enabled) return false;
        BallColor v = getColor(slot);
        return v == BallColor.PURPLE || v == BallColor.GREEN;
    }

    /**
     * Find the slot index containing the specified color.
     * Returns -1 if not found.
     */
    public int findBallSlot(BallColor targetColor) {
        if (!enabled || targetColor == BallColor.NONE) return -1;
        for (int i = 0; i < 3; i++) {
            if (getColor(i) == targetColor) {
                return i;
            }
        }
        return -1;
    }
}
