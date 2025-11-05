package org.firstinspires.ftc.teamcode.managers;

import org.firstinspires.ftc.teamcode.config.UiLightConfig;
import org.firstinspires.ftc.teamcode.subsystems.RgbIndicator;

public final class UiLight {
    private final RgbIndicator led;

    public UiLight(RgbIndicator led) {
        this.led = led;
    }

    public void setBase(UiLightConfig.UiState s) {
        RgbIndicator.Pattern p = UiLightConfig.BASE.get(s);
        if (p != null) led.setBasePattern(p);
    }

    public void notify(UiLightConfig.UiEvent e, long durationMs) {
        RgbIndicator.Pattern p = UiLightConfig.EVENTS.get(e);
        if (p != null) led.notifyPattern(p, durationMs);
    }

    public void update() {
        led.update();
    }
}
