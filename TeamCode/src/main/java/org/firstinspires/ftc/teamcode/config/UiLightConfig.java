package org.firstinspires.ftc.teamcode.config;

import static java.util.Map.entry;

import org.firstinspires.ftc.teamcode.subsystems.RgbIndicator;
import org.firstinspires.ftc.teamcode.subsystems.RgbIndicator.Fx;
import org.firstinspires.ftc.teamcode.subsystems.RgbIndicator.OffBehavior;

import java.util.Map;

public final class UiLightConfig {
    private UiLightConfig() {
    }

    public enum UiState {
        STARTUP, IDLE, SEEKING, DECIDING, NAVIGATING, SPINUP, READY,
        INTAKE, JAM, PARK, DONE, ERROR
    }

    public static final class Timing {
        public static final long SPEED_ON_MS = 200;
        public static final long SPEED_OFF_MS = 350;
        public static final long LOCK_ON_MS  = 150;
        public static final long LOCK_OFF_MS = 300;
    }

    public enum UiEvent {SHOT, PICKUP, FAIL}

    public static final Map<UiState, RgbIndicator.Pattern> BASE = Map.ofEntries(
            entry(UiState.STARTUP, new RgbIndicator.Pattern(180, Fx.BLINK, 600, 0.4, 0, OffBehavior.OFF)), // teal blink
            entry(UiState.SEEKING, new RgbIndicator.Pattern(50, Fx.SWEEP, 900, 1.0, 20, OffBehavior.OFF)), // yellow sweep
            entry(UiState.DECIDING, new RgbIndicator.Pattern(290, Fx.BLINK, 700, 0.5, 0, OffBehavior.OFF)), // violet blink
            entry(UiState.NAVIGATING, new RgbIndicator.Pattern(210, Fx.SOLID, 0, 1.0, 0, OffBehavior.OFF)), // blue solid
            entry(UiState.SPINUP, new RgbIndicator.Pattern(40, Fx.BLINK, 250, 0.5, 0, OffBehavior.OFF)), // amber fast blink
            entry(UiState.READY, new RgbIndicator.Pattern(120, Fx.SOLID, 0, 1.0, 0, OffBehavior.OFF)), // green solid
            entry(UiState.INTAKE, new RgbIndicator.Pattern(170, Fx.SWEEP, 800, 1.0, 15, OffBehavior.OFF)), // cyan sweep
            entry(UiState.JAM, new RgbIndicator.Pattern(25, Fx.STROBE, 120, 0.5, 0, OffBehavior.OFF)), // orange strobe
            entry(UiState.PARK, new RgbIndicator.Pattern(0, Fx.BLINK, 800, 0.3, 0, OffBehavior.WHITE)), // white blink
            entry(UiState.IDLE, new RgbIndicator.Pattern(180, Fx.SOLID, 0, 1.0, 0, OffBehavior.OFF)), // teal solid
            entry(UiState.DONE, new RgbIndicator.Pattern(120, Fx.BLINK, 1200, 0.3, 0, OffBehavior.OFF)), // green slow blink
            entry(UiState.ERROR, new RgbIndicator.Pattern(0, Fx.STROBE, 90, 0.5, 0, OffBehavior.OFF)) // red strobe
    );

    // Event overlays
    public static final Map<UiEvent, RgbIndicator.Pattern> EVENTS = Map.ofEntries(
            entry(UiEvent.SHOT, new RgbIndicator.Pattern(120, Fx.STROBE, 70, 1.0, 0, OffBehavior.OFF)),
            entry(UiEvent.PICKUP, new RgbIndicator.Pattern(160, Fx.STROBE, 80, 1.0, 0, OffBehavior.OFF)),
            entry(UiEvent.FAIL, new RgbIndicator.Pattern(0, Fx.STROBE, 100, 1.0, 0, OffBehavior.OFF))
    );
}
