package org.firstinspires.ftc.teamcode.managers;

import static org.firstinspires.ftc.teamcode.config.AutoUnifiedConfig.TARGET_RPM_BAND;

import org.firstinspires.ftc.teamcode.config.UiLightConfig;
import org.firstinspires.ftc.teamcode.config.UiLightConfig.Timing;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.ShooterYaw;

public final class LightController {
    private final UiLight ui;
    private final Shooter shooter;
    private final ShooterYaw yaw;
    private final Intake intake;

    private boolean atSpeed = false;
    private boolean locked = false;

    private long spdEdgeMs = 0;
    private long lckEdgeMs = 0;

    private boolean lastRawSpd = false;
    private boolean lastRawLck = false;

    public LightController(UiLight ui, Shooter shooter, ShooterYaw yaw, Intake intake) {
        this.ui = ui;
        this.shooter = shooter;
        this.yaw = yaw;
        this.intake = intake;
    }

    public void update(boolean enable) {
        if (!enable) {
            ui.setBase(UiLightConfig.UiState.PARK);
            ui.update();
            return;
        }

        boolean rawSpd = shooter.isAtTarget(TARGET_RPM_BAND);
        long now = nowMs();
        if (rawSpd != lastRawSpd) {
            spdEdgeMs = now;
            lastRawSpd = rawSpd;
        }
        if (rawSpd && now - spdEdgeMs >= Timing.SPEED_ON_MS) atSpeed = true;
        if (!rawSpd && now - spdEdgeMs >= Timing.SPEED_OFF_MS) atSpeed = false;

        boolean rawLck = yaw.isLockedOnTarget();
        if (rawLck != lastRawLck) {
            lckEdgeMs = now;
            lastRawLck = rawLck;
        }
        if (rawLck && now - lckEdgeMs >= Timing.LOCK_ON_MS) locked = true;
        if (!rawLck && now - lckEdgeMs >= Timing.LOCK_OFF_MS) locked = false;

        if (isJammed()) {
            ui.setBase(UiLightConfig.UiState.JAM);
        } else if (isIntaking()) {
            ui.setBase(UiLightConfig.UiState.INTAKE);
        } else if (atSpeed && locked) {
            ui.setBase(UiLightConfig.UiState.READY);
        } else if (!atSpeed) {
            ui.setBase(UiLightConfig.UiState.SPINUP);
        } else {
            ui.setBase(UiLightConfig.UiState.SEEKING);
        }

        ui.update();
    }

    private boolean isJammed() {
        return intake.isJammed();
    }

    private boolean isIntaking() {
        return intake.isRunning();
    }

    private static long nowMs() {
        return System.nanoTime() / 1_000_000L;
    }
}
