package org.firstinspires.ftc.teamcode.managers.coordinators;

import static org.firstinspires.ftc.teamcode.config.ShooterYawConfig.AIM_BIAS_STEP_DEG;
import static org.firstinspires.ftc.teamcode.config.TeleOpShooterConfig.MANUAL_RPM_OVERRIDE_ENABLED;
import static org.firstinspires.ftc.teamcode.config.TeleOpShooterConfig.MANUAL_RPM_OVERRIDE_MAX_RPM;
import static org.firstinspires.ftc.teamcode.config.TeleOpShooterConfig.MANUAL_RPM_OVERRIDE_MIN_RPM;
import static org.firstinspires.ftc.teamcode.config.TeleOpShooterConfig.MANUAL_RPM_OVERRIDE_RATE_RPM_PER_S;
import static org.firstinspires.ftc.teamcode.config.TeleOpShooterConfig.MANUAL_RPM_OVERRIDE_TRIGGER_DEADBAND;

import org.firstinspires.ftc.teamcode.GamepadMap;
import org.firstinspires.ftc.teamcode.config.ShooterConfig;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.ShooterYaw;
import org.firstinspires.ftc.teamcode.subsystems.Transfer;
import org.firstinspires.ftc.teamcode.util.Timer;

public class ShootCoordinator {

    private final Shooter shooter;
    private final ShooterYaw shooterYaw;
    private final Transfer transfer;

    private boolean shootingMode = false;
    private double targetRpm = ShooterConfig.IDLE_RPM;
    private final Timer rpmAdjustTimer = new Timer();

    public ShootCoordinator(Shooter shooter, ShooterYaw shooterYaw, Transfer transfer) {
        this.shooter = shooter;
        this.shooterYaw = shooterYaw;
        this.transfer = transfer;
    }

    /**
     * Update coordinator state.
     * @param map gamepad inputs
     * @param managerControlsTransfer if true, don't issue transfer commands (manager owns them)
     * @return true if a shot occurred
     */
    public boolean update(GamepadMap map, boolean managerControlsTransfer) {
        handleShootingInputs(map, managerControlsTransfer);
        handleAimInputs(map, managerControlsTransfer);

        // Manual RPM override (TeleopManager disabled/manual mode).
        // Uses triggers as "RT up / LT down" to adjust RPM continuously.
        if (!managerControlsTransfer && MANUAL_RPM_OVERRIDE_ENABLED) {
            double dt = rpmAdjustTimer.getElapsedTimeSeconds();
            rpmAdjustTimer.resetTimer();
            dt = Math.max(0.0, Math.min(0.10, dt));

            double up = map.shooterUp;
            double down = map.shooterDown;
            boolean hasInput = up > MANUAL_RPM_OVERRIDE_TRIGGER_DEADBAND
                    || down > MANUAL_RPM_OVERRIDE_TRIGGER_DEADBAND;

            if (hasInput) {
                double delta = (up - down) * MANUAL_RPM_OVERRIDE_RATE_RPM_PER_S * dt;
                targetRpm = clamp(targetRpm + delta, MANUAL_RPM_OVERRIDE_MIN_RPM, MANUAL_RPM_OVERRIDE_MAX_RPM);
            }
        } else {
            rpmAdjustTimer.resetTimer();
        }

        // Always apply shooter RPM - manager doesn't control shooter
        shooter.setAutoRpm(targetRpm);

        // Only control transfer when manager doesn't own it
        if (!managerControlsTransfer) {
            applyTransferState();
        }

        boolean shot = shooter.shotOccurred();
        if (shot) {
            shooter.acknowledgeShotOccurred();
        }
        return shot;
    }

    public void setTargetRpm(double rpm) {
        targetRpm = rpm;
    }

    public void enterShootingMode() {
        shootingMode = true;
    }

    public void exitShootingMode() {
        shootingMode = false;
    }

    public void flick() {
        transfer.flick();
    }

    public void holdUp() {
        transfer.holdUp();
    }

    public void releaseHold() {
        transfer.releaseHold();
    }

    public boolean isShootingMode() {
        return shootingMode;
    }

    public boolean isReady(double rpmBand) {
        return shootingMode
                && shooter.isAtTarget(rpmBand)
                && shooterYaw.isLockedOnTarget();
    }

    public boolean isAtRpm(double rpmBand) {
        return shooter.isAtTarget(rpmBand);
    }

    public boolean isAimLocked() {
        return shooterYaw.isLockedOnTarget();
    }

    public boolean isTransferIdle() {
        return transfer.isIdle();
    }

    private void handleShootingInputs(GamepadMap map, boolean managerControlsTransfer) {
        // Shooting mode toggle always allowed
        if (map.leverToggle) {
            shootingMode = !shootingMode;
        }

        // Transfer commands only when manager doesn't own them
        if (!managerControlsTransfer) {
            if (shootingMode) {
                if (map.transferButtonHeld) {
                    transfer.holdUp();
                } else if (map.transferButton) {
                    transfer.flick();
                }
            }

            if (map.transferCrForward) {
                transfer.runTransfer(Transfer.CrState.FORWARD);
            } else if (map.transferCrReverse) {
                transfer.runTransfer(Transfer.CrState.REVERSE);
            }
        }
    }

    private void handleAimInputs(GamepadMap map, boolean managerControlsTransfer) {
        if (map.shooterYawAutoLockToggle) {
            shooterYaw.lockAllianceGoal();
        }

        if (map.resetShooterYaw) {
            shooterYaw.resetShooterYaw();
        }

        // Bumpers are used as manual intake controls when the TeleopManager is OFF,
        // so only apply yaw-bias adjustments when the manager owns the transfer.
        if (managerControlsTransfer) {
            if (map.shooterYawBiasInc) {
                shooterYaw.adjustAimBiasDeg(AIM_BIAS_STEP_DEG);
            } else if (map.shooterYawBiasDec) {
                shooterYaw.adjustAimBiasDeg(-AIM_BIAS_STEP_DEG);
            }
        }

        if (Math.abs(map.shooterYaw) > 0.1) {
            shooterYaw.adjustYaw(map.shooterYaw);
        }
    }

    private void applyTransferState() {
        if (shootingMode) {
            transfer.raiseLever();
        } else {
            transfer.lowerLever();
        }
    }

    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }
}
