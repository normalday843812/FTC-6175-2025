package org.firstinspires.ftc.teamcode.managers.coordinators;

import org.firstinspires.ftc.teamcode.GamepadMap;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.ShooterYaw;
import org.firstinspires.ftc.teamcode.subsystems.Transfer;

public class ShootCoordinator {

    private final Shooter shooter;
    private final ShooterYaw shooterYaw;
    private final Transfer transfer;

    private boolean shootingMode = false;
    private double targetRpm = 0.0;

    public ShootCoordinator(Shooter shooter, ShooterYaw shooterYaw, Transfer transfer) {
        this.shooter = shooter;
        this.shooterYaw = shooterYaw;
        this.transfer = transfer;
    }

    public boolean update(GamepadMap map) {
        handleShootingInputs(map);
        handleAimInputs(map);
        applyState();
        return shooter.shotOccurred();
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

    private void handleShootingInputs(GamepadMap map) {
        if (map.shootingModeToggle) {
            shootingMode = !shootingMode;
        }

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

    private void handleAimInputs(GamepadMap map) {
        if (map.shooterYawAutoLockToggle) {
            shooterYaw.lockAllianceGoal();
        }

        if (map.resetShooterYaw) {
            shooterYaw.resetShooterYaw();
        }

        if (Math.abs(map.shooterYaw) > 0.1) {
            shooterYaw.adjustYaw(map.shooterYaw);
        }
    }

    private void applyState() {
        shooter.setAutoRpm(targetRpm);

        if (shootingMode) {
            transfer.raiseLever();
        } else {
            transfer.lowerLever();
        }
    }
}