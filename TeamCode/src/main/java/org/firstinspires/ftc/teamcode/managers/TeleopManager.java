package org.firstinspires.ftc.teamcode.managers;

import static org.firstinspires.ftc.teamcode.config.AutoUnifiedConfig.TELEOP_FEED_DWELL_S;
import static org.firstinspires.ftc.teamcode.config.AutoUnifiedConfig.TELEOP_MANAGER_TELEMETRY_ENABLED;
import static org.firstinspires.ftc.teamcode.config.AutoUnifiedConfig.TELEOP_SHOT_TIMEOUT_S;
import static org.firstinspires.ftc.teamcode.config.AutoUnifiedConfig.TARGET_RPM_BAND;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.GamepadMap;
import org.firstinspires.ftc.teamcode.config.ShooterConfig;
import org.firstinspires.ftc.teamcode.config.UiLightConfig;
import org.firstinspires.ftc.teamcode.managers.coordinators.IntakeCoordinator;
import org.firstinspires.ftc.teamcode.managers.coordinators.ShootCoordinator;
import org.firstinspires.ftc.teamcode.managers.coordinators.SpindexCoordinator;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.ShooterYaw;
import org.firstinspires.ftc.teamcode.subsystems.SlotColorSensors;
import org.firstinspires.ftc.teamcode.subsystems.Spindexer;
import org.firstinspires.ftc.teamcode.subsystems.Transfer;
import org.firstinspires.ftc.teamcode.util.TelemetryHelper;
import org.firstinspires.ftc.teamcode.util.Timer;

public class TeleopManager {

    public enum State {
        INTAKING,
        LOADING,
        INDEXING,
        READY,
        SHOOTING
    }

    private final IntakeCoordinator intakeCoord;
    private final ShootCoordinator shootCoord;
    private final SpindexCoordinator spindexCoord;

    private final Intake intake;
    private final Shooter shooter;
    private final ShooterYaw shooterYaw;
    private final Spindexer spindexer;
    private final Transfer transfer;
    private final SlotColorSensors sensors;
    private final UiLight ui;

    private final TelemetryHelper tele;
    private final Timer stateTimer = new Timer();

    private boolean enabled = false;
    private State state = State.INTAKING;
    private boolean waitingForButtonRelease = false;
    private double targetRpm = ShooterConfig.IDLE_RPM;

    public TeleopManager(Intake intake,
                         Shooter shooter,
                         ShooterYaw shooterYaw,
                         Spindexer spindexer,
                         Transfer transfer,
                         SlotColorSensors sensors,
                         InventoryManager inventory,
                         UiLight ui,
                         OpMode opmode) {
        this.intake = intake;
        this.shooter = shooter;
        this.shooterYaw = shooterYaw;
        this.spindexer = spindexer;
        this.transfer = transfer;
        this.sensors = sensors;
        this.ui = ui;

        this.intakeCoord = new IntakeCoordinator(intake, sensors);
        this.shootCoord = new ShootCoordinator(shooter, shooterYaw, transfer);
        this.spindexCoord = new SpindexCoordinator(spindexer, sensors, inventory);

        this.tele = new TelemetryHelper(opmode, TELEOP_MANAGER_TELEMETRY_ENABLED);
    }

    public void setEnabled(boolean enabled) {
        if (this.enabled == enabled) return;
        this.enabled = enabled;

        if (enabled) {
            enterState(State.INTAKING);
        } else {
            intakeCoord.forceStop();
            shootCoord.exitShootingMode();
            shootCoord.setTargetRpm(ShooterConfig.IDLE_RPM);
        }
    }

    public boolean isEnabled() {
        return enabled;
    }

    public void setTargetRpm(double rpm) {
        this.targetRpm = rpm;
    }

    public State getState() {
        return state;
    }

    public void update(GamepadMap map) {
        sensors.update();

        if (map.clearAll) {
            clearAll();
        }

        // Coordinators process their gamepad inputs
        // When enabled, manager controls intake/transfer to prevent conflicts
        boolean ballDetected = intakeCoord.update(map, enabled);
        boolean shotOccurred = shootCoord.update(map, enabled);
        spindexCoord.update(map);

        // Track button release for held shooting
        if (waitingForButtonRelease && !map.transferButtonHeld) {
            waitingForButtonRelease = false;
        }

        if (enabled) {
            runStateMachine(map, ballDetected, shotOccurred);
            updateLights();
        }

        // Execute all subsystems
        intake.operate();
        shooter.operate();
        shooterYaw.operate();
        spindexer.operate();
        transfer.operate();

        addTelemetry();
    }

    private void runStateMachine(GamepadMap map,
                                 boolean ballDetected,
                                 boolean shotOccurred) {
        switch (state) {
            case INTAKING:
                handleIntaking(map, ballDetected);
                break;
            case LOADING:
                handleLoading();
                break;
            case INDEXING:
                handleIndexing();
                break;
            case READY:
                handleReady(map, shotOccurred);
                break;
            case SHOOTING:
                handleShooting(map, shotOccurred);
                break;
        }
    }

    private void handleIntaking(GamepadMap map, boolean ballDetected) {
        // Check if ball is at front (from model) - if so, raise transfer
        boolean ballAtFront = spindexCoord.getFrontBucketContents() == SpindexerModel.BallColor.BALL;

        // Shooter idle during intaking
        shootCoord.setTargetRpm(ShooterConfig.IDLE_RPM);

        if (ballAtFront) {
            // Ball at front - keep transfer up, CR forward to hold ball
            transfer.raiseLever();
            transfer.runTransfer(Transfer.CrState.FORWARD);
            intakeCoord.setDesiredState(false, false);
        } else {
            // No ball at front - intake normally
            intakeCoord.setDesiredState(true, true);  // running=true, reversing=true (REVERSE for intake)
            transfer.lowerLever();  // Lever down to accept ball
            transfer.runTransfer(Transfer.CrState.REVERSE);
        }

        // Ball detected - start loading
        if (ballDetected) {
            spindexCoord.onBallIntaked();
            intakeCoord.setDesiredState(false, false);  // Stop intake briefly
            enterState(State.LOADING);
            return;
        }

        // User manually enters shooting mode OR manually spindexed ball to front
        if (shootCoord.isShootingMode() || ballAtFront) {
            intakeCoord.setDesiredState(false, false);
            shootCoord.enterShootingMode();
            enterState(State.READY);
            return;
        }

        // Auto-enter ready when full
        if (spindexCoord.isFull()) {
            shootCoord.enterShootingMode();
            enterState(State.READY);
        }
    }

    private void handleLoading() {
        // Push ball into bucket
        shootCoord.enterShootingMode();
        shootCoord.setTargetRpm(ShooterConfig.IDLE_RPM);
        transfer.raiseLever();  // Lever up to keep ball in
        transfer.runTransfer(Transfer.CrState.FORWARD);
        intakeCoord.setDesiredState(false, false);  // Stop intake

        // Wait for dwell time
        if (stateTimer.getElapsedTimeSeconds() >= TELEOP_FEED_DWELL_S) {
            enterState(State.INDEXING);
        }
    }

    private void handleIndexing() {
        boolean ballAtFront = spindexCoord.getFrontBucketContents() == SpindexerModel.BallColor.BALL;

        // Shooter idle during indexing
        shootCoord.setTargetRpm(ShooterConfig.IDLE_RPM);

        // Transfer up during indexing (keeping ball in), CR forward to hold ball
        transfer.raiseLever();
        transfer.runTransfer(Transfer.CrState.FORWARD);
        intakeCoord.setDesiredState(false, false);  // Intake off during indexing

        // Wait for spindexer to settle (goToEmpty was called in enterState)
        if (!spindexCoord.isSettled()) {
            return;
        }

        // Spindexer is settled - decide next state
        if (spindexCoord.isFull()) {
            // Full - go to ready state
            shootCoord.enterShootingMode();
            enterState(State.READY);
        } else if (ballAtFront) {
            // User manually spindexed ball to front - go to ready
            shootCoord.enterShootingMode();
            enterState(State.READY);
        } else {
            // At empty slot (front is empty) - go back to intaking
            shootCoord.exitShootingMode();
            enterState(State.INTAKING);
        }
    }

    private void handleReady(GamepadMap map, boolean shotOccurred) {
        boolean isFull = spindexCoord.isFull();
        boolean ballAtFront = spindexCoord.getFrontBucketContents() == SpindexerModel.BallColor.BALL;

        // Maintain ready state - transfer always up, CR forward
        shootCoord.enterShootingMode();
        transfer.raiseLever();
        transfer.runTransfer(Transfer.CrState.FORWARD);

        if (isFull) {
            // Full - shooter at max, intake runs FORWARD to push balls through
            shootCoord.setTargetRpm(ShooterConfig.MAX_RPM);
            intakeCoord.setDesiredState(true, false);  // running=true, reversing=false (FORWARD)
        } else {
            // Not full but ready to shoot - shooter at max when ball at front
            if (ballAtFront) {
                shootCoord.setTargetRpm(ShooterConfig.MAX_RPM);
                intakeCoord.setDesiredState(false, false);  // Intake off
            } else {
                // No ball at front and not full - go back to intaking
                shootCoord.exitShootingMode();
                enterState(State.INTAKING);
                return;
            }
        }

        // Check for shot (catches quick shots)
        if (shotOccurred) {
            handleShotOccurred();
            return;
        }

        // User exits shooting mode manually (only if not full - if full, stay ready)
        if (!shootCoord.isShootingMode() && !isFull) {
            enterState(State.INTAKING);
            return;
        }

        // User initiates a shot (and we're not waiting for release)
        if (!waitingForButtonRelease) {
            if (map.transferButtonHeld || map.transferButton) {
                enterState(State.SHOOTING);
            }
        }
    }

    private void handleShooting(GamepadMap map, boolean shotOccurred) {
        // Maintain shooting state - shooter at max RPM
        shootCoord.setTargetRpm(ShooterConfig.MAX_RPM);
        transfer.raiseLever();  // Lever up for shooting
        transfer.runTransfer(Transfer.CrState.FORWARD);

        // Shot detected
        if (shotOccurred) {
            handleShotOccurred();
            return;
        }

        // Button released without shot detection - go back to ready
        if (!map.transferButtonHeld && transfer.isIdle()) {
            enterState(State.READY);
            return;
        }

        // Timeout fallback
        if (stateTimer.getElapsedTimeSeconds() >= TELEOP_SHOT_TIMEOUT_S) {
            // Assume shot happened or failed, go back to ready
            shootCoord.releaseHold();
            enterState(State.READY);
        }
    }

    private void clearAll() {
        waitingForButtonRelease = false;

        // Clear persistent + model state so logic doesn't "think" we still have balls
        PersistentBallState.reset();
        spindexCoord.clearAllState();

        // Stop/neutral all mechanisms
        intakeCoord.forceStop();
        intake.clearJam();
        shootCoord.exitShootingMode();
        shootCoord.setTargetRpm(ShooterConfig.IDLE_RPM);
        shooter.resetShotLogic();
        shooterYaw.resetAimBiasDeg();

        transfer.lowerLever();
        transfer.runTransfer(Transfer.CrState.OFF);

        enterState(State.INTAKING);
    }

    private void handleShotOccurred() {
        spindexCoord.onBallShot();
        shootCoord.releaseHold();
        waitingForButtonRelease = true;

        if (spindexCoord.isEmpty()) {
            // All empty - go back to intaking
            shootCoord.exitShootingMode();
            enterState(State.INTAKING);
        } else {
            // Still have balls - stay ready to shoot more
            // But also turn intake back on if not full
            enterState(State.READY);
        }
    }

    private void enterState(State newState) {
        state = newState;
        stateTimer.resetTimer();

        // State entry actions
        switch (newState) {
            case INDEXING:
                spindexCoord.goToEmpty();
                break;
            case READY:
                shootCoord.enterShootingMode();
                break;
            case INTAKING:
                waitingForButtonRelease = false;
                break;
        }
    }

    private void updateLights() {
        if (ui == null) return;

        switch (state) {
            case INTAKING:
                ui.setBase(UiLightConfig.UiState.INTAKE);
                break;
            case LOADING:
            case INDEXING:
                ui.setBase(UiLightConfig.UiState.NAVIGATING);
                break;
            case READY:
                boolean ready = shootCoord.isReady(TARGET_RPM_BAND);
                ui.setBase(ready ? UiLightConfig.UiState.READY : UiLightConfig.UiState.SPINUP);
                break;
            case SHOOTING:
                ui.setBase(UiLightConfig.UiState.READY);
                break;
        }

        ui.update();
    }

    private void addTelemetry() {
        tele.addLine("=== TELEOP MANAGER ===")
                .addData("Enabled", "%b", enabled)
                .addData("State", state::name)
                .addData("StateTime", "%.2f", stateTimer.getElapsedTimeSeconds())
                .addData("BallCount", "%d", spindexCoord.getBallCount())
                .addData("IsFull", "%b", spindexCoord.isFull())
                .addData("IsEmpty", "%b", spindexCoord.isEmpty())
                .addData("ShootingMode", "%b", shootCoord.isShootingMode())
                .addData("IntakeRunning", "%b", intakeCoord.isRunning())
                .addData("SpindexSettled", "%b", spindexCoord.isSettled())
                .addData("WaitingRelease", "%b", waitingForButtonRelease)
                .addData("TargetRPM", "%.0f", targetRpm);

        tele.addLine("--- Buckets ---")
                .addData("Slot0", spindexCoord.getBucketContents(0)::name)
                .addData("Slot1", spindexCoord.getBucketContents(1)::name)
                .addData("Slot2", spindexCoord.getBucketContents(2)::name)
                .addData("Front", "%d", spindexCoord.getCurrentSlot());
    }
}
