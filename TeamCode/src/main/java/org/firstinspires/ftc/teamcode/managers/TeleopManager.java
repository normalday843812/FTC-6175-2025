package org.firstinspires.ftc.teamcode.managers;

import static org.firstinspires.ftc.teamcode.config.AutoUnifiedConfig.TELEOP_FEED_DWELL_S;
import static org.firstinspires.ftc.teamcode.config.AutoUnifiedConfig.TELEOP_LOAD_FRONT_CONFIRM_CYCLES;
import static org.firstinspires.ftc.teamcode.config.AutoUnifiedConfig.TELEOP_LOAD_TIMEOUT_S;
import static org.firstinspires.ftc.teamcode.config.AutoUnifiedConfig.TELEOP_MANAGER_TELEMETRY_ENABLED;
import static org.firstinspires.ftc.teamcode.config.AutoUnifiedConfig.TELEOP_SHOT_TIMEOUT_S;
import static org.firstinspires.ftc.teamcode.config.AutoUnifiedConfig.TARGET_RPM_BAND;
import static org.firstinspires.ftc.teamcode.config.TeleOpShooterConfig.MANUAL_RPM_OVERRIDE_ENABLED;
import static org.firstinspires.ftc.teamcode.config.TeleOpShooterConfig.MANUAL_RPM_OVERRIDE_LATCH_REQUIRES_FULL;
import static org.firstinspires.ftc.teamcode.config.TeleOpShooterConfig.MANUAL_RPM_OVERRIDE_MAX_RPM;
import static org.firstinspires.ftc.teamcode.config.TeleOpShooterConfig.MANUAL_RPM_OVERRIDE_MIN_RPM;
import static org.firstinspires.ftc.teamcode.config.TeleOpShooterConfig.MANUAL_RPM_OVERRIDE_RATE_RPM_PER_S;
import static org.firstinspires.ftc.teamcode.config.TeleOpShooterConfig.MANUAL_RPM_OVERRIDE_TRIGGER_DEADBAND;

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
    private final Timer rpmAdjustTimer = new Timer();

    private boolean enabled = false;
    private State state = State.INTAKING;
    private double targetRpm = ShooterConfig.IDLE_RPM;
    private boolean rpmOverrideLatched = false;
    private double rpmOverrideRpm = ShooterConfig.MAX_RPM;
    private boolean pendingBallIntake = false;
    private int loadFrontConfirmCount = 0;
    private int indexRetryCount = 0;

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
            spindexCoord.syncFromSensors();
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
        setShooterTargetRpm(rpm);
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
        boolean frontHasBall = sensors != null && sensors.hasFrontBall();
        boolean isFull = spindexCoord.isFull();
        boolean wantsShoot = shootCoord.isShootingMode();

        // Shooter idle during intaking
        setShooterTargetRpm(ShooterConfig.IDLE_RPM);

        // Ball detected (usually intake sensor) - start loading so it seats into slot-0.
        // This must take priority over any "front has ball" checks so we don't skip model updates.
        if (ballDetected) {
            pendingBallIntake = true;
            intakeCoord.setDesiredState(false, false);  // Stop intake briefly
            enterState(State.LOADING);
            return;
        }

        // If full, stop intaking and go to READY (intake will be put into FORWARD block mode there).
        if (isFull) {
            enterState(State.READY);
            return;
        }

        // If there's a ball at the front:
        // - If the driver wants to shoot, go READY.
        // - Otherwise, index to an empty front bucket so we can keep intaking.
        if (frontHasBall) {
            transfer.raiseLever();
            transfer.runTransfer(Transfer.CrState.FORWARD);
            intakeCoord.setDesiredState(false, false);

            if (wantsShoot) {
                enterState(State.READY);
            } else {
                // Ensure the model knows the current front bucket really has a ball before choosing an empty slot.
                spindexCoord.syncFromSensors();
                enterState(State.INDEXING);
            }
            return;
        }

        // Front empty and not full: intake normally.
        intakeCoord.setDesiredState(true, true);  // running=true, reversing=true (REVERSE for intake)
        transfer.lowerLever();  // Lever down to accept ball
        transfer.runTransfer(Transfer.CrState.REVERSE);
    }

    private void handleLoading() {
        // Push ball into bucket
        shootCoord.enterShootingMode();
        setShooterTargetRpm(ShooterConfig.IDLE_RPM);
        transfer.raiseLever();  // Lever up to keep ball in
        transfer.runTransfer(Transfer.CrState.FORWARD);
        intakeCoord.setDesiredState(false, false);  // Stop intake

        boolean frontHasBall = sensors != null && sensors.hasFrontBall();
        if (frontHasBall) {
            loadFrontConfirmCount++;
        } else {
            loadFrontConfirmCount = 0;
        }

        // Require the ball to actually reach slot-0/front sensors before we update the model.
        if (stateTimer.getElapsedTimeSeconds() >= TELEOP_FEED_DWELL_S
                && frontHasBall
                && loadFrontConfirmCount >= Math.max(1, TELEOP_LOAD_FRONT_CONFIRM_CYCLES)) {
            if (pendingBallIntake) {
                spindexCoord.onBallIntaked();
                pendingBallIntake = false;
            }
            enterState(State.INDEXING);
            return;
        }

        if (stateTimer.getElapsedTimeSeconds() >= Math.max(TELEOP_FEED_DWELL_S, TELEOP_LOAD_TIMEOUT_S)) {
            // Failed to seat the ball into slot 0. Don't lie to the model; go back to intaking.
            pendingBallIntake = false;
            enterState(State.INTAKING);
        }
    }

    private void handleIndexing() {
        boolean frontHasBall = sensors != null && sensors.hasFrontBall();

        // Shooter idle during indexing
        setShooterTargetRpm(ShooterConfig.IDLE_RPM);

        // Transfer up during indexing (keeping ball in), CR forward to hold ball
        transfer.raiseLever();
        transfer.runTransfer(Transfer.CrState.FORWARD);
        intakeCoord.setDesiredState(false, false);  // Intake off during indexing

        // Wait for spindexer to settle (goToEmpty was called in enterState)
        if (!spindexCoord.isSettled()) {
            return;
        }

        // Update model from front sensor truth at the final indexed position.
        spindexCoord.syncFromSensors();
        frontHasBall = sensors != null && sensors.hasFrontBall();

        // If front is empty, we're ready to intake again (unless full).
        if (!frontHasBall) {
            if (spindexCoord.isFull()) {
                enterState(State.READY);
            } else {
                enterState(State.INTAKING);
            }
            return;
        }

        // Front still has a ball after attempting to go-to-empty:
        // - If model says full, we're actually full -> READY.
        // - Otherwise retry indexing a limited number of times to avoid loops.
        if (spindexCoord.isFull()) {
            enterState(State.READY);
            return;
        }

        indexRetryCount++;
        if (indexRetryCount >= SpindexerModel.NUM_BUCKETS) {
            // Give up and let the driver shoot/clear; we can't find an empty front right now.
            enterState(State.READY);
            return;
        }

        spindexCoord.goToEmpty();
    }

    private void handleReady(GamepadMap map, boolean shotOccurred) {
        boolean isFull = spindexCoord.isFull();
        boolean frontHasBall = sensors != null && sensors.hasFrontBall();
        boolean isEmpty = spindexCoord.isEmpty();

        // Maintain ready state - transfer always up, CR forward
        shootCoord.enterShootingMode();
        transfer.raiseLever();
        transfer.runTransfer(Transfer.CrState.FORWARD);

        if (isFull) {
            // Full - shooter at max by default, but allow a latched manual override using triggers.
            double rpm = applyLatchedRpmOverride(map, ShooterConfig.MAX_RPM, isFull, isEmpty);
            setShooterTargetRpm(rpm);
            intakeCoord.setDesiredState(true, false);  // running=true, reversing=false (FORWARD)
        } else {
            // Not full but ready to shoot - shooter at max when ball at front
            if (frontHasBall) {
                double rpm = applyLatchedRpmOverride(map, ShooterConfig.MAX_RPM, isFull, isEmpty);
                setShooterTargetRpm(rpm);
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

        // User initiates a shot (simple: press = flick)
        if (map.transferButton) {
            shootCoord.flick();
            enterState(State.SHOOTING);
        }
    }

    private void handleShooting(GamepadMap map, boolean shotOccurred) {
        boolean isFull = spindexCoord.isFull();
        boolean isEmpty = spindexCoord.isEmpty();
        // Maintain shooting state - shooter at max RPM by default, with optional latched override.
        double rpm = applyLatchedRpmOverride(map, ShooterConfig.MAX_RPM, isFull, isEmpty);
        setShooterTargetRpm(rpm);
        transfer.raiseLever();  // Lever up for shooting
        transfer.runTransfer(Transfer.CrState.FORWARD);

        // Shot detected
        if (shotOccurred) {
            handleShotOccurred();
            return;
        }

        // Timeout fallback
        if (stateTimer.getElapsedTimeSeconds() >= TELEOP_SHOT_TIMEOUT_S) {
            // Assume shot happened or failed, go back to ready
            enterState(State.READY);
        }
    }

    private void clearAll() {
        rpmOverrideLatched = false;
        rpmOverrideRpm = ShooterConfig.MAX_RPM;
        pendingBallIntake = false;
        loadFrontConfirmCount = 0;

        // Clear persistent + model state so logic doesn't "think" we still have balls
        PersistentBallState.reset();
        spindexCoord.clearAllState();

        // Stop/neutral all mechanisms
        intakeCoord.forceStop();
        intake.clearJam();
        shootCoord.exitShootingMode();
        setShooterTargetRpm(ShooterConfig.IDLE_RPM);
        shooter.resetShotLogic();
        shooterYaw.resetAimBiasDeg();

        transfer.lowerLever();
        transfer.runTransfer(Transfer.CrState.OFF);

        enterState(State.INTAKING);
    }

    private void handleShotOccurred() {
        spindexCoord.onBallShot();

        if (spindexCoord.isEmpty()) {
            rpmOverrideLatched = false;
            rpmOverrideRpm = ShooterConfig.MAX_RPM;
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
        rpmAdjustTimer.resetTimer();

        // State entry actions
        switch (newState) {
            case INDEXING:
                indexRetryCount = 0;
                spindexCoord.goToEmpty();
                break;
            case LOADING:
                loadFrontConfirmCount = 0;
                break;
            case READY:
                shootCoord.enterShootingMode();
                break;
            case INTAKING:
                pendingBallIntake = false;
                break;
        }
    }

    private void setShooterTargetRpm(double rpm) {
        this.targetRpm = rpm;
        shootCoord.setTargetRpm(rpm);
        shooter.setAutoRpm(rpm);
    }

    private double applyLatchedRpmOverride(GamepadMap map, double baseRpm, boolean isFull, boolean isEmpty) {
        if (!MANUAL_RPM_OVERRIDE_ENABLED) {
            return baseRpm;
        }

        if (isEmpty) {
            rpmOverrideLatched = false;
            rpmOverrideRpm = ShooterConfig.MAX_RPM;
            return baseRpm;
        }

        double dt = rpmAdjustTimer.getElapsedTimeSeconds();
        rpmAdjustTimer.resetTimer();
        dt = Math.max(0.0, Math.min(0.10, dt));

        double up = map.shooterUp;
        double down = map.shooterDown;
        boolean hasInput = up > MANUAL_RPM_OVERRIDE_TRIGGER_DEADBAND
                || down > MANUAL_RPM_OVERRIDE_TRIGGER_DEADBAND;

        // Only allow *starting* the override when we're full (so it doesn't fight normal ready logic).
        if (!rpmOverrideLatched && hasInput && (!MANUAL_RPM_OVERRIDE_LATCH_REQUIRES_FULL || isFull)) {
            rpmOverrideLatched = true;
            rpmOverrideRpm = baseRpm;
        }

        if (rpmOverrideLatched && hasInput) {
            double delta = (up - down) * MANUAL_RPM_OVERRIDE_RATE_RPM_PER_S * dt;
            rpmOverrideRpm = clamp(rpmOverrideRpm + delta, MANUAL_RPM_OVERRIDE_MIN_RPM, MANUAL_RPM_OVERRIDE_MAX_RPM);
        }

        return rpmOverrideLatched ? rpmOverrideRpm : baseRpm;
    }

    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
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
                .addData("TargetRPM", "%.0f", targetRpm)
                .addData("RpmOverrideLatched", "%b", rpmOverrideLatched)
                .addData("RpmOverrideRpm", "%.0f", rpmOverrideRpm)
                .addData("PendingIntake", "%b", pendingBallIntake)
                .addData("LoadFrontConfirm", "%d", loadFrontConfirmCount);

        tele.addLine("--- Buckets ---")
                .addData("Slot0", spindexCoord.getBucketContents(0)::name)
                .addData("Slot1", spindexCoord.getBucketContents(1)::name)
                .addData("Slot2", spindexCoord.getBucketContents(2)::name)
                .addData("Front", "%d", spindexCoord.getCurrentSlot());
    }
}
