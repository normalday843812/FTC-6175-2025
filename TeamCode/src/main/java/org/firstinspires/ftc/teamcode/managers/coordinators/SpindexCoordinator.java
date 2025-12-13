package org.firstinspires.ftc.teamcode.managers.coordinators;

import static org.firstinspires.ftc.teamcode.config.AutoUnifiedConfig.TELEOP_SPINDEX_JIGGLE_DELTA;
import static org.firstinspires.ftc.teamcode.config.AutoUnifiedConfig.TELEOP_SPINDEX_JIGGLE_DWELL_S;
import static org.firstinspires.ftc.teamcode.config.AutoUnifiedConfig.TELEOP_SPINDEX_MAX_RECOVERY_ATTEMPTS;
import static org.firstinspires.ftc.teamcode.config.AutoUnifiedConfig.TELEOP_SPINDEX_VERIFY_DELAY_S;

import org.firstinspires.ftc.teamcode.GamepadMap;
import org.firstinspires.ftc.teamcode.managers.InventoryManager;
import org.firstinspires.ftc.teamcode.managers.SpindexerModel;
import org.firstinspires.ftc.teamcode.subsystems.SlotColorSensors;
import org.firstinspires.ftc.teamcode.subsystems.Spindexer;
import org.firstinspires.ftc.teamcode.util.Timer;

public class SpindexCoordinator {

    // Jam detection state machine
    private enum JamState {
        IDLE,           // No move in progress
        MOVING,         // Waiting for spindexer to settle
        VERIFYING,      // Checking if move succeeded
        RECOVERING,     // Jiggling to try to clear jam
        RETRY_MOVE,     // Re-attempting the move after jiggle
        RESYNC          // Giving up, rebuilding from sensors
    }

    private final Spindexer spindexer;
    private final SlotColorSensors sensors;
    private final InventoryManager inventory;

    // Jam detection state
    private JamState jamState = JamState.IDLE;
    private int moveSourceSlot = -1;
    private int moveTargetSlot = -1;
    private int recoveryAttempts = 0;
    private boolean expectEmptyAtFront = false;
    private final Timer jamTimer = new Timer();

    public SpindexCoordinator(Spindexer spindexer,
                              SlotColorSensors sensors,
                              InventoryManager inventory) {
        this.spindexer = spindexer;
        this.sensors = sensors;
        this.inventory = inventory;
    }

    public void start() {
        spindexer.start();
        syncFromSensors();
    }

    public void update(GamepadMap map) {
        syncModel();
        updateJamDetection();
        handleInputs(map);
    }

    // --- Slot Commands ---

    public void goToSlot(int slot) {
        spindexer.setSlot(slot);
        syncModel();
    }

    /**
     * Go to the nearest empty slot with jam detection.
     * After calling this, poll isSettled() to wait for completion (includes verification).
     */
    public boolean goToEmpty() {
        // Always trust front sensors before choosing an "empty" target.
        // With all sensors mounted at slot-0, the model can easily think the front bucket is EMPTY
        // when it actually contains a ball (e.g., after manual spindexing or a partial auto->teleop transition).
        if (sensors != null) {
            inventory.syncFromSensors(sensors, spindexer);
        } else {
            syncModel();
        }
        int slot = inventory.findNearestEmptySlot(spindexer);
        if (slot >= 0) {
            goToSlotWithVerification(slot, true);  // expect empty at target
            return true;
        }
        return false;
    }

    /**
     * Start a move with jam verification.
     * @param targetSlot the slot to move to
     * @param expectEmpty true if we expect the target slot to be empty
     */
    private void goToSlotWithVerification(int targetSlot, boolean expectEmpty) {
        if (jamState != JamState.IDLE) {
            // Already in a move - skip
            return;
        }

        int currentSlot = spindexer.getCommandedSlot();

        // If already at target slot, no need to move or verify
        if (currentSlot == targetSlot) {
            syncModel();
            return;
        }

        moveSourceSlot = currentSlot;
        moveTargetSlot = targetSlot;
        expectEmptyAtFront = expectEmpty;
        recoveryAttempts = 0;

        spindexer.setSlot(targetSlot);
        jamState = JamState.MOVING;
        jamTimer.resetTimer();
        syncModel();
    }

    /**
     * Update the jam detection state machine.
     */
    private void updateJamDetection() {
        switch (jamState) {
            case IDLE:
                // Nothing to do
                break;

            case MOVING:
                // Wait for spindexer to settle
                if (spindexer.isSettled()) {
                    jamState = JamState.VERIFYING;
                    jamTimer.resetTimer();
                }
                break;

            case VERIFYING:
                // Give sensors a moment to stabilize after movement
                if (jamTimer.getElapsedTimeSeconds() >= TELEOP_SPINDEX_VERIFY_DELAY_S) {
                    if (verifyMoveSucceeded()) {
                        // Move succeeded
                        jamState = JamState.IDLE;
                    } else {
                        // Move may have failed - try recovery
                        attemptRecovery();
                    }
                }
                break;

            case RECOVERING:
                // Wait for jiggle to complete
                if (spindexer.updateJiggle()) {
                    // Jiggle done, retry the move
                    spindexer.setSlot(moveTargetSlot);
                    jamState = JamState.RETRY_MOVE;
                    jamTimer.resetTimer();
                }
                break;

            case RETRY_MOVE:
                // Wait for retry move to settle
                if (spindexer.isSettled() && jamTimer.getElapsedTimeSeconds() >= TELEOP_SPINDEX_VERIFY_DELAY_S) {
                    if (verifyMoveSucceeded()) {
                        // Retry succeeded
                        jamState = JamState.IDLE;
                    } else {
                        // Still failed - resync from sensors
                        jamState = JamState.RESYNC;
                        jamTimer.resetTimer();
                    }
                }
                break;

            case RESYNC:
                // Rebuild model from sensors (accept sensor truth)
                syncFromSensors();
                jamState = JamState.IDLE;
                break;
        }
    }

    /**
     * Verify that the move succeeded by checking sensor state.
     */
    private boolean verifyMoveSucceeded() {
        // This codebase intentionally places all color sensors at the front (slot 0),
        // so verification can only use "front has ball" truth.
        if (!expectEmptyAtFront) {
            return true;
        }

        if (sensors == null) {
            return true;
        }

        // Expected EMPTY at front.
        if (!sensors.hasFrontBall()) {
            return true;
        }

        // Sensors still see a BALL at the front.
        // If the servo actually reached the commanded target slot, this isn't necessarily a mechanical jam;
        // it usually means the model thought this bucket was empty when it wasn't. Accept sensor truth and
        // let higher-level logic decide what to do next instead of jiggling forever.
        if (spindexer.getCurrentSlot() == moveTargetSlot) {
            inventory.syncFromSensors(sensors, spindexer);
            return true;
        }

        return false;
    }

    /**
     * Attempt to recover from a suspected jam.
     */
    private void attemptRecovery() {
        recoveryAttempts++;

        if (recoveryAttempts <= TELEOP_SPINDEX_MAX_RECOVERY_ATTEMPTS) {
            // Try jiggling to dislodge
            spindexer.startJiggle(TELEOP_SPINDEX_JIGGLE_DELTA, TELEOP_SPINDEX_JIGGLE_DELTA, TELEOP_SPINDEX_JIGGLE_DWELL_S);
            jamState = JamState.RECOVERING;
            jamTimer.resetTimer();
        } else {
            // Recovery failed - resync from sensors
            jamState = JamState.RESYNC;
            jamTimer.resetTimer();
        }
    }

    public int decideTargetSlot() {
        syncModel();
        return inventory.decideTargetSlot(spindexer);
    }

    // --- Ball Events ---

    public void onBallIntaked() {
        syncModel();
        inventory.onBallIntaked();
    }

    public void onBallShot() {
        syncModel();
        inventory.onShot();
    }

    // --- State Queries ---

    /**
     * Returns true when spindexer is settled AND jam detection is complete.
     */
    public boolean isSettled() {
        return jamState == JamState.IDLE && spindexer.isSettled();
    }

    /**
     * Returns true if spindexer is moving OR jam detection is in progress.
     */
    public boolean isMoving() {
        return jamState != JamState.IDLE || spindexer.isMoving();
    }

    /**
     * Returns true if jam detection is currently active.
     */
    public boolean isVerifying() {
        return jamState != JamState.IDLE;
    }

    public int getCurrentSlot() {
        return spindexer.getCurrentSlot();
    }

    public int getBallCount() {
        syncModel();
        return inventory.getModel().getBallCount();
    }

    public boolean isEmpty() {
        syncModel();
        return inventory.getModel().isEmpty();
    }

    public boolean isFull() {
        syncModel();
        return inventory.getModel().isFull();
    }

    public SpindexerModel.BallColor getFrontBucketContents() {
        syncModel();
        return inventory.getModel().getFrontBucketContents();
    }

    public SpindexerModel.BallColor getBucketContents(int bucket) {
        return inventory.getModel().getBucketContents(bucket);
    }

    // --- Sensor Sync ---

    public void syncFromSensors() {
        inventory.syncFromSensors(sensors, spindexer);
    }

    public boolean verifyModel() {
        return inventory.verifyModel(sensors, spindexer);
    }

    // --- Reset ---

    public void reset() {
        inventory.reset();
        spindexer.start();
    }

    public void clearAllState() {
        jamState = JamState.IDLE;
        moveSourceSlot = -1;
        moveTargetSlot = -1;
        recoveryAttempts = 0;
        expectEmptyAtFront = false;
        jamTimer.resetTimer();
        spindexer.stopJiggle();

        inventory.reset();
        syncModel();
    }

    // --- Internal ---

    private void handleInputs(GamepadMap map) {
        if (map.spindexerForward) {
            spindexer.stepForward();
            syncModel();
        }
        if (map.spindexerBackward) {
            spindexer.stepBackward();
            syncModel();
        }
    }

    private void syncModel() {
        inventory.getModel().setBucketAtFront(spindexer.getCommandedSlot());
    }
}
