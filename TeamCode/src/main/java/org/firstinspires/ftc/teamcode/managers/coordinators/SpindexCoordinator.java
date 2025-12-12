package org.firstinspires.ftc.teamcode.managers.coordinators;

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

    // Jam detection config
    private static final double VERIFY_DELAY_S = 0.15;
    private static final double JIGGLE_DELTA = 0.06;
    private static final double JIGGLE_DWELL_S = 0.12;
    private static final int MAX_RECOVERY_ATTEMPTS = 1;

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
        syncModel();
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

        int currentSlot = spindexer.getCurrentSlot();

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
                if (jamTimer.getElapsedTimeSeconds() >= VERIFY_DELAY_S) {
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
                if (spindexer.isSettled() && jamTimer.getElapsedTimeSeconds() >= VERIFY_DELAY_S) {
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
        // Check if all slots show balls - if so, we're actually full
        boolean allFull = true;
        for (int i = 0; i < 3; i++) {
            if (sensors.getColor(i) != SlotColorSensors.BallColor.BALL) {
                allFull = false;
                break;
            }
        }

        if (allFull) {
            // All sensors show balls - accept that we're full
            // Update model to match reality
            for (int i = 0; i < 3; i++) {
                inventory.getModel().setBucketContents(i, SpindexerModel.BallColor.BALL);
            }
            return true;
        }

        // If we expected the front to be empty, check sensor 0
        if (expectEmptyAtFront) {
            SlotColorSensors.BallColor frontSensor = sensors.getColor(0);
            if (frontSensor == SlotColorSensors.BallColor.BALL) {
                // Front sensor sees a ball but we expected empty - possible jam
                return false;
            }
        }

        // Move looks successful
        return true;
    }

    /**
     * Attempt to recover from a suspected jam.
     */
    private void attemptRecovery() {
        recoveryAttempts++;

        if (recoveryAttempts <= MAX_RECOVERY_ATTEMPTS) {
            // Try jiggling to dislodge
            spindexer.startJiggle(JIGGLE_DELTA, JIGGLE_DELTA, JIGGLE_DWELL_S);
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
        inventory.getModel().setBucketAtFront(spindexer.getCurrentSlot());
    }
}