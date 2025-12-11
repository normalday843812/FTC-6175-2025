package org.firstinspires.ftc.teamcode.managers.coordinators;

import org.firstinspires.ftc.teamcode.GamepadMap;
import org.firstinspires.ftc.teamcode.managers.InventoryManager;
import org.firstinspires.ftc.teamcode.managers.SpindexerModel;
import org.firstinspires.ftc.teamcode.subsystems.SlotColorSensors;
import org.firstinspires.ftc.teamcode.subsystems.Spindexer;

public class SpindexCoordinator {

    private final Spindexer spindexer;
    private final SlotColorSensors sensors;
    private final InventoryManager inventory;

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
        handleInputs(map);
    }

    // --- Slot Commands ---

    public void goToSlot(int slot) {
        spindexer.setSlot(slot);
        syncModel();
    }

    public boolean goToEmpty() {
        syncModel();
        int slot = inventory.findNearestEmptySlot(sensors, spindexer);
        if (slot >= 0) {
            spindexer.setSlot(slot);
            return true;
        }
        return false;
    }

    public int decideTargetSlot() {
        syncModel();
        return inventory.decideTargetSlot(sensors, spindexer);
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

    public boolean isSettled() {
        return spindexer.isSettled();
    }

    public boolean isMoving() {
        return spindexer.isMoving();
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