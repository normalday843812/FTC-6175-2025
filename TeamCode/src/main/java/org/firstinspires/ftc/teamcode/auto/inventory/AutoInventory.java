package org.firstinspires.ftc.teamcode.auto.inventory;

import androidx.annotation.NonNull;

public class AutoInventory {
    public enum PickupPreference {NONE, PREFER_PURPLE, PREFER_GREEN}

    private int purple = 0;
    private int green = 0;

    private final int capacity;
    private final int targetPurple;
    private final int targetGreen;

    public AutoInventory() {
        this(3, 2, 1);
    }

    public AutoInventory(int capacity) {
        this(capacity, Math.min(2, capacity), Math.max(0, capacity - Math.min(2, capacity)));
    }

    public AutoInventory(int capacity, int targetPurple, int targetGreen) {
        this.capacity = Math.max(0, capacity);
        this.targetPurple = Math.max(0, targetPurple);
        this.targetGreen = Math.max(0, targetGreen);
    }

    public void onCapturedPurple() {
        purple++;
    }

    public void onCapturedGreen() {
        green++;
    }

    public void clear() {
        purple = 0;
        green = 0;
    }

    // Current counts
    public int count() {
        return purple + green;
    }

    public int purpleCount() {
        return purple;
    }

    public int greenCount() {
        return green;
    }

    public int capacity() {
        return capacity;
    }

    public boolean isFull() {
        return count() >= capacity;
    }

    public int neededPurple() {
        return Math.max(0, targetPurple - purple);
    }

    public int neededGreen() {
        return Math.max(0, targetGreen - green);
    }

    public boolean needsPurple() {
        return neededPurple() > 0;
    }

    public boolean needsGreen() {
        return neededGreen() > 0;
    }

    /**
     * Preference logic for capacity=3 with target 2 purple + 1 green:
     * 0/0: no preference (both needed)
     * 1/0: no preference (both needed)
     * 2/0: prefer green (needG>0, needP==0)
     * 1/1: prefer purple (needP>0, needG==0)
     * 0/1: prefer purple (needP>0, needG==0)
     * full: NONE
     * General rule if:
     * Full: NONE
     * Both needed: NONE
     * Only purple needed: PREFER_PURPLE
     * Only green needed: PREFER_GREEN
     * Neither needed: NONE (should be full)
     */
    public PickupPreference getPickupPreference() {
        if (isFull()) return PickupPreference.NONE;

        boolean needP = needsPurple();
        boolean needG = needsGreen();

        if (needP && needG) return PickupPreference.NONE;
        if (needP) return PickupPreference.PREFER_PURPLE;
        if (needG) return PickupPreference.PREFER_GREEN;
        return PickupPreference.NONE;
    }

    @NonNull
    @Override
    public String toString() {
        return "Inventory{ P:" + purple + " G:" + green + " / Cap:" + capacity +
                " | Target P:" + targetPurple + " G:" + targetGreen + " }";
    }
}
