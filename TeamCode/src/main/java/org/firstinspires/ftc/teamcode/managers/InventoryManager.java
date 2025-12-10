package org.firstinspires.ftc.teamcode.managers;

import static org.firstinspires.ftc.teamcode.config.AutoUnifiedConfig.PREFER_CLOCKWISE_ON_TIE;

import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.config.DecodeGameConfig;
import org.firstinspires.ftc.teamcode.subsystems.SlotColorSensors;
import org.firstinspires.ftc.teamcode.subsystems.Spindexer;

public class InventoryManager {
    private final SpindexerModel model = new SpindexerModel();

    // Intake set tracking (field positions to visit)
    private final boolean[] visitedSets = new boolean[]{false, false, false};

    public SpindexerModel getModel() {
        return model;
    }

    // --- Pattern Management (delegates to model) ---

    public void setPatternFromTagId(int tagId) {
        boolean[] boolPattern = DecodeGameConfig.patternForTag(tagId);
        if (boolPattern != null) {
            SpindexerModel.BallColor[] colors = new SpindexerModel.BallColor[boolPattern.length];
            for (int i = 0; i < boolPattern.length; i++) {
                colors[i] = boolPattern[i] ? SpindexerModel.BallColor.PURPLE : SpindexerModel.BallColor.GREEN;
            }
            model.setPattern(colors);
        }
    }

    public boolean isPatternKnown() {
        return model.getNextPatternColor() != null || !model.isPatternComplete();
    }

    public boolean wantPurpleThisShot() {
        SpindexerModel.BallColor next = model.getNextPatternColor();
        return next == SpindexerModel.BallColor.PURPLE;
    }

    public void setWantPurple(boolean wantPurple) {
        // Override pattern - set a single-shot pattern
        SpindexerModel.BallColor color = wantPurple ?
                SpindexerModel.BallColor.PURPLE : SpindexerModel.BallColor.GREEN;
        model.setPattern(color);
    }

    public void onShot() {
        model.onBallShot();
    }

    // --- Bucket Decisions (uses model, not sensors) ---

    public int decideTargetSlot(SlotColorSensors slots, Spindexer spx) {
        // Sync model with spindexer position
        model.setBucketAtFront(spx.getCurrentSlot());

        // If pattern known, find bucket with needed color
        SpindexerModel.BallColor needed = model.getNextPatternColor();
        if (needed != null) {
            int bucket = model.findBucketWithColor(needed);
            if (bucket >= 0) return bucket;
        }

        // No pattern or color not found - find any non-empty bucket
        for (int i = 0; i < 3; i++) {
            if (model.getBucketContents(i) != SpindexerModel.BallColor.EMPTY) {
                return i;
            }
        }

        return -1; // All empty
    }

    public int findNearestEmptySlot(SlotColorSensors slots, Spindexer spx) {
        model.setBucketAtFront(spx.getCurrentSlot());
        int direction = PREFER_CLOCKWISE_ON_TIE ?
                SpindexerModel.DIRECTION_CW : SpindexerModel.DIRECTION_CCW;
        return model.findNearestEmptyBucket(direction);
    }

    // --- Ball Events (update model) ---

    public void onBallIntaked(SlotColorSensors.BallColor color) {
        model.onBallIntaked(color);
    }

    public void onBallIntaked(SpindexerModel.BallColor color) {
        model.onBallIntaked(color);
    }

    // --- Sensor Verification ---

    public void syncFromSensors(SlotColorSensors slots, Spindexer spx) {
        model.setBucketAtFront(spx.getCurrentSlot());
        model.rebuildFromSensors(slots);
    }

    public boolean verifyModel(SlotColorSensors slots, Spindexer spx) {
        model.setBucketAtFront(spx.getCurrentSlot());
        return model.verifyAllBuckets(slots);
    }

    // --- Intake Pose Management ---

    public Pose nextIntakePose(boolean isRed) {
        Pose[] sets = isRed ? DecodeGameConfig.INTAKE_SETS_RED : DecodeGameConfig.INTAKE_SETS_BLUE;
        for (int i = 0; i < 3; i++) if (!visitedSets[i]) return sets[i];
        return sets[2];
    }

    public void markOneIntakeSetVisited() {
        for (int i = 0; i < 3; i++) {
            if (!visitedSets[i]) {
                visitedSets[i] = true;
                break;
            }
        }
    }

    public boolean setsRemain() {
        return !(visitedSets[0] && visitedSets[1] && visitedSets[2]);
    }

    // --- Reset ---

    public void reset() {
        model.reset();
        visitedSets[0] = false;
        visitedSets[1] = false;
        visitedSets[2] = false;
    }
}