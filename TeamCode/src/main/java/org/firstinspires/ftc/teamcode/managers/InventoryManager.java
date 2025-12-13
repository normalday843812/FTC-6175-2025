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


    public void onShot() {
        model.onBallShot();
    }


    public int decideTargetSlot(Spindexer spx) {
        model.setBucketAtFront(spx.getCommandedSlot());
        return model.findBallBucket();
    }

    public int findNearestEmptySlot(Spindexer spx) {
        model.setBucketAtFront(spx.getCommandedSlot());
        int direction = PREFER_CLOCKWISE_ON_TIE ?
                SpindexerModel.DIRECTION_CW : SpindexerModel.DIRECTION_CCW;
        return model.findNearestEmptyBucket(direction);
    }

    // --- Ball Events (update model) ---

    public void onBallIntaked() {
        model.onBallIntaked();
    }

    // --- Sensor Verification ---

    public void syncFromSensors(SlotColorSensors slots, Spindexer spx) {
        model.setBucketAtFront(spx.getCommandedSlot());
        model.rebuildFromSensors(slots);
    }

    public boolean verifyModel(SlotColorSensors slots, Spindexer spx) {
        model.setBucketAtFront(spx.getCommandedSlot());
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

    // --- Convenience methods ---

    public int getBallCount() {
        return model.getBallCount();
    }

    public boolean hasBalls() {
        return model.getBallCount() > 0;
    }

    public boolean hasEmptySlots() {
        return model.getBallCount() < SpindexerModel.NUM_BUCKETS;
    }

    // --- Reset ---

    public void reset() {
        model.reset();
        visitedSets[0] = false;
        visitedSets[1] = false;
        visitedSets[2] = false;
    }
}
