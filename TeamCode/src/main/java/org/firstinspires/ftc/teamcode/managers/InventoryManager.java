package org.firstinspires.ftc.teamcode.managers;

import static org.firstinspires.ftc.teamcode.config.AutoUnifiedConfig.PREFER_CLOCKWISE_ON_TIE;

import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.config.DecodeGameConfig;
import org.firstinspires.ftc.teamcode.subsystems.SlotColorSensors;
import org.firstinspires.ftc.teamcode.subsystems.Spindexer;

public class InventoryManager {

    // true=purple, false=green
    private boolean[] pattern = null;
    private int shotsTaken = 0;
    private final boolean[] visitedSets = new boolean[]{false, false, false};
    private boolean wantPurple = false;
    private boolean wantPurpleSet = false;

    public void setPatternFromTagId(int tagId) {
        pattern = DecodeGameConfig.patternForTag(tagId);
    }

    public boolean isPatternKnown() {
        return pattern != null;
    }

    public boolean wantPurpleThisShot() {
        if (wantPurpleSet) return wantPurple;
        if (!isPatternKnown()) return false;
        int idx = Math.min(shotsTaken, 2);
        return pattern[idx];
    }

    public void setWantPurple(boolean wantPurple) {
        wantPurpleSet = true;
        this.wantPurple = wantPurple;
    }

    public void onShot() {
        shotsTaken++;
    }

    public Pose nextIntakePose(boolean isRed) {
        Pose[] sets = isRed ? DecodeGameConfig.INTAKE_SETS_RED : DecodeGameConfig.INTAKE_SETS_BLUE;
        for (int i = 0; i < 3; i++) if (!visitedSets[i]) return sets[i];
        return sets[2];
    }

    public void markOneIntakeSetVisited() {
        for (int i = 0; i < 3; i++)
            if (!visitedSets[i]) {
                visitedSets[i] = true;
                break;
            }
    }

    public boolean setsRemain() {
        return !(visitedSets[0] && visitedSets[1] && visitedSets[2]);
    }

    public int decideTargetSlot(SlotColorSensors slots, Spindexer spx) {
        int cur = spx.getCurrentSlot(); // Where the spindexer is currently rotated
        int best = -1;
        int bestDist = 999;

        // 1. Determine which color we want based on the Pattern (GPP, PGP, etc.)
        boolean wantPurple = false;
        boolean specificColorNeeded = false;

        if (isPatternKnown()) {
            wantPurple = wantPurpleThisShot();
            specificColorNeeded = true;
        }

        // 2. Iterate through physical Buckets (0, 1, 2)
        for (int targetBucket = 0; targetBucket < 3; targetBucket++) {

            // RELATIVE MAPPING: Which sensor is currently looking at 'targetBucket'?
            // Formula: (TargetBucket - CurrentPos + 3) % 3
            int sensorIndex = (targetBucket - cur + 3) % 3;

            SlotColorSensors.BallColor colorSeen = slots.getColor(sensorIndex);

            // If the bucket is empty, we can't shoot from it. Skip.
            if (colorSeen == SlotColorSensors.BallColor.NONE) continue;

            // If we need a specific color, check it.
            if (specificColorNeeded) {
                if (wantPurple && colorSeen != SlotColorSensors.BallColor.PURPLE) continue;
                if (!wantPurple && colorSeen != SlotColorSensors.BallColor.GREEN) continue;
            }

            // Calculate rotation distance
            int d = modDist(cur, targetBucket, 3);

            // Standard "Find Nearest" logic
            if (d < bestDist || (d == bestDist && tiePrefers(targetBucket, best, cur))) {
                best = targetBucket;
                bestDist = d;
            }
        }

        return best;
    }

    public int findNearestEmptySlot(SlotColorSensors slots, Spindexer spx) {
        int currentSlot = spx.getCurrentSlot();
        int best = -1;
        int bestDist = Integer.MAX_VALUE;

        for (int i = 0; i < 3; i++) {
            int sensorIndex = (i - currentSlot + 3) % 3;

            // Check that specific sensor
            if (slots.hasAnyBall(sensorIndex)) continue;

            int d = modDist(currentSlot, i, 3);
            if (d < bestDist || (d == bestDist && tiePrefers(i, best, currentSlot))) {
                best = i;
                bestDist = d;
            }
        }

        return best;
    }


    private static int modDist(int a, int b, int mod) {
        int diff = Math.abs(b - a) % mod;
        return Math.min(diff, mod - diff);
    }

    private static boolean tiePrefers(int cand, int incumbent, int cur) {
        if (incumbent < 0) return true;
        int mod = 3;
        int dI = (incumbent - cur + mod) % mod;
        int dC = (cand - cur + mod) % mod;
        return PREFER_CLOCKWISE_ON_TIE ? (dC < dI) : (dC > dI);
    }
}