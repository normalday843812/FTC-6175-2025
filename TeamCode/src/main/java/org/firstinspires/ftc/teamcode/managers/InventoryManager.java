package org.firstinspires.ftc.teamcode.managers;

import static org.firstinspires.ftc.teamcode.config.AutoUnifiedConfig.PREFER_CLOCKWISE_ON_TIE;

import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.config.DecodeGameConfig;
import org.firstinspires.ftc.teamcode.subsystems.SpindexSlotsColor;
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

    public int getShotsTaken() {
        return shotsTaken;
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

    public int decideTargetSlot(SpindexSlotsColor slots, Spindexer spx) {
        int cur = spx.getCurrentSlot();
        int best = -1, bestDist = 999;

        if (isPatternKnown()) {
            boolean wantPurple = wantPurpleThisShot();
            SpindexSlotsColor.BallColor s0 = slots.getColor(0);
            if ((wantPurple && s0 == SpindexSlotsColor.BallColor.PURPLE) ||
                    (!wantPurple && s0 == SpindexSlotsColor.BallColor.GREEN)) {
                return 0;
            }
            for (int i = 0; i < 3; i++) {
                SpindexSlotsColor.BallColor c = slots.getColor(i);
                if (wantPurple && c != SpindexSlotsColor.BallColor.PURPLE) continue;
                if (!wantPurple && c != SpindexSlotsColor.BallColor.GREEN) continue;
                int d = modDist(cur, i, 3);
                if (d < bestDist || (d == bestDist && tiePrefers(i, best, cur))) {
                    best = i;
                    bestDist = d;
                }
            }
        } else {
            if (slots.hasAnyBall(0)) return 0;
            for (int i = 0; i < 3; i++) {
                if (!slots.hasAnyBall(i)) continue;
                int d = modDist(cur, i, 3);
                if (d < bestDist || (d == bestDist && tiePrefers(i, best, cur))) {
                    best = i;
                    bestDist = d;
                }
            }
        }
        return best;
    }

    public int findNearestEmptySlot(SpindexSlotsColor slots, Spindexer spx) {
        int cur = spx.getCurrentSlot();
        int best = -1;
        int bestDist = Integer.MAX_VALUE;

        for (int i = 0; i < 3; i++) {
            if (slots.hasAnyBall(i)) continue;

            int d = modDist(cur, i, 3);
            if (d < bestDist || (d == bestDist && tiePrefers(i, best, cur))) {
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