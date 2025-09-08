package org.firstinspires.ftc.teamcode.misc;

import com.qualcomm.ftccommon.SoundPlayer;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;
import org.firstinspires.ftc.robotcore.external.android.AndroidSoundPool;

public final class ThickOfIt {
    private static final String TAG = "ThickOfIt";

    private final AndroidSoundPool soundPool;
    private final ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

    private final int totalSounds;
    private final long defaultDurationMs;
    private final boolean[] loaded;
    private final long[] perTrackDuration;

    private int index;
    private boolean playing;
    private boolean loop;

    /** @param totalSounds how many files like "1.mp3", "2.mp3", ... exist
     *  @param defaultDurationMs fallback duration per track, if individual durations aren’t set */
    public ThickOfIt(int totalSounds, long defaultDurationMs) {
        if (totalSounds <= 0) throw new IllegalArgumentException("totalSounds must be > 0");
        if (defaultDurationMs <= 0) throw new IllegalArgumentException("defaultDurationMs must be > 0");

        this.totalSounds = totalSounds;
        this.defaultDurationMs = defaultDurationMs;
        this.loaded = new boolean[totalSounds];
        this.perTrackDuration = new long[totalSounds];

        soundPool = new AndroidSoundPool();
        soundPool.initialize(SoundPlayer.getInstance());
        index = 0;
        playing = false;
        loop = false;
    }

    /** Preload all sounds named "1.mp3" .. "N.mp3". Returns number successfully loaded. */
    public int preloadSounds() {
        int ok = 0;
        for (int i = 0; i < totalSounds; i++) {
            final String name = buildName(i);
            boolean success = soundPool.preloadSound(name);
            loaded[i] = success;
            if (success) {
                ok++;
                RobotLog.dd(TAG, "Preloaded %s", name);
            } else {
                RobotLog.ee(TAG, "FAILED to preload %s", name);
            }
        }
        return ok;
    }

    /** Begin playback from the first track (index 0). */
    public void start() {
        startAt(0);
    }

    /** Begin playback from a 0-based index. Values are clamped into range. */
    public void startAt(int startIndex) {
        index = clamp(startIndex, 0, totalSounds - 1);
        playing = true;
        playCurrentOrAdvance();
    }

    /** Call from loop. Advances when the current track's time has elapsed. */
    public void update() {
        if (!playing) return;

        long dur = trackDurationMs(index);
        if (timer.milliseconds() >= dur) {
            advance();
        }
    }

    /** Stop the sequence and reset to the first track. */
    public void stop() {
        playing = false;
        index = 0;
    }

    /** Manually skip to the next track (respects loop). */
    public void skip() {
        if (!playing) return;
        advance();
    }

    /** Optionally loop the sequence when it reaches the end. Default: false. */
    public void setLoop(boolean loop) {
        this.loop = loop;
    }

    /** Provide a per-track duration override (1-based track index to match filenames). */
    public void setDurationMs(int trackNumberOneBased, long durationMs) {
        int i = clamp(trackNumberOneBased - 1, 0, totalSounds - 1);
        perTrackDuration[i] = Math.max(1, durationMs);
    }

    /** 1-based index to match filenames, for telemetry. */
    public int getCurrentTrackNumber() {
        return index + 1;
    }

    public boolean isPlaying() {
        return playing;
    }

    public int getTotalSounds() {
        return totalSounds;
    }

    /** Free native resources. Call this once done. */
    public void release() {
        soundPool.close();
        RobotLog.dd(TAG, "Resources released");
    }

    // --- internals ---

    private void advance() {
        index++;
        if (index >= totalSounds) {
            if (loop) {
                index = 0;
            } else {
                playing = false;
                RobotLog.dd(TAG, "Sequence complete");
                return;
            }
        }
        playCurrentOrAdvance();
    }

    /** Try to play current index; if not loaded or play fails, move forward until we either play or stop. */
    private void playCurrentOrAdvance() {
        int guard = 0; // avoid infinite loops if nothing is playable
        while (playing && guard < totalSounds) {
            final String name = buildName(index);
            if (!loaded[index]) {
                RobotLog.ee(TAG, "Skipping (not loaded): %s", name);
                index = (index + 1) % totalSounds;
                if (index == 0 && !loop) { playing = false; break; }
                guard++;
                continue;
            }

            boolean ok = soundPool.play(name);
            if (ok) {
                RobotLog.dd(TAG, "Playing %s", name);
                timer.reset();
                return;
            } else {
                RobotLog.ee(TAG, "Play failed, skipping: %s", name);
                index = (index + 1) % totalSounds;
                if (index == 0 && !loop) { playing = false; break; }
                guard++;
            }
        }
        if (guard >= totalSounds) {
            RobotLog.ee(TAG, "No playable sounds found; stopping");
            playing = false;
        }
    }

    private long trackDurationMs(int idx0) {
        long d = perTrackDuration[idx0];
        return d > 0 ? d : defaultDurationMs;
    }

    private static int clamp(int v, int lo, int hi) {
        return Math.max(lo, Math.min(hi, v));
    }

    private static String buildName(int idx0) {
        // Filenames are "1.mp3", "2.mp3", ... to match existing convention
        return (idx0 + 1) + ".mp3";
    }
}
