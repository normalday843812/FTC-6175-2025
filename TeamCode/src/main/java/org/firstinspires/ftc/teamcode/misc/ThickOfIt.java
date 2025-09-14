package org.firstinspires.ftc.teamcode.misc;

import org.firstinspires.ftc.robotcore.external.android.AndroidSoundPool;
import com.qualcomm.ftccommon.SoundPlayer;
import com.qualcomm.robotcore.util.ElapsedTime;

public class ThickOfIt {
    private final AndroidSoundPool androidSoundPool;
    private int currentSoundIndex;
    private static final int TOTAL_SOUNDS = 33;
    private static final double SOUND_DURATION_MS = 5000;
    private final ElapsedTime soundTimer;
    private boolean isPlaying;

    public ThickOfIt() {
        androidSoundPool = new AndroidSoundPool();
        androidSoundPool.initialize(SoundPlayer.getInstance());
        currentSoundIndex = 1;
        soundTimer = new ElapsedTime();
        isPlaying = false;
    }

    public void preloadSounds() {
        for (int i = 1; i <= TOTAL_SOUNDS; i++) {
            String soundFileName = i + ".mp3";
            boolean success = androidSoundPool.preloadSound(soundFileName);

            if (!success) {
                System.out.println("Failed to preload sound: " + soundFileName);
            } else {
                System.out.println("Sound preloaded successfully: " + soundFileName);
            }
        }
    }

    public void startSoundSequence() {
        if (!isPlaying) {
            isPlaying = true;
            currentSoundIndex = 1;
            playCurrentSound();
            soundTimer.reset();
        }
    }

    public void update() {
        if (isPlaying && soundTimer.milliseconds() >= SOUND_DURATION_MS) {
            currentSoundIndex++;
            if (currentSoundIndex <= TOTAL_SOUNDS) {
                playCurrentSound();
                soundTimer.reset();
            } else {
                isPlaying = false;
                currentSoundIndex = 1;
            }
        }
    }

    private void playCurrentSound() {
        String soundFileName = currentSoundIndex + ".mp3";
        boolean success = androidSoundPool.play(soundFileName);

        if (success) {
            System.out.println("Playing sound: " + soundFileName);
        } else {
            System.out.println("Failed to play sound: " + soundFileName);
            isPlaying = false;
        }
    }

    public void stopSequence() {
        isPlaying = false;
        currentSoundIndex = 1;
    }
    public int getCurrentIndex() {
        return currentSoundIndex;
    }
    public boolean isPlaying() {
        return isPlaying;
    }

    public void release() {
        androidSoundPool.close();
        System.out.println("Resources released.");
    }
}