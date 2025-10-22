package org.firstinspires.ftc.teamcode.util;

public final class Timer {
    private long startTimeMs;

    public Timer() {
        resetTimer();
    }

    public void resetTimer() {
        startTimeMs = System.currentTimeMillis();
    }

    public long getElapsedTime() {
        return System.currentTimeMillis() - startTimeMs;
    }

    public double getElapsedTimeSeconds() {
        return getElapsedTime() / 1000.0;
    }
}