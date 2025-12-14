package org.firstinspires.ftc.teamcode.managers;

import com.pedropathing.geometry.Pose;

import java.util.Locale;

/**
 * Static holder for pose state that persists across OpModes.
 *
 * This allows the Pedro follower pose to carry over from auto to teleop (including when auto is stopped early).
 */
public final class PersistentPoseState {

    private static boolean initialized = false;
    private static double x = 0.0;
    private static double y = 0.0;
    private static double headingRad = 0.0;

    private PersistentPoseState() {}

    public static boolean isInitialized() {
        return initialized;
    }

    public static void reset() {
        initialized = false;
        x = 0.0;
        y = 0.0;
        headingRad = 0.0;
    }

    public static void saveFromPose(Pose pose) {
        if (pose == null) return;
        x = pose.getX();
        y = pose.getY();
        headingRad = pose.getHeading();
        initialized = true;
    }

    public static Pose loadPose() {
        return new Pose(x, y, headingRad);
    }

    public static String getPoseString() {
        return String.format(Locale.US, "(%.1f, %.1f, %.1f°)", x, y, Math.toDegrees(headingRad));
    }
}

