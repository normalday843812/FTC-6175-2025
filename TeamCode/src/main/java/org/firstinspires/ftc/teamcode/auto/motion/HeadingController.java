package org.firstinspires.ftc.teamcode.auto.motion;

import static com.pedropathing.math.MathFunctions.clamp;
import static org.firstinspires.ftc.teamcode.config.AutoMotionConfig.*;

public class HeadingController {
    private double prevErrDeg = 0.0;
    private long prevMs = 0;

    public double update(double desiredDeg, double currentDeg) {
        double err = wrapDeg(desiredDeg - currentDeg);

        long now = System.currentTimeMillis();
        double dt = prevMs == 0 ? 0.02 : Math.max(0.001, (now - prevMs) / 1000.0);
        double d = (err - prevErrDeg) / dt;

        prevErrDeg = err;
        prevMs = now;

        if (Math.abs(err) < HEADING_DEADBAND_DEG) {
            return 0.0;
        }

        double cmd = HEADING_KP * err + HEADING_KD * d;
        cmd /= 90.0; // Normalize so 90 deg error should map to 1.0 before the clamping

        return clamp(cmd, -HEADING_MAX_ROT, HEADING_MAX_ROT);
    }

    private static double wrapDeg(double a) {
        while (a > 180) a -= 360;
        while (a < -180) a += 360;
        return a;
    }

    public void reset() {
        prevErrDeg = 0;
        prevMs = 0;
    }
}
