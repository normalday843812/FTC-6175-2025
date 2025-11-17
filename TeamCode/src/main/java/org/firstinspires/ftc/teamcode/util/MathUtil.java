package org.firstinspires.ftc.teamcode.util;

public final class MathUtil {
    private MathUtil() {
    }

    public static double deadband(double v, double d) {
        return Math.abs(v) > d ? v : 0.0;
    }
}
