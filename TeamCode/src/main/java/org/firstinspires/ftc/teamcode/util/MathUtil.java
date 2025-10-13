package org.firstinspires.ftc.teamcode.util;

public final class MathUtil {
    private MathUtil() {
    }

    public static double deadband(double v, double d) {
        return Math.abs(v) > d ? v : 0.0;
    }

    public static double wrapRad(double a) {
        while (a > Math.PI) a -= 2 * Math.PI;
        while (a < -Math.PI) a += 2 * Math.PI;
        return a;
    }
}
