package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.config.AutoUnifiedConfig.DIST_MAX_VALID_CM;
import static org.firstinspires.ftc.teamcode.config.AutoUnifiedConfig.DIST_SMOOTH_WINDOW;
import static org.firstinspires.ftc.teamcode.config.AutoUnifiedConfig.DIST_WALL_CLOSE_CM;
import static org.firstinspires.ftc.teamcode.config.AutoUnifiedConfig.TELEMETRY_ENABLED;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.util.TelemetryHelper;

public class DistanceSubsystem {
    private final DistanceSensor sensor;
    private final TelemetryHelper tele;
    private final double[] buf = new double[Math.max(1, DIST_SMOOTH_WINDOW)];
    private int n = 0, i = 0;

    public DistanceSubsystem(DistanceSensor sensor, OpMode opmode) {
        this.sensor = sensor;
        this.tele = new TelemetryHelper(opmode, TELEMETRY_ENABLED);
    }

    public void update() {
        double raw = sensor.getDistance(DistanceUnit.CM);
        if (Double.isFinite(raw) && raw > 0 && raw < DIST_MAX_VALID_CM) {
            buf[i] = raw;
            i = (i + 1) % buf.length;
            n = Math.min(n + 1, buf.length);
        }
        tele.addLine("--- DIST ---")
                .addData("raw_cm", "%.1f", raw)
                .addData("avg_cm", "%.1f", getAvg());
    }

    public double getAvg() {
        if (n == 0) return Double.NaN;
        double s = 0;
        for (int k = 0; k < n; k++) s += buf[k];
        return s / n;
    }

    public boolean isWallClose() {
        double d = getAvg();
        return Double.isFinite(d) && d <= DIST_WALL_CLOSE_CM;
    }
}