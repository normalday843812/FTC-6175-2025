package org.firstinspires.ftc.teamcode.shooting;

import com.pedropathing.geometry.Pose;
import com.pedropathing.math.MathFunctions;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.util.TelemetryHelper;

public class ShooterManager {
    private final Shooter shooter;
    private boolean enabled = false;

    // TODO
    private double A0 = 1200.0;
    private double A1 = 25.0;
    private double A2 = 0.2;
    private double B1 = 30.0;
    private double B2 = 10.0;

    private double idleRpm = 600.0;
    private double maxRpm = 4500.0;

    TelemetryHelper tele;

    public ShooterManager(Shooter shooter, OpMode opmode) {
        this.shooter = shooter;
        this.tele = new TelemetryHelper(opmode, true);
    }

    public void setEnabled(boolean on) {
        if (this.enabled == on) return;
        this.enabled = on;
        if (on) {
            shooter.startAuto();
        } else {
            shooter.startTeleop();
        }
    }

    public boolean isEnabled() {
        return enabled;
    }

    public void update(Pose robot, Pose goal) {
        if (!enabled) return;
        double rpm = computeOutputRpm(robot, goal);
        shooter.setAutoRpm(rpm);

    }

    public double computeOutputRpm(Pose robot, Pose goal) {
        double dx = goal.getX() - robot.getX();
        double dy = goal.getY() - robot.getY();
        double dist = Math.hypot(dx, dy);

        double desired = Math.atan2(dy, dx);
        double theta = Math.abs(MathFunctions.getSmallestAngleDifference(robot.getHeading(), desired));

        double rpm = A0 + A1 * dist + A2 * dist * dist + B1 * theta + B2 * theta * theta;
        return clamp(rpm, idleRpm, maxRpm);
    }

    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }

    public void setCoefficients(double a0, double a1, double a2, double b1, double b2) {
        A0 = a0;
        A1 = a1;
        A2 = a2;
        B1 = b1;
        B2 = b2;
    }

    public void setLimits(double idle, double max) {
        idleRpm = idle;
        maxRpm = max;
    }

    private void addTelemetry() {
        tele.addLine("--- Shooter Manager ---")
                .addData("Enabled", "%b", isEnabled());
    }
}