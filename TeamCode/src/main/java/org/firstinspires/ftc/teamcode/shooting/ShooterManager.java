package org.firstinspires.ftc.teamcode.shooting;

import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.teamcode.config.TeleOpShooterConfig;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.util.TelemetryHelper;

public class ShooterManager {
    private final Shooter shooter;
    private final RpmModel rpmModel;
    private boolean enabled = false;

    private double idleRpm = 600.0;
    private double maxRpm = 4500.0;

    private final TelemetryHelper tele;

    public ShooterManager(Shooter shooter, RpmModel model, com.qualcomm.robotcore.eventloop.opmode.OpMode opmode) {
        this.shooter = shooter;
        this.rpmModel = model;
        this.tele = new TelemetryHelper(opmode, true);
    }

    public void setEnabled(boolean on) {
        if (this.enabled == on) return;
        this.enabled = on;
        if (on) {
            shooter.startAuto();
        } else {
            shooter.startTeleop();
            shooter.setAutoRpm(idleRpm);
        }
    }

    public boolean isEnabled() {
        return enabled;
    }

    public void setLimits(double idle, double max) {
        this.idleRpm = idle;
        this.maxRpm = max;
        shooter.setIdleRpm(idle);
    }

    public void update(Pose robot, Pose goal, Limelight3A limelight) {
        if (!enabled) {
            shooter.setAutoRpm(idleRpm);
            addTelemetry(robot, goal, true, false, false, idleRpm);
            return;
        }

        boolean abortToIdle = shouldIdleNow(robot, limelight);
        if (abortToIdle) {
            shooter.setAutoRpm(idleRpm);
            addTelemetry(robot, goal, true, true, false, idleRpm);
            return;
        }

        double rpm;
        if (rpmModel != null && rpmModel.isValid()) {
            rpm = rpmModel.computeTargetRpm(robot, goal);
        } else {
            rpm = idleRpm; // fallback if model not set
        }

        double clamped = Math.max(idleRpm, Math.min(maxRpm, rpm));
        shooter.setAutoRpm(clamped);
        addTelemetry(robot, goal, false, false, rpmModel != null && rpmModel.isValid(), clamped);
    }

    private boolean shouldIdleNow(Pose robotPose, Limelight3A ll) {
        double headingDeg = Math.toDegrees(robotPose.getHeading());
        double delta = normalizeAngleDeg(headingDeg - TeleOpShooterConfig.IDLE_HEADING_DEG);
        boolean inNoGoWindow = Math.abs(delta) <= TeleOpShooterConfig.IDLE_HEADING_TOLERANCE_DEG;

        boolean noTag = true;
        try {
            LLResult r = ll.getLatestResult();
            noTag = (r == null || !r.isValid() || r.getFiducialResults() == null || r.getFiducialResults().isEmpty());
        } catch (Throwable ignored) {
        }
        return inNoGoWindow && noTag;
    }

    private static double normalizeAngleDeg(double a) {
        while (a > 180) a -= 360;
        while (a < -180) a += 360;
        return a;
    }

    private void addTelemetry(Pose robot, Pose goal, boolean idling, boolean policyIdle, boolean modelValid, double targetRpm) {
        tele.addLine("--- ShooterManager ---")
                .addData("Enabled", "%b", enabled)
                .addData("PolicyIdle", "%b", policyIdle)
                .addData("ModelValid", "%b", modelValid)
                .addData("TargetRPM", "%.0f", targetRpm)
                .addData("RobotPose", "(%.1f, %.1f, %.1f°)",
                        robot.getX(), robot.getY(), Math.toDegrees(robot.getHeading()))
                .addData("GoalPose", "(%.1f, %.1f, %.1f°)",
                        goal.getX(), goal.getY(), Math.toDegrees(goal.getHeading()));
    }
}