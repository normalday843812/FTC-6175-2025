package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.config.ShooterYawConfig.GOAL_BLUE_X;
import static org.firstinspires.ftc.teamcode.config.ShooterYawConfig.GOAL_BLUE_Y;
import static org.firstinspires.ftc.teamcode.config.ShooterYawConfig.GOAL_RED_X;
import static org.firstinspires.ftc.teamcode.config.ShooterYawConfig.GOAL_RED_Y;
import static org.firstinspires.ftc.teamcode.config.ShooterYawConfig.INTEGRAL_MAX;
import static org.firstinspires.ftc.teamcode.config.ShooterYawConfig.INTEGRAL_ZONE_TICKS;
import static org.firstinspires.ftc.teamcode.config.ShooterYawConfig.KD;
import static org.firstinspires.ftc.teamcode.config.ShooterYawConfig.KF;
import static org.firstinspires.ftc.teamcode.config.ShooterYawConfig.KI;
import static org.firstinspires.ftc.teamcode.config.ShooterYawConfig.KP;
import static org.firstinspires.ftc.teamcode.config.ShooterYawConfig.KS;
import static org.firstinspires.ftc.teamcode.config.ShooterYawConfig.MAX_POWER;
import static org.firstinspires.ftc.teamcode.config.ShooterYawConfig.MAX_TICKS;
import static org.firstinspires.ftc.teamcode.config.ShooterYawConfig.MIN_TICKS;
import static org.firstinspires.ftc.teamcode.config.ShooterYawConfig.TELEMETRY_ENABLED;
import static org.firstinspires.ftc.teamcode.config.ShooterYawConfig.TICKS_PER_DEG;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.util.TelemetryHelper;

public class ShooterYaw {

    private enum Mode {
        IDLE,
        TRACK_GOAL
    }

    private final DcMotorEx motor;
    private final Follower follower;
    private final TelemetryHelper tele;

    // Goal position for current alliance
    private double goalX;
    private double goalY;
    private final boolean isRed;

    private Mode mode = Mode.IDLE;

    // Tracking state
    private double targetWorldDeg = 0.0;
    private boolean targetInitialized = false;

    // PID state
    private double integral = 0.0;
    private double lastError = 0.0;
    private long lastUpdateMs = 0;

    public ShooterYaw(DcMotorEx motor, Follower follower, boolean isRed, OpMode opmode) {
        this.motor = motor;
        this.follower = follower;
        this.isRed = isRed;
        this.tele = new TelemetryHelper(opmode, TELEMETRY_ENABLED);

        // Set goal position based on alliance
        this.goalX = isRed ? GOAL_RED_X : GOAL_BLUE_X;
        this.goalY = isRed ? GOAL_RED_Y : GOAL_BLUE_Y;

        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void start() {
        targetInitialized = false;
        mode = Mode.TRACK_GOAL;
        resetPID();
    }

    public void operate() {
        long nowMs = now();
        double robotHeadingDeg = Math.toDegrees(follower.getPose().getHeading());
        double robotOmegaDegPerSec = Math.toDegrees(follower.getAngularVelocity());

        switch (mode) {
            case IDLE:
                motor.setPower(0);
                break;

            case TRACK_GOAL:
                operateTrackGoal(nowMs, robotHeadingDeg, robotOmegaDegPerSec);
                break;
        }

        lastUpdateMs = nowMs;
        addTelemetry(robotHeadingDeg);
    }

    // --- Tracking ---

    public void lockAllianceGoal() {
        goalX = isRed ? GOAL_RED_X : GOAL_BLUE_X;
        goalY = isRed ? GOAL_RED_Y : GOAL_BLUE_Y;
        mode = Mode.TRACK_GOAL;
    }

    public void setGoalPosition(double x, double y) {
        this.goalX = x;
        this.goalY = y;
        mode = Mode.TRACK_GOAL;
    }

    public boolean isLockedOnTarget() {
        return mode == Mode.TRACK_GOAL && targetInitialized;
    }

    // --- Manual Control ---

    public void center() {
        double robotHeadingDeg = Math.toDegrees(follower.getPose().getHeading());
        targetWorldDeg = robotHeadingDeg;
        targetInitialized = true;
    }

    public void adjustYaw(double deltaDeg) {
        targetWorldDeg = normalizeDeg(targetWorldDeg + deltaDeg);
        targetInitialized = true;
    }

    public void resetShooterYaw() {
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        targetInitialized = false;
        mode = Mode.IDLE;
        resetPID();
    }

    public int getCurrentTicks() {
        return motor.getCurrentPosition();
    }

    // --- Internal: Track Goal Mode ---

    private void operateTrackGoal(long nowMs, double robotHeadingDeg, double robotOmegaDegPerSec) {
        // Calculate angle from robot to goal
        Pose robotPose = follower.getPose();
        double dx = goalX - robotPose.getX();
        double dy = goalY - robotPose.getY();

        // atan2 gives us the world angle to the goal
        targetWorldDeg = Math.toDegrees(Math.atan2(dy, dx));
        targetInitialized = true;

        double desiredRelativeDeg = normalizeDeg(targetWorldDeg - robotHeadingDeg);
        int desiredTicks = clampTicks(degToTicks(desiredRelativeDeg));
        int currentTicks = motor.getCurrentPosition();

        // Feedforward for robot rotation compensation
        double feedforward = -robotOmegaDegPerSec * KF;

        double power = calculatePID(desiredTicks, currentTicks, nowMs) + feedforward;
        motor.setPower(clampPower(power));
    }

    // --- Internal: PID ---

    private double calculatePID(int targetTicks, int currentTicks, long nowMs) {
        double error = targetTicks - currentTicks;
        double dt = Math.max(0.001, (nowMs - lastUpdateMs) / 1000.0);

        double p = KP * error;

        if (Math.abs(error) < INTEGRAL_ZONE_TICKS) {
            integral += error * dt;
            integral = clamp(integral, -INTEGRAL_MAX, INTEGRAL_MAX);
        } else {
            integral = 0;
        }
        double i = KI * integral;

        double d = KD * (error - lastError) / dt;
        lastError = error;

        double output = p + i + d;

        if (KS > 0 && Math.abs(output) > 0.01) {
            output += Math.copySign(KS, output);
        }

        return output;
    }

    private void resetPID() {
        integral = 0.0;
        lastError = 0.0;
        lastUpdateMs = now();
    }

    // --- Utilities ---

    private int degToTicks(double deg) {
        return (int) Math.round(deg * TICKS_PER_DEG);
    }

    private double ticksToDeg(int ticks) {
        return ticks / TICKS_PER_DEG;
    }

    private int clampTicks(int ticks) {
        return Math.max(MIN_TICKS, Math.min(MAX_TICKS, ticks));
    }

    private double clampPower(double power) {
        return Math.max(-MAX_POWER, Math.min(MAX_POWER, power));
    }

    private static double clamp(double val, double min, double max) {
        return Math.max(min, Math.min(max, val));
    }

    private static double normalizeDeg(double deg) {
        while (deg > 180) deg -= 360;
        while (deg <= -180) deg += 360;
        return deg;
    }

    private static long now() {
        return System.nanoTime() / 1_000_000L;
    }

    private void addTelemetry(double robotHeadingDeg) {
        Pose robotPose = follower.getPose();
        int currentTicks = motor.getCurrentPosition();
        tele.addLine("=== SHOOTER YAW ===")
                .addData("Mode", mode::name)
                .addData("TargetWorld", "%.1f°", targetWorldDeg)
                .addData("RobotHeading", "%.1f°", robotHeadingDeg)
                .addData("CurrentTicks", "%d", currentTicks)
                .addData("GoalPos", "(%.0f, %.0f)", goalX, goalY)
                .addData("GOAL_BLUE_X", "%.1f", GOAL_BLUE_X)
                .addData("GOAL_BLUE_Y", "%.1f", GOAL_BLUE_Y)
                .addData("GOAL_RED_X", "%.1f", GOAL_RED_X)
                .addData("GOAL_RED_Y", "%.1f", GOAL_RED_Y)
                .addData("RobotPos", "(%.1f, %.1f)", robotPose.getX(), robotPose.getY());
    }
}