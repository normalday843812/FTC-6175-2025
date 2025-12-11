package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.config.AutoConfig.APRIL_TAG_BLUE;
import static org.firstinspires.ftc.teamcode.config.AutoConfig.APRIL_TAG_RED;
import static org.firstinspires.ftc.teamcode.config.AutoConfig.isRed;
import static org.firstinspires.ftc.teamcode.config.ShooterYawConfig.CENTER_TICKS;
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
import static org.firstinspires.ftc.teamcode.config.ShooterYawConfig.SCAN_AMPLITUDE_DEG;
import static org.firstinspires.ftc.teamcode.config.ShooterYawConfig.SCAN_PERIOD_MS;
import static org.firstinspires.ftc.teamcode.config.ShooterYawConfig.SEEK_TIMEOUT_MS;
import static org.firstinspires.ftc.teamcode.config.ShooterYawConfig.TAG_STALE_MS;
import static org.firstinspires.ftc.teamcode.config.ShooterYawConfig.TELEMETRY_ENABLED;
import static org.firstinspires.ftc.teamcode.config.ShooterYawConfig.TICKS_PER_DEG;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.util.TelemetryHelper;
import org.firstinspires.ftc.teamcode.util.Timer;
import org.firstinspires.ftc.teamcode.vision.LLAprilTag;

public class ShooterYaw {

    private enum Mode {
        IDLE,
        TRACK_TAG,
        SEEK_PATTERN
    }

    private final DcMotorEx motor;
    private final LLAprilTag limelight;
    private final Follower follower;
    private final TelemetryHelper tele;

    private Mode mode = Mode.IDLE;

    // Tracking state
    private double targetWorldDeg = 0.0;
    private boolean targetInitialized = false;
    private int targetId = -1;
    private long lastTagSeenMs = 0;

    // PID state
    private double integral = 0.0;
    private double lastError = 0.0;
    private long lastUpdateMs = 0;

    // Pattern seeking state
    private int[] patternIds = null;
    private int patternIndex = 0;
    private int chosenPatternId = -1;
    private long stableStartMs = 0;
    private long stableRequiredMs = 0;
    private long seekStartMs = 0;
    private final Timer seekTimer = new Timer();

    public ShooterYaw(DcMotorEx motor, LLAprilTag limelight, Follower follower, OpMode opmode) {
        this.motor = motor;
        this.limelight = limelight;
        this.follower = follower;
        this.tele = new TelemetryHelper(opmode, TELEMETRY_ENABLED);

        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void start() {
        targetId = isRed ? APRIL_TAG_RED : APRIL_TAG_BLUE;
        targetInitialized = false;
        mode = Mode.TRACK_TAG;
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

            case TRACK_TAG:
                operateTrackTag(nowMs, robotHeadingDeg, robotOmegaDegPerSec);
                break;

            case SEEK_PATTERN:
                operateSeekPattern(nowMs, robotHeadingDeg, robotOmegaDegPerSec);
                break;
        }

        lastUpdateMs = nowMs;
        addTelemetry(robotHeadingDeg);
    }

    // --- Pattern Seeking ---

    public void seekPattern(int[] ids, long stableHoldMs) {
        if (ids == null || ids.length == 0) {
            mode = Mode.TRACK_TAG;
            return;
        }

        patternIds = ids.clone();
        patternIndex = 0;
        chosenPatternId = -1;
        stableRequiredMs = stableHoldMs;
        stableStartMs = 0;
        seekStartMs = now();
        seekTimer.resetTimer();
        mode = Mode.SEEK_PATTERN;
    }

    public boolean isPatternChosen() {
        return chosenPatternId >= 0;
    }

    public int getChosenPatternId() {
        return chosenPatternId;
    }

    // --- Tracking ---

    public void lockAllianceGoal() {
        targetId = isRed ? APRIL_TAG_RED : APRIL_TAG_BLUE;
        mode = Mode.TRACK_TAG;
    }

    public void setTargetTag(int tagId) {
        this.targetId = tagId;
        mode = Mode.TRACK_TAG;
    }

    public boolean isLockedOnTarget() {
        if (mode != Mode.TRACK_TAG || !targetInitialized) return false;
        return (now() - lastTagSeenMs) < TAG_STALE_MS;
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
        chosenPatternId = -1;
        mode = Mode.IDLE;
        resetPID();
    }

    public int getCurrentTicks() {
        return motor.getCurrentPosition();
    }

    // --- Internal: Track Tag Mode ---

    private void operateTrackTag(long nowMs, double robotHeadingDeg, double robotOmegaDegPerSec) {
        updateTargetFromVision(nowMs, robotHeadingDeg, targetId);

        if (!targetInitialized) {
            targetWorldDeg = robotHeadingDeg + ticksToDeg(motor.getCurrentPosition());
            targetInitialized = true;
        }

        double desiredRelativeDeg = normalizeDeg(targetWorldDeg - robotHeadingDeg);
        int desiredTicks = clampTicks(degToTicks(desiredRelativeDeg));
        int currentTicks = motor.getCurrentPosition();

        boolean tagFresh = (nowMs - lastTagSeenMs) < TAG_STALE_MS;
        double feedforward = tagFresh ? 0.0 : -robotOmegaDegPerSec * KF;

        double power = calculatePID(desiredTicks, currentTicks, nowMs) + feedforward;
        motor.setPower(clampPower(power));
    }

    // --- Internal: Seek Pattern Mode ---

    private void operateSeekPattern(long nowMs, double robotHeadingDeg, double robotOmegaDegPerSec) {
        int currentTagId = patternIds[patternIndex];

        // Scan back and forth
        double scanOffset = SCAN_AMPLITUDE_DEG * Math.sin(2.0 * Math.PI * (nowMs - seekStartMs) / SCAN_PERIOD_MS);
        double baseTargetDeg = robotHeadingDeg + scanOffset;

        int desiredTicks = clampTicks(degToTicks(normalizeDeg(baseTargetDeg - robotHeadingDeg) + scanOffset));
        int currentTicks = motor.getCurrentPosition();

        double power = calculatePID(desiredTicks, currentTicks, nowMs);
        motor.setPower(clampPower(power));

        // Check if current tag is visible
        LLAprilTag.YawInfo info = limelight.getYawInfoForTag(currentTagId);
        boolean tagVisible = info != null && info.fresh && !Double.isNaN(info.rawDeg);

        if (tagVisible) {
            if (stableStartMs == 0) {
                stableStartMs = nowMs;
            } else if (nowMs - stableStartMs >= stableRequiredMs) {
                // Tag stable long enough - choose it
                chosenPatternId = currentTagId;
                targetId = currentTagId;
                mode = Mode.TRACK_TAG;

                // Initialize tracking from current tag position
                double currentWorldDeg = robotHeadingDeg + ticksToDeg(currentTicks);
                targetWorldDeg = normalizeDeg(currentWorldDeg + info.rawDeg);
                targetInitialized = true;
                lastTagSeenMs = nowMs;
                return;
            }
        } else {
            stableStartMs = 0;
        }

        // Timeout - try next tag
        if (nowMs - seekStartMs > SEEK_TIMEOUT_MS) {
            patternIndex = (patternIndex + 1) % patternIds.length;
            seekStartMs = nowMs;
            stableStartMs = 0;
        }
    }

    // --- Internal: Vision Update ---

    private void updateTargetFromVision(long nowMs, double robotHeadingDeg, int tagId) {
        if (limelight == null || tagId < 0) return;

        LLAprilTag.YawInfo info = limelight.getYawInfoForTag(tagId);

        if (info != null && info.fresh && !Double.isNaN(info.rawDeg)) {
            double currentWorldDeg = robotHeadingDeg + ticksToDeg(motor.getCurrentPosition());
            targetWorldDeg = normalizeDeg(currentWorldDeg + info.rawDeg);
            lastTagSeenMs = nowMs;
        }
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
        int currentTicks = motor.getCurrentPosition();
        tele.addLine("=== SHOOTER YAW ===")
                .addData("Mode", mode::name)
                .addData("TargetWorld", "%.1f°", targetWorldDeg)
                .addData("RobotHeading", "%.1f°", robotHeadingDeg)
                .addData("CurrentTicks", "%d", currentTicks)
                .addData("TagLocked", "%b", isLockedOnTarget())
                .addData("TargetId", "%d", targetId);

        if (mode == Mode.SEEK_PATTERN) {
            tele.addData("SeekIndex", "%d", patternIndex)
                    .addData("ChosenId", "%d", chosenPatternId)
                    .addData("StableMs", "%d", stableStartMs > 0 ? now() - stableStartMs : 0);
        }
    }
}