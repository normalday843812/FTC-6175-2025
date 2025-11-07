package org.firstinspires.ftc.teamcode.subsystems;

import static com.pedropathing.math.MathFunctions.clamp;
import static org.firstinspires.ftc.teamcode.config.AutoConfig.APRIL_TAG_BLUE;
import static org.firstinspires.ftc.teamcode.config.AutoConfig.APRIL_TAG_RED;
import static org.firstinspires.ftc.teamcode.config.AutoConfig.isRed;
import static org.firstinspires.ftc.teamcode.config.LLAprilTagConfig.TTL_MS;
import static org.firstinspires.ftc.teamcode.config.ShooterYawConfig.AUTO_LOCK_DEADBAND_DEG;
import static org.firstinspires.ftc.teamcode.config.ShooterYawConfig.CENTER_POS;
import static org.firstinspires.ftc.teamcode.config.ShooterYawConfig.CONTROL_LEVEL;
import static org.firstinspires.ftc.teamcode.config.ShooterYawConfig.IMU_HOLD_ENABLED;
import static org.firstinspires.ftc.teamcode.config.ShooterYawConfig.KP_IMU;
import static org.firstinspires.ftc.teamcode.config.ShooterYawConfig.LL_GRACE_MS;
import static org.firstinspires.ftc.teamcode.config.ShooterYawConfig.MAX_POSITION;
import static org.firstinspires.ftc.teamcode.config.ShooterYawConfig.MAX_SLEW_TICKS_PER_SEC;
import static org.firstinspires.ftc.teamcode.config.ShooterYawConfig.METERS_PER_POSE_UNIT;
import static org.firstinspires.ftc.teamcode.config.ShooterYawConfig.MIN_POSITION;
import static org.firstinspires.ftc.teamcode.config.ShooterYawConfig.MIN_SCAN_AMPLITUDE_TICKS;
import static org.firstinspires.ftc.teamcode.config.ShooterYawConfig.REACQUIRE_AFTER_MS;
import static org.firstinspires.ftc.teamcode.config.ShooterYawConfig.REACQUIRE_SCAN_ENABLED;
import static org.firstinspires.ftc.teamcode.config.ShooterYawConfig.SCAN_BAND_DEG;
import static org.firstinspires.ftc.teamcode.config.ShooterYawConfig.SCAN_PERIOD_MS;
import static org.firstinspires.ftc.teamcode.config.ShooterYawConfig.SEEK_WINDOW_MS;
import static org.firstinspires.ftc.teamcode.config.ShooterYawConfig.SMOOTHING_ALPHA;
import static org.firstinspires.ftc.teamcode.config.ShooterYawConfig.SMOOTHING_ENABLED;
import static org.firstinspires.ftc.teamcode.config.ShooterYawConfig.SOFT_LIMIT_MARGIN;
import static org.firstinspires.ftc.teamcode.config.ShooterYawConfig.STABLE_HOLD_MS_DEFAULT;
import static org.firstinspires.ftc.teamcode.config.ShooterYawConfig.TELEMETRY_ENABLED;
import static org.firstinspires.ftc.teamcode.config.ShooterYawConfig.TICKS_PER_DEG_INIT;
import static org.firstinspires.ftc.teamcode.config.ShooterYawConfig.YAW_POWER;

import com.pedropathing.control.FilteredPIDFController;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.teamcode.GamepadMap;
import org.firstinspires.ftc.teamcode.config.ShooterYawConfig;
import org.firstinspires.ftc.teamcode.util.SubsystemMode;
import org.firstinspires.ftc.teamcode.util.TelemetryHelper;
import org.firstinspires.ftc.teamcode.util.Timer;
import org.firstinspires.ftc.teamcode.vision.LLAprilTag;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagMetadata;

public class ShooterYaw {

    private enum ControlState {
        MANUAL_HOLD,
        AUTO_IDLE,
        LL_TRACK,
        LL_STALE_HOLD,
        IMU_HOLD,
        SEEK_PATTERN
    }

    private final DcMotorEx motor;
    private final GamepadMap map;
    private final TelemetryHelper tele;
    private final LLAprilTag ll;
    private final Follower follower;

    private SubsystemMode mode = SubsystemMode.MANUAL;
    private ControlState control = ControlState.MANUAL_HOLD;

    private int targetTicks = CENTER_POS;
    private int reportedTicks = 0;
    private int lastDesiredTicks = CENTER_POS;
    private double lastLlErrorDeg = 0.0;

    private final double ticksPerDeg = TICKS_PER_DEG_INIT;

    private int targetId = -1;

    private final FilteredPIDFController llPid = new FilteredPIDFController(ShooterYawConfig.PIDF_COEFFICIENTS);

    private boolean haveLock = false;
    private int lockTicks = CENTER_POS;
    private double lockRobotYawRad = 0.0;

    private double smoothSetpoint = CENTER_POS;

    private long lastLoopNs = System.nanoTime();
    private long lastLLFreshMs = 0L;

    private int[] patternIds = null;
    private int patternIdx = 0;
    private int chosenPatternId = -1;
    private long stableHoldMsGoal = STABLE_HOLD_MS_DEFAULT;
    private final Timer stableTimer = new Timer();
    private long sweepStartMs = System.currentTimeMillis();

    public ShooterYaw(DcMotorEx shooterYawMotor,
                      LLAprilTag aprilTag,
                      GamepadMap map,
                      Follower follower,
                      OpMode opmode) {

        this.motor = shooterYawMotor;
        this.ll = aprilTag;
        this.map = map;
        this.follower = follower;
        this.tele = new TelemetryHelper(opmode, TELEMETRY_ENABLED);

        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setTargetPosition(targetTicks);
        motor.setPower(Math.abs(YAW_POWER));

        llPid.reset();
        smoothSetpoint = targetTicks;
    }

    public void startTeleop() {
        mode = SubsystemMode.MANUAL;
        control = ControlState.MANUAL_HOLD;
        haveLock = false;
        llPid.reset();
        smoothSetpoint = motor.getCurrentPosition();
    }

    public void startAuto() {
        mode = SubsystemMode.AUTO;
        control = ControlState.AUTO_IDLE;
        targetId = isRed ? APRIL_TAG_RED : APRIL_TAG_BLUE;
        haveLock = false;
        lockTicks = motor.getCurrentPosition();
        lockRobotYawRad = getRobotYawRad();
        llPid.reset();
        smoothSetpoint = lockTicks;
        lastLLFreshMs = 0L;
    }

    public void seekPattern(int[] ids, long stableHoldMs) {
        patternIds = (ids == null || ids.length == 0) ? null : ids.clone();
        patternIdx = 0;
        chosenPatternId = -1;
        stableHoldMsGoal = stableHoldMs <= 0 ? STABLE_HOLD_MS_DEFAULT : stableHoldMs;

        if (mode != SubsystemMode.AUTO) startAuto();
        control = ControlState.SEEK_PATTERN;
        sweepStartMs = System.currentTimeMillis();
        stableTimer.resetTimer();
    }

    public boolean isPatternChosen() {
        return chosenPatternId >= 0;
    }

    public int getChosenPatternId() {
        return chosenPatternId;
    }

    public boolean isLockedOnTarget() {
        return mode == SubsystemMode.AUTO &&
                (control == ControlState.LL_TRACK || control == ControlState.IMU_HOLD);
    }

    public void center() {
        targetTicks = CENTER_POS;
        runTo(targetTicks);
        smoothSetpoint = targetTicks;
    }

    public void lockAllianceGoal() {
        mode = SubsystemMode.AUTO;
        control = ControlState.AUTO_IDLE;
        targetId = isRed ? APRIL_TAG_RED : APRIL_TAG_BLUE;
        haveLock = false;
        llPid.reset();
        smoothSetpoint = motor.getCurrentPosition();
    }

    public void resetShooterYaw() {
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        targetTicks = CENTER_POS;
        reportedTicks = 0;
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setTargetPosition(targetTicks);
        motor.setPower(Math.abs(YAW_POWER));
        haveLock = false;
        llPid.reset();
        smoothSetpoint = targetTicks;
    }

    public void operate() {
        if (map != null && mode == SubsystemMode.MANUAL && map.shooterYawAutoLockToggle) {
            enableAutoLock();
        } else if (map != null && mode == SubsystemMode.AUTO && map.shooterYawAutoLockToggle) {
            disableAutoToManual();
        }
        if (map != null && map.resetShooterYaw) {
            resetShooterYaw();
            return;
        }

        reportedTicks = motor.getCurrentPosition();

        if (mode == SubsystemMode.MANUAL) {
            operateManual();
        } else {
            operateAuto();
        }

        addTelemetry();
    }

    private void operateManual() {
        if (map != null) {
            if ((map.shooterYaw < 0 && targetTicks > MIN_POSITION) ||
                    (map.shooterYaw > 0 && targetTicks < MAX_POSITION)) {
                targetTicks += (int) (map.shooterYaw * CONTROL_LEVEL);
            }
        }
        targetTicks = clampPosition(targetTicks);
        runTo(targetTicks);
        control = ControlState.MANUAL_HOLD;
    }

    private void operateAuto() {
        long now = System.currentTimeMillis();
        LLAprilTag.YawInfo info = ll.getYawInfoForTag(resolveTargetId());
        boolean llValid = isLlValid(info);

        if (llValid) {
            double errorDeg = -info.avgDeg;
            lastLlErrorDeg = errorDeg;

            llPid.updateError(errorDeg);
            double pidOutDeg = llPid.run();

            int desired = clampPosition(reportedTicks + (int) Math.round(-pidOutDeg * ticksPerDeg));

            desired = enforceSoftLimitVector(desired, reportedTicks);

            desired = applySmoothing(desired);
            lastDesiredTicks = desired;

            targetTicks = slewTowards(targetTicks, desired);
            runTo(targetTicks);

            if (Math.abs(errorDeg) <= AUTO_LOCK_DEADBAND_DEG) {
                haveLock = true;
                lockTicks = reportedTicks;
                lockRobotYawRad = getRobotYawRad();
            }

            lastLLFreshMs = now;
            control = ControlState.LL_TRACK;
            return;
        }

        if (now - lastLLFreshMs <= LL_GRACE_MS) {
            control = ControlState.LL_STALE_HOLD;
            runTo(targetTicks);
            return;
        }

        if (control == ControlState.SEEK_PATTERN) {
            seekPatternStep(now);
            return;
        }

        if (IMU_HOLD_ENABLED && haveLock) {
            double robotYaw = getRobotYawRad();
            double deltaYawDeg = Math.toDegrees(lockRobotYawRad - robotYaw);
            int center = clampPosition(lockTicks + (int) Math.round(KP_IMU * ticksPerDeg * deltaYawDeg));

            int desired = center;

            if (REACQUIRE_SCAN_ENABLED && (now - lastLLFreshMs) >= REACQUIRE_AFTER_MS) {
                desired = addSafeScan(center, now);
            }

            desired = enforceSoftLimitVector(desired, reportedTicks);
            desired = applySmoothing(desired);
            lastDesiredTicks = desired;

            targetTicks = slewTowards(targetTicks, desired);
            runTo(targetTicks);

            control = ControlState.IMU_HOLD;
        } else {
            holdHere();
            control = ControlState.AUTO_IDLE;
        }
    }

    private void seekPatternStep(long now) {
        if (patternIds == null || patternIds.length == 0) {
            control = ControlState.AUTO_IDLE;
            return;
        }

        int id = patternIds[patternIdx];

        Pose pose = follower != null ? follower.getPose() : new Pose();
        double headingDeg = Math.toDegrees(pose.getHeading());
        double bearingDeg = getBearingDeg(id, pose);
        double baseErrDeg = normalizeDeg(bearingDeg - headingDeg);

        int center = clampPosition(reportedTicks - (int) Math.round(baseErrDeg * ticksPerDeg));

        int desired = addSafeScan(center, now);

        desired = enforceSoftLimitVector(desired, reportedTicks);
        desired = applySmoothing(desired);
        lastDesiredTicks = desired;

        targetTicks = slewTowards(targetTicks, desired);
        runTo(targetTicks);

        LLAprilTag.YawInfo info = ll.getYawInfoForTag(id);
        if (isLlValid(info)) {
            if (stableTimer.getElapsedTime() >= stableHoldMsGoal) {
                chosenPatternId = id;
                targetId = id;
                haveLock = true;
                lockTicks = reportedTicks;
                lockRobotYawRad = getRobotYawRad();
                control = ControlState.LL_TRACK; // LL valid now
                return;
            }
        } else {
            stableTimer.resetTimer();
        }

        if (now - sweepStartMs > SEEK_WINDOW_MS) {
            patternIdx = (patternIdx + 1) % patternIds.length;
            sweepStartMs = now;
        }
    }

    private void enableAutoLock() {
        mode = SubsystemMode.AUTO;
        control = ControlState.AUTO_IDLE;
        targetId = isRed ? APRIL_TAG_RED : APRIL_TAG_BLUE;
        haveLock = false;
        lockTicks = motor.getCurrentPosition();
        lockRobotYawRad = getRobotYawRad();
        llPid.reset();
        smoothSetpoint = lockTicks;
        lastLLFreshMs = 0L;
    }

    private void disableAutoToManual() {
        mode = SubsystemMode.MANUAL;
        control = ControlState.MANUAL_HOLD;
        haveLock = false;
        llPid.reset();
        smoothSetpoint = motor.getCurrentPosition();
        holdHere();
    }

    private boolean isLlValid(LLAprilTag.YawInfo info) {
        if (info == null) return false;
        if (!info.fresh) return false;
        if (info.ageMs > TTL_MS) return false;
        return Double.isFinite(info.avgDeg);
    }

    private int applySmoothing(int desired) {
        if (!SMOOTHING_ENABLED) return desired;
        smoothSetpoint = smoothSetpoint + SMOOTHING_ALPHA * ((double) desired - smoothSetpoint);
        return clampPosition((int) Math.round(smoothSetpoint));
    }

    private int addSafeScan(int centerTicks, long nowMs) {
        int ampTicks = (int) Math.round(ticksPerDeg * SCAN_BAND_DEG);

        int leftRoom = (MAX_POSITION - SOFT_LIMIT_MARGIN) - centerTicks;
        int rightRoom = centerTicks - (MIN_POSITION + SOFT_LIMIT_MARGIN);
        int maxAmp = Math.max(0, Math.min(Math.min(leftRoom, rightRoom), ampTicks));

        if (maxAmp < MIN_SCAN_AMPLITUDE_TICKS) return centerTicks;

        double phase = twoPi(nowMs - sweepStartMs) / SCAN_PERIOD_MS;
        int offset = (int) Math.round(maxAmp * Math.sin(phase));
        return clampPosition(centerTicks + offset);
    }

    private int slewTowards(int current, int desired) {
        long now = System.nanoTime();
        double dt = Math.max(1e-3, (now - lastLoopNs) / 1e9);
        lastLoopNs = now;

        int maxStep = (int) Math.max(1, Math.round(MAX_SLEW_TICKS_PER_SEC * dt));
        int delta = desired - current;
        if (Math.abs(delta) <= maxStep) return desired;
        return current + (delta > 0 ? maxStep : -maxStep);
    }

    private int enforceSoftLimitVector(int desired, int current) {
        if ((desired >= MAX_POSITION - SOFT_LIMIT_MARGIN && desired > current) ||
                (desired <= MIN_POSITION + SOFT_LIMIT_MARGIN && desired < current)) {
            return desired > current ? MAX_POSITION : MIN_POSITION;
        }
        return desired;
    }

    private void runTo(int ticks) {
        ticks = clampPosition(ticks);
        if (motor.getMode() != DcMotor.RunMode.RUN_TO_POSITION) {
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        motor.setTargetPosition(ticks);
        motor.setPower(Math.abs(YAW_POWER));
        targetTicks = ticks;
    }

    private void holdHere() {
        runTo(reportedTicks);
    }

    private int clampPosition(int pos) {
        return (int) clamp(pos, MIN_POSITION, MAX_POSITION);
    }

    private int resolveTargetId() {
        if (targetId > 0) return targetId;
        return isRed ? APRIL_TAG_RED : APRIL_TAG_BLUE;
    }

    private double getRobotYawRad() {
        try {
            if (follower != null && follower.getPose() != null) {
                return follower.getPose().getHeading();
            }
        } catch (Throwable ignored) {
        }
        return 0.0;
    }

    private static double normalizeDeg(double deg) {
        double x = deg % 360.0;
        if (x > 180.0) x -= 360.0;
        if (x <= -180.0) x += 360.0;
        return x;
    }

    private static double twoPi(long ms) {
        return 2.0 * Math.PI * (ms / 1000.0);
    }

    private static double getBearingDeg(int tagId, Pose robotPose) {
        AprilTagMetadata md = AprilTagGameDatabase.getCurrentGameTagLibrary().lookupTag(tagId);
        if (md == null) return 0.0;
        VectorF p = md.fieldPosition;
        double tagXm = p.get(0);
        double tagYm = p.get(1);
        double rxm = robotPose.getX() * METERS_PER_POSE_UNIT;
        double rym = robotPose.getY() * METERS_PER_POSE_UNIT;
        double dx = tagXm - rxm;
        double dy = tagYm - rym;
        return Math.toDegrees(Math.atan2(dy, dx));
    }

    private void addTelemetry() {
        if (!TELEMETRY_ENABLED) return;

        long now = System.currentTimeMillis();
        tele.addLine("=== SHOOTER YAW ===")
                .addData("Mode", mode::name)
                .addData("Ctrl", control::name)
                .addData("AllianceRed", "%b", isRed)
                .addData("TargetId", "%d", resolveTargetId())
                .addData("CurPos", "%d", reportedTicks)
                .addData("RawDesired", "%d", lastDesiredTicks)
                .addData("CmdPos", "%d", targetTicks)
                .addData("HaveLock", "%b", haveLock)
                .addData("LL Error (deg)", "%.2f", lastLlErrorDeg)
                .addData("IMU Hold Enabled", "%b", IMU_HOLD_ENABLED)
                .addData("LL Age(ms)", "%d", now - lastLLFreshMs)
                .addData("Smoothing", "%b", SMOOTHING_ENABLED)
                .addData("Slew(t/s)", "%.0f", MAX_SLEW_TICKS_PER_SEC);

        double robotYawDeg = Math.toDegrees(getRobotYawRad());
        double lockYawDeg = Math.toDegrees(lockRobotYawRad);
        tele.addData("RobotYawDeg", "%.1f", robotYawDeg)
                .addData("LockYawDeg", "%.1f", lockYawDeg)
                .addData("LockTicks", "%d", lockTicks);

        if (control == ControlState.SEEK_PATTERN) {
            tele.addData("SeekIdx", "%d", patternIdx)
                    .addData("ChosenId", "%d", chosenPatternId)
                    .addData("SeekElapsed(ms)", "%d", now - sweepStartMs);
        }
    }
}