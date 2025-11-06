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
import static org.firstinspires.ftc.teamcode.config.ShooterYawConfig.MAX_SLEW_TICKS_PER_SEC;
import static org.firstinspires.ftc.teamcode.config.ShooterYawConfig.MAX_POSITION;
import static org.firstinspires.ftc.teamcode.config.ShooterYawConfig.MIN_POSITION;
import static org.firstinspires.ftc.teamcode.config.ShooterYawConfig.SOFT_LIMIT_MARGIN;
import static org.firstinspires.ftc.teamcode.config.ShooterYawConfig.TELEMETRY_ENABLED;
import static org.firstinspires.ftc.teamcode.config.ShooterYawConfig.TICKS_PER_DEG_INIT;
import static org.firstinspires.ftc.teamcode.config.ShooterYawConfig.YAW_POWER;

import com.pedropathing.control.FilteredPIDFController;
import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.GamepadMap;
import org.firstinspires.ftc.teamcode.config.ShooterYawConfig;
import org.firstinspires.ftc.teamcode.util.SubsystemMode;
import org.firstinspires.ftc.teamcode.util.TelemetryHelper;
import org.firstinspires.ftc.teamcode.vision.LLAprilTag;

public class ShooterYaw {

    private enum ControlState {
        MANUAL_HOLD,
        AUTO_IDLE,
        LL_TRACK,
        IMU_HOLD
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

    private final double ticksPerDeg = TICKS_PER_DEG_INIT;

    private int targetId = -1;

    private final FilteredPIDFController llPid = new FilteredPIDFController(ShooterYawConfig.PIDF_COEFFICIENTS);

    private boolean haveLock = false;
    private int lockTicks = CENTER_POS;
    private double lockRobotYawRad = 0.0;

    private long lastLoopNs = System.nanoTime();

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
    }

    public void startTeleop() {
        mode = SubsystemMode.MANUAL;
        control = ControlState.MANUAL_HOLD;
        haveLock = false;
        llPid.reset();
    }

    public void startAuto() {
        mode = SubsystemMode.AUTO;
        control = ControlState.AUTO_IDLE;
        targetId = isRed ? APRIL_TAG_RED : APRIL_TAG_BLUE;
        haveLock = false;
        lockTicks = motor.getCurrentPosition();
        lockRobotYawRad = getRobotYawRad();
        llPid.reset();
    }

    public void seekPattern(int[] ids, long stableHoldMs) {
        if (mode != SubsystemMode.AUTO) startAuto();
    }

    public boolean isPatternChosen() {
        return false;
    }

    public int getChosenPatternId() {
        return -1;
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
        LLAprilTag.YawInfo info = ll.getYawInfoForTag(resolveTargetId());
        boolean llValid = isLlValid(info);

        if (llValid) {
            double errorDeg = -info.avgDeg;

            llPid.updateError(errorDeg);
            double pidOutDeg = llPid.run();

            int desired = clampPosition(reportedTicks + (int) Math.round(-pidOutDeg * ticksPerDeg));

            desired = enforceSoftLimitVector(desired, reportedTicks);
            targetTicks = slewTowards(targetTicks, desired);
            runTo(targetTicks);

            if (Math.abs(errorDeg) <= AUTO_LOCK_DEADBAND_DEG) {
                haveLock = true;
                lockTicks = reportedTicks;
                lockRobotYawRad = getRobotYawRad();
            }

            control = ControlState.LL_TRACK;
            return;
        }

        if (IMU_HOLD_ENABLED && haveLock) {
            double robotYaw = getRobotYawRad();
            double deltaYawDeg = Math.toDegrees(lockRobotYawRad - robotYaw);
            double desiredTicksD = lockTicks + (KP_IMU * ticksPerDeg * deltaYawDeg);
            int desired = clampPosition((int) Math.round(desiredTicksD));

            desired = enforceSoftLimitVector(desired, reportedTicks);
            targetTicks = slewTowards(targetTicks, desired);
            runTo(targetTicks);

            control = ControlState.IMU_HOLD;
        } else {
            holdHere();
            control = ControlState.AUTO_IDLE;
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
    }

    private void disableAutoToManual() {
        mode = SubsystemMode.MANUAL;
        control = ControlState.MANUAL_HOLD;
        haveLock = false;
        llPid.reset();
        holdHere();
    }

    private boolean isLlValid(LLAprilTag.YawInfo info) {
        if (info == null) return false;
        if (!info.fresh) return false;
        if (info.ageMs > TTL_MS) return false;
        return Double.isFinite(info.avgDeg);
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

    public boolean isLockedOnTarget() {
        return mode == SubsystemMode.AUTO && (control == ControlState.LL_TRACK || control == ControlState.IMU_HOLD);
    }

    public void center() {
        targetTicks = CENTER_POS;
        runTo(targetTicks);
    }

    public void lockAllianceGoal() {
        mode = SubsystemMode.AUTO;
        control = ControlState.AUTO_IDLE;
        targetId = isRed ? APRIL_TAG_RED : APRIL_TAG_BLUE;
        haveLock = false;
        llPid.reset();
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

    private void addTelemetry() {
        if (!TELEMETRY_ENABLED) return;

        tele.addLine("=== SHOOTER YAW ===")
                .addData("Mode", mode::name)
                .addData("Ctrl", control::name)
                .addData("AllianceRed", "%b", isRed)
                .addData("TargetId", "%d", resolveTargetId())
                .addData("CurPos", "%d", reportedTicks)
                .addData("SetPos", "%d", targetTicks)
                .addData("HaveLock", "%b", haveLock)
                .addData("IMU Hold Enabled", "%b", IMU_HOLD_ENABLED);
    }
}