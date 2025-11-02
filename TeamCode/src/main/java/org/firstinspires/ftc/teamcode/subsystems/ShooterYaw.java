package org.firstinspires.ftc.teamcode.subsystems;

import static com.pedropathing.math.MathFunctions.clamp;
import static org.firstinspires.ftc.teamcode.config.AutoConfig.APRIL_TAG_BLUE;
import static org.firstinspires.ftc.teamcode.config.AutoConfig.APRIL_TAG_RED;
import static org.firstinspires.ftc.teamcode.config.ShooterYawConfig.AUTO_LOCK_DEADBAND_DEG;
import static org.firstinspires.ftc.teamcode.config.ShooterYawConfig.AUTO_LOCK_ENABLED;
import static org.firstinspires.ftc.teamcode.config.ShooterYawConfig.AUTO_LOCK_KD;
import static org.firstinspires.ftc.teamcode.config.ShooterYawConfig.AUTO_LOCK_KI;
import static org.firstinspires.ftc.teamcode.config.ShooterYawConfig.AUTO_LOCK_KP;
import static org.firstinspires.ftc.teamcode.config.ShooterYawConfig.AUTO_LOCK_MAX_POWER;
import static org.firstinspires.ftc.teamcode.config.ShooterYawConfig.AUTO_LOCK_T;
import static org.firstinspires.ftc.teamcode.config.ShooterYawConfig.CENTER_POS;
import static org.firstinspires.ftc.teamcode.config.ShooterYawConfig.CONTROL_LEVEL;
import static org.firstinspires.ftc.teamcode.config.ShooterYawConfig.MAX_POSITION;
import static org.firstinspires.ftc.teamcode.config.ShooterYawConfig.MIN_POSITION;
import static org.firstinspires.ftc.teamcode.config.ShooterYawConfig.SOFT_LIMIT_MARGIN;
import static org.firstinspires.ftc.teamcode.config.ShooterYawConfig.TELEMETRY_ENABLED;
import static org.firstinspires.ftc.teamcode.config.ShooterYawConfig.YAW_POWER;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import com.pedropathing.control.FilteredPIDFController;
import com.pedropathing.control.FilteredPIDFCoefficients;

import org.firstinspires.ftc.teamcode.GamepadMap;
import org.firstinspires.ftc.teamcode.util.SubsystemMode;
import org.firstinspires.ftc.teamcode.util.TelemetryHelper;
import org.firstinspires.ftc.teamcode.vision.LLAprilTag;

public class ShooterYaw {
    private enum ControlMode {MANUAL, AUTO_LOCK}

    private final DcMotorEx shooterYawMotor;
    private final GamepadMap map;
    private final TelemetryHelper tele;
    private final LLAprilTag ll;
    private final boolean allianceRed;

    private SubsystemMode mode = SubsystemMode.MANUAL;
    private ControlMode control = ControlMode.MANUAL;

    private int targetPosition = 0;
    private int position = 0;

    private boolean autoLockEnabled = false;

    private double yawRawDeg = Double.NaN;
    private double yawAvgDeg = Double.NaN;
    private boolean tagFresh = false;

    private final FilteredPIDFController yawPid =
            new FilteredPIDFController(
                    new FilteredPIDFCoefficients(
                            AUTO_LOCK_KP, AUTO_LOCK_KI, AUTO_LOCK_KD,
                            AUTO_LOCK_T,
                            0.0
                    )
            );

    // Diagnostics
    private boolean wasLocked = false;
    private long lockStartMs = 0L;
    private int stableFrames = 0;

    public ShooterYaw(DcMotorEx shooterYawMotor, GamepadMap map, OpMode opmode) {
        this(shooterYawMotor, null, false, map, opmode);
    }

    public ShooterYaw(DcMotorEx shooterYawMotor,
                      LLAprilTag aprilTag,
                      boolean allianceRed,
                      GamepadMap map,
                      OpMode opmode) {
        this.shooterYawMotor = shooterYawMotor;
        this.ll = aprilTag;
        this.allianceRed = allianceRed;
        this.map = map;
        this.tele = new TelemetryHelper(opmode, TELEMETRY_ENABLED);
    }

    public void startTeleop() {
        mode = SubsystemMode.MANUAL;
        control = ControlMode.MANUAL;
        autoLockEnabled = false;
        resetState();
    }

    public void startAuto() {
        mode = SubsystemMode.AUTO;
        control = ControlMode.MANUAL;
        autoLockEnabled = false;
        resetState();
    }

    public void setAutoLock(boolean enabled) {
        autoLockEnabled = enabled;
        control = enabled ? ControlMode.AUTO_LOCK : ControlMode.MANUAL;
        resetState();
    }

    public boolean isLockedOnTarget() {
        if (!(autoLockEnabled && control == ControlMode.AUTO_LOCK)) return false;
        if (!tagFresh || Double.isNaN(yawAvgDeg)) return false;
        return Math.abs(yawAvgDeg) < AUTO_LOCK_DEADBAND_DEG;
    }

    public void center() {
        targetPosition = clampPosition(CENTER_POS);
        if (shooterYawMotor.getMode() != DcMotor.RunMode.RUN_TO_POSITION) {
            shooterYawMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        shooterYawMotor.setTargetPosition(targetPosition);
        shooterYawMotor.setPower(YAW_POWER);
    }

    public void operate() {
        position = shooterYawMotor.getCurrentPosition();

        if (mode == SubsystemMode.MANUAL && map != null && map.shooterYawAutoLockToggle) {
            setAutoLock(!autoLockEnabled);
        }

        readYawFromLL();

        if (control == ControlMode.AUTO_LOCK && AUTO_LOCK_ENABLED) {
            operateAutoLock();
        } else {
            operateManual();
        }

        addTelemetry();
    }

    private void operateManual() {
        if (map != null) {
            if ((map.shooterYaw < 0 && targetPosition > MIN_POSITION) ||
                    (map.shooterYaw > 0 && targetPosition < MAX_POSITION)) {
                targetPosition += (int) (map.shooterYaw * CONTROL_LEVEL);
            }
        }
        if (map != null && map.resetShooterYaw) {
            resetShooterYaw();
            return;
        }

        if (targetPosition < MIN_POSITION) targetPosition = MIN_POSITION;
        if (targetPosition > MAX_POSITION) targetPosition = MAX_POSITION;

        if (shooterYawMotor.getMode() != DcMotor.RunMode.RUN_TO_POSITION) {
            shooterYawMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        shooterYawMotor.setTargetPosition(targetPosition);
        shooterYawMotor.setPower(YAW_POWER);
    }

    private void operateAutoLock() {
        if (!tagFresh || Double.isNaN(yawAvgDeg)) {
            holdHere();
            yawPid.reset();
            stableFrames = 0;
            wasLocked = false;
            return;
        }

        double errorDeg = -yawAvgDeg;

        if (Math.abs(errorDeg) < AUTO_LOCK_DEADBAND_DEG) {
            stableFrames++;
            if (!wasLocked) {
                wasLocked = true;
                lockStartMs = System.currentTimeMillis();
            }
            yawPid.reset();
            shooterYawMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            shooterYawMotor.setPower(0.0);
            return;
        } else {
            wasLocked = false;
            stableFrames = 0;
        }

        yawPid.updateError(errorDeg);
        double power = clamp(yawPid.run(), -AUTO_LOCK_MAX_POWER, AUTO_LOCK_MAX_POWER);

        if ((position >= MAX_POSITION - SOFT_LIMIT_MARGIN && power > 0) ||
                (position <= MIN_POSITION + SOFT_LIMIT_MARGIN && power < 0)) {
            shooterYawMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            shooterYawMotor.setPower(0.0);
            return;
        }

        shooterYawMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterYawMotor.setPower(power);
    }

    private void holdHere() {
        if (shooterYawMotor.getMode() != DcMotor.RunMode.RUN_TO_POSITION) {
            shooterYawMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        targetPosition = position;
        shooterYawMotor.setTargetPosition(targetPosition);
        shooterYawMotor.setPower(YAW_POWER);
    }

    private void resetState() {
        yawPid.reset();
        stableFrames = 0;
        wasLocked = false;
        yawRawDeg = Double.NaN;
        yawAvgDeg = Double.NaN;
        tagFresh = false;
    }

    private void readYawFromLL() {
        if (ll == null) {
            tagFresh = false;
            yawRawDeg = yawAvgDeg = Double.NaN;
            return;
        }
        int tagId = allianceRed ? APRIL_TAG_RED : APRIL_TAG_BLUE;
        LLAprilTag.YawInfo info = ll.getYawInfoForTag(tagId);
        yawRawDeg = info.rawDeg;
        yawAvgDeg = info.avgDeg;
        tagFresh = info.fresh;
    }

    private int clampPosition(int pos) {
        return (int) clamp(pos, MIN_POSITION, MAX_POSITION);
    }

    public void resetShooterYaw() {
        shooterYawMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        targetPosition = 0;
        position = 0;
        resetState();
        shooterYawMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        shooterYawMotor.setTargetPosition(targetPosition);
        shooterYawMotor.setPower(YAW_POWER);
    }

    private void addTelemetry() {
        int targetTag = allianceRed ? APRIL_TAG_RED : APRIL_TAG_BLUE;
        tele.addLine("=== SHOOTER YAW ===")
                .addData("Mode", mode::name)
                .addData("Control", control::name)
                .addData("AutoLock", "%b", autoLockEnabled)
                .addData("Locked", "%b", isLockedOnTarget())
                .addData("StableFrames", "%d", stableFrames)
                .addData("LockTimeMs", "%d", wasLocked ? (int) (System.currentTimeMillis() - lockStartMs) : 0)
                .addData("CurPos", "%d", position)
                .addData("TgtPos", "%d", targetPosition)
                .addData("YawRaw", "%.1f", Double.isNaN(yawRawDeg) ? 0.0 : yawRawDeg)
                .addData("YawAvg", "%.1f", Double.isNaN(yawAvgDeg) ? 0.0 : yawAvgDeg)
                .addData("TagFresh", "%b", tagFresh)
                .addData("TagId", "%d", targetTag);
    }
}
