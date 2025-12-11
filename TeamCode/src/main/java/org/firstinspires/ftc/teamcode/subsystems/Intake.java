package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.config.IntakeConfig.FORWARD_PWR;
import static org.firstinspires.ftc.teamcode.config.IntakeConfig.FWD_ARM_FACTOR;
import static org.firstinspires.ftc.teamcode.config.IntakeConfig.INTAKE_TPR;
import static org.firstinspires.ftc.teamcode.config.IntakeConfig.JAM_FWD_MIN_RPM;
import static org.firstinspires.ftc.teamcode.config.IntakeConfig.JAM_IN_TIME_S;
import static org.firstinspires.ftc.teamcode.config.IntakeConfig.JAM_OUT_TIME_S;
import static org.firstinspires.ftc.teamcode.config.IntakeConfig.JAM_REV_MIN_ABS_RPM;
import static org.firstinspires.ftc.teamcode.config.IntakeConfig.REVERSE_PWR;
import static org.firstinspires.ftc.teamcode.config.IntakeConfig.REV_ARM_FACTOR;
import static org.firstinspires.ftc.teamcode.config.IntakeConfig.TELEMETRY_ENABLED;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.GamepadMap;
import org.firstinspires.ftc.teamcode.util.SubsystemMode;
import org.firstinspires.ftc.teamcode.util.TelemetryHelper;
import org.firstinspires.ftc.teamcode.util.Timer;

public class Intake {
    public enum AutoMode {OFF, FORWARD, REVERSE}

    private enum JamState {IDLE, OUT, IN}

    private final DcMotorEx motor;
    private final TelemetryHelper tele;

    private double commandedPower = 0.0;
    private double motorBefore = 0.0;

    private JamState jamState = JamState.IDLE;
    private final Timer jamTimer = new Timer();

    private AutoMode autoMode = AutoMode.OFF;

    private boolean fwdArmed = false;
    private boolean revArmed = false;
    private int lastDir = 0;
    private long lastDirChangeMs = 0;

    public Intake(DcMotorEx motor, OpMode opmode) {
        this.motor = motor;
        this.tele = new TelemetryHelper(opmode, TELEMETRY_ENABLED);
    }

    public void setAutoMode(AutoMode m) {
        this.autoMode = m;
    }

    public void clearJam() {
        jamState = JamState.OUT;
        jamTimer.resetTimer();
    }

    public boolean isJammed() {
        return jamState != JamState.IDLE;
    }

    public boolean isRunning() {
        return this.autoMode != AutoMode.OFF;
    }

    public void operate() {
        if (handleJam()) {
            addTelemetry();
            return;
        }

        switch (autoMode) {
            case OFF:
                commandedPower = 0.0;
                motor.setPower(0.0);
                break;
            case FORWARD:
                commandedPower = FORWARD_PWR;
                motor.setPower(FORWARD_PWR);
                break;
            case REVERSE:
                commandedPower = REVERSE_PWR;
                motor.setPower(REVERSE_PWR);
                break;
        }
        detectAndClearJamIfNeeded();
        addTelemetry();
    }

    private boolean handleJam() {
        switch (jamState) {
            case IDLE:
                return false;

            case OUT:
                motorBefore = motor.getPower();
                motor.setPower(REVERSE_PWR);
                if (jamTimer.getElapsedTimeSeconds() >= JAM_OUT_TIME_S) {
                    jamState = JamState.IN;
                    jamTimer.resetTimer();
                }
                return true;

            case IN:
                motor.setPower(FORWARD_PWR);
                if (jamTimer.getElapsedTimeSeconds() >= JAM_IN_TIME_S) {
                    motor.setPower(motorBefore);
                    jamState = JamState.IDLE;
                }
                return true;
        }
        return false;
    }

    private int desiredDir() {
        if (commandedPower > 0.9) return +1;
        if (commandedPower < -0.9) return -1;
        return 0;
    }

    private void updateArming(double rpm) {
        int dir = desiredDir();

        if (dir != lastDir) {
            fwdArmed = false;
            revArmed = false;
            lastDir = dir;
            lastDirChangeMs = System.currentTimeMillis(); // debounce window
        }

        if (dir == 1) {
            if (!fwdArmed && rpm >= JAM_FWD_MIN_RPM * FWD_ARM_FACTOR) {
                fwdArmed = true;
            }
            revArmed = false;
        } else if (dir == -1) {
            if (!revArmed && Math.abs(rpm) >= JAM_REV_MIN_ABS_RPM * REV_ARM_FACTOR) {
                revArmed = true;
            }
            fwdArmed = false;
        } else {
            fwdArmed = false;
            revArmed = false;
        }
    }

    private void detectAndClearJamIfNeeded() {
        if (jamState != JamState.IDLE) return;

        double rpm = getMotorRPM();
        updateArming(rpm);

        if (System.currentTimeMillis() - lastDirChangeMs < 150) return;

        int dir = desiredDir();

        if (dir == 1 && fwdArmed && rpm < JAM_FWD_MIN_RPM) {
            clearJam();
        }
    }

    private double getMotorRPM() {
        double tps = motor.getVelocity();
        return tps * 60.0 / INTAKE_TPR;
    }

    private void addTelemetry() {
        tele.addLine("=== INTAKE ===")
                .addData("AutoMode", autoMode::name)
                .addData("CmdPower", "%.2f", commandedPower)
                .addData("RPM", "%.0f", getMotorRPM())
                .addData("JamState", jamState::name)
                .addData("FwdArmed", "%b", fwdArmed)
                .addData("RevArmed", "%b", revArmed);
    }
}