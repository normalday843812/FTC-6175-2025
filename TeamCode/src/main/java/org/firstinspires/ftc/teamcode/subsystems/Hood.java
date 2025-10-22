package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.config.GlobalConfig.FALLBACK_MODE;
import static org.firstinspires.ftc.teamcode.config.HoodServoConfig.MANUAL_DEADBAND;
import static org.firstinspires.ftc.teamcode.config.HoodServoConfig.MANUAL_HOLD_SEC;
import static org.firstinspires.ftc.teamcode.config.HoodServoConfig.MANUAL_RATE_PER_SEC;
import static org.firstinspires.ftc.teamcode.config.HoodServoConfig.MAX_POS;
import static org.firstinspires.ftc.teamcode.config.HoodServoConfig.MIN_POS;
import static org.firstinspires.ftc.teamcode.config.HoodServoConfig.START_POS;
import static org.firstinspires.ftc.teamcode.config.HoodServoConfig.TELEMETRY_ENABLED;
import static org.firstinspires.ftc.teamcode.util.MathUtil.deadband;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.GamepadMap;
import org.firstinspires.ftc.teamcode.util.SubsystemMode;
import org.firstinspires.ftc.teamcode.util.TelemetryHelper;

public class Hood {
    private final OpMode opmode;
    private final Servo hoodServo;
    private final GamepadMap map;
    private final TelemetryHelper tele;
    private double autoTarget = START_POS;
    private double commanded = START_POS;
    private boolean manualActive = false;
    private double lastManualTimeSec = -1.0;
    private double lastUpdateSec = -1.0;
    private SubsystemMode mode = SubsystemMode.MANUAL;

    public Hood(Servo hoodServo, GamepadMap map, OpMode opmode) {
        this.opmode = opmode;
        this.hoodServo = hoodServo;
        this.map = map;
        this.tele = new TelemetryHelper(opmode, TELEMETRY_ENABLED);
        hoodServo.setPosition(commanded);
    }

    public void startTeleop() { mode = SubsystemMode.MANUAL; }
    public void startAuto()   { mode = SubsystemMode.AUTO; manualActive = false; }

    public void setAutoTarget(double pos) {
        autoTarget = clamp(pos);
        if (mode == SubsystemMode.AUTO && !manualActive) commanded = autoTarget;
    }

    public void operate() {
        if (FALLBACK_MODE) { operateFallback(); return; }

        double now = opmode.getRuntime();
        double dt = lastUpdateSec < 0 ? 0 : Math.max(0, now - lastUpdateSec);
        lastUpdateSec = now;

        if (mode == SubsystemMode.MANUAL) {
            double axis = deadband(map != null ? map.hoodAxis : 0.0, MANUAL_DEADBAND);
            if (Math.abs(axis) > 0) {
                manualActive = true;
                lastManualTimeSec = now;
                commanded = clamp(commanded + axis * MANUAL_RATE_PER_SEC * dt);
            } else {
                //noinspection StatementWithEmptyBody
                if (manualActive && (now - lastManualTimeSec) < MANUAL_HOLD_SEC) {
                    // Hold commanded
                } else {
                    manualActive = false;
                    commanded = autoTarget;
                }
            }
        } else {
            manualActive = false;
            commanded = autoTarget;
        }

        hoodServo.setPosition(commanded);
        addTelemetry();
    }

    private void operateFallback() {
        double axis = map != null ? map.hoodAxis : 0.0;
        manualActive = true;
        commanded = clamp(commanded + axis * MANUAL_RATE_PER_SEC);
        hoodServo.setPosition(commanded);
    }

    public double getCommanded() {
        return commanded;
    }

    private static double clamp(double v) {
        return Math.max(MIN_POS, Math.min(MAX_POS, v));
    }

    private void addTelemetry() {
        tele.addLine("--- HOOD ---")
                .addData("Mode", mode::name)
                .addData("Cmd", "%.3f", commanded)
                .addData("AutoTgt", "%.3f", autoTarget)
                .addData("Manual", "%b", manualActive);
    }
}
