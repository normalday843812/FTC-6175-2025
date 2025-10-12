package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.config.HoodServoConfig.*;
import static org.firstinspires.ftc.teamcode.util.MathUtil.deadband;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.GamepadMap;
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

    public Hood(Servo hoodServo, GamepadMap map, OpMode opmode) {
        this.opmode = opmode;
        this.hoodServo = hoodServo;
        this.map = map;
        this.tele = new TelemetryHelper(opmode, TELEMETRY_ENABLED);
        hoodServo.setPosition(commanded);
    }

    public void operate() {
        double now = opmode.getRuntime();
        double dt = lastUpdateSec < 0 ? 0 : Math.max(0, now - lastUpdateSec);
        lastUpdateSec = now;

        // Read manual axis and apply deadband
        double axis = deadband(map.hoodAxis, MANUAL_DEADBAND);

        if (Math.abs(axis) > 0) {
            manualActive = true;
            lastManualTimeSec = now;

            commanded += axis * MANUAL_RATE_PER_SEC * dt;
            commanded = clamp(commanded, MIN_POS, MAX_POS);
        } else {
            //noinspection StatementWithEmptyBody
            if (manualActive && (now - lastManualTimeSec) < MANUAL_HOLD_SEC) {
                // Hold commanded
            } else {
                manualActive = false;
                commanded = autoTarget;
            }
        }

        hoodServo.setPosition(commanded);

        tele.addData("HoodMode", "%s", manualActive ? "MANUAL" : "AUTO")
                .addData("Cmd", "%.3f", commanded)
                .addData("AutoTgt", "%.3f", autoTarget)
                .addData("Axis", "%.3f", axis);
    }

    public void setHoodServoPos(double pos) {
        autoTarget = clamp(pos, MIN_POS, MAX_POS);
        if (!manualActive) commanded = autoTarget;
    }

//    public double getCommanded() { return commanded; }

    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }
}
