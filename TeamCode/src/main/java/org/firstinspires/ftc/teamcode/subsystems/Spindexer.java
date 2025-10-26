package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.config.SpindexerConfig.BIAS;
import static org.firstinspires.ftc.teamcode.config.SpindexerConfig.STEP;
import static org.firstinspires.ftc.teamcode.config.SpindexerConfig.TELEMETRY_ENABLED;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.GamepadMap;
import org.firstinspires.ftc.teamcode.util.SubsystemMode;
import org.firstinspires.ftc.teamcode.util.TelemetryHelper;

public class Spindexer {
    private final Servo spindexerServo;
    private final GamepadMap map;
    private final TelemetryHelper tele;

    private SubsystemMode mode = SubsystemMode.MANUAL;

    public Spindexer(Servo spindexerServo, GamepadMap map, OpMode opmode) {
        this.spindexerServo = spindexerServo;
        this.map = map;
        this.tele = new TelemetryHelper(opmode, TELEMETRY_ENABLED);
    }

    public void startTeleop() {
        mode = SubsystemMode.MANUAL;
        spindexerServo.setPosition(BIAS);
    }

    public void startAuto() {
        mode = SubsystemMode.AUTO;
        spindexerServo.setPosition(BIAS);
    }

    public void operate() {
        if (mode == SubsystemMode.MANUAL) {
            if (map != null) {
                if (map.spindexerForward) {
                    if (spindexerServo.getPosition() < ((int) (1 / STEP)) * STEP) {
                        spindexerServo.setPosition(spindexerServo.getPosition() + STEP);
                    } else {

                    }
                }
                if (map.spindexerBackward) {
                    if (spindexerServo.getPosition() > 0) {
                        spindexerServo.setPosition(spindexerServo.getPosition() - STEP);
                    } else {

                    }
                }
            }
        }
        addTelemetry();
    }

    public void stepForward() {
        double top = ((int) (1 / STEP)) * STEP;
        double next = spindexerServo.getPosition() + STEP;
        if (next > top + 1e-6) next = BIAS;
        spindexerServo.setPosition(next);
    }

    public void stepBackward() {
        double top = ((int) (1 / STEP)) * STEP;
        double prev = spindexerServo.getPosition() - STEP;
        if (prev < BIAS - 1e-6) prev = top;
        spindexerServo.setPosition(prev);
    }

    public void setAbsolute(double pos) {
        spindexerServo.setPosition(Math.max(0.0, Math.min(1.0, pos)));
    }

    public double getPosition() {
        return spindexerServo.getPosition();
    }

    public static class JigglePlan {
        public final double base;
        public final double up;
        public final double down;

        public JigglePlan(double base, double up, double down) {
            this.base = base;
            this.up = up;
            this.down = down;
        }
    }

    public JigglePlan makeJigglePlan(double deltaUp, double deltaDown) {
        double base = getPosition();
        double up = Math.min(1.0, base + deltaUp);
        double down = Math.max(0.0, base - deltaDown);
        return new JigglePlan(base, up, down);
    }

    private void addTelemetry() {
        tele.addLine("=== SPINDEXER ===")
                .addData("Mode", mode::name)
                .addData("Position", "%.2f", spindexerServo.getPosition());
    }
}
