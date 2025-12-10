package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.config.SpindexerConfig.BIAS;
import static org.firstinspires.ftc.teamcode.config.SpindexerConfig.SLOTS;
import static org.firstinspires.ftc.teamcode.config.SpindexerConfig.STEP;
import static org.firstinspires.ftc.teamcode.config.SpindexerConfig.TELEMETRY_ENABLED;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.GamepadMap;
import org.firstinspires.ftc.teamcode.util.SubsystemMode;
import org.firstinspires.ftc.teamcode.util.TelemetryHelper;
import org.firstinspires.ftc.teamcode.util.Timer;

public class Spindexer {
    private static final long SETTLE_TIME_MS = 300;

    private final Servo spindexerServo;
    private final GamepadMap map;
    private final TelemetryHelper tele;
    private SlotColorSensors slots;

    private SubsystemMode mode = SubsystemMode.MANUAL;
    private int commandedSlot = 0;
    private long lastCommandMs = 0;

    // Jiggle state
    private boolean jiggleActive = false;
    private double jUp, jDown, jDwellS;
    private int jPhase = 0;
    private final Timer jTimer = new Timer();
    private double lastJiggleBase = 0.0;

    public Spindexer(Servo spindexerServo, GamepadMap map, OpMode opmode) {
        this.spindexerServo = spindexerServo;
        this.map = map;
        this.tele = new TelemetryHelper(opmode, TELEMETRY_ENABLED);
        spindexerServo.setPosition(BIAS);
    }

    public void startTeleop() {
        mode = SubsystemMode.MANUAL;
        spindexerServo.setPosition(BIAS);
        commandedSlot = 0;
    }

    public void startAuto() {
        mode = SubsystemMode.AUTO;
        spindexerServo.setPosition(BIAS);
        commandedSlot = 0;
    }

    public void setColorSlots(SlotColorSensors slots) {
        this.slots = slots;
    }

    public void operate() {
        if (mode == SubsystemMode.MANUAL && map != null) {
            if (map.spindexerForward) stepForward();
            if (map.spindexerBackward) stepBackward();

            if (map.findGreenBall && slots != null) {
                int slot = slots.findBallSlot(SlotColorSensors.BallColor.GREEN);
                if (slot >= 0) {
                    setSlot(slot);
                }
            }
            if (map.findPurpleBall && slots != null) {
                int slot = slots.findBallSlot(SlotColorSensors.BallColor.PURPLE);
                if (slot >= 0) {
                    setSlot(slot);
                }
            }
        }

        if (jiggleActive) {
            switch (jPhase) {
                case 0:
                    spindexerServo.setPosition(jUp);
                    if (jTimer.getElapsedTimeSeconds() >= jDwellS) {
                        jPhase = 1;
                        jTimer.resetTimer();
                    }
                    break;
                case 1:
                    spindexerServo.setPosition(jDown);
                    if (jTimer.getElapsedTimeSeconds() >= jDwellS) {
                        jPhase = 2;
                        jTimer.resetTimer();
                    }
                    break;
                case 2:
                    spindexerServo.setPosition(lastJiggleBase);
                    if (jTimer.getElapsedTimeSeconds() >= jDwellS) {
                        jiggleActive = false;
                    }
                    break;
            }
        }

        addTelemetry();
    }

    public int getSlots() {
        return SLOTS > 0 ? SLOTS : (int) Math.max(1, Math.round(1.0 / STEP));
    }

    public int getCurrentSlot() {
        double pos = getPosition() - BIAS;
        if (pos < 0) pos += 1.0;
        int slot = (int) Math.round(pos / STEP);
        return ((slot % getSlots()) + getSlots()) % getSlots();
    }

    public int getCommandedSlot() {
        return commandedSlot;
    }

    public boolean isMoving() {
        return jiggleActive || (System.currentTimeMillis() - lastCommandMs < SETTLE_TIME_MS);
    }

    public boolean isSettled() {
        return !isMoving();
    }

    public void setSlot(int idx) {
        int slots = getSlots();
        int k = ((idx % slots) + slots) % slots;

        commandedSlot = k;
        lastCommandMs = System.currentTimeMillis();

        double target = BIAS + k * STEP;
        while (target > 1.0) target -= 1.0;
        while (target < 0.0) target += 1.0;
        setAbsolute(target);
    }

    public void stepSlots(int delta) {
        setSlot(getCurrentSlot() + delta);
    }

    public void stepForward() {
        stepSlots(1);
    }

    public void stepBackward() {
        stepSlots(-1);
    }

    public void setAbsolute(double pos) {
        spindexerServo.setPosition(Math.max(0.0, Math.min(1.0, pos)));
    }

    public double getPosition() {
        return spindexerServo.getPosition();
    }

    public void startJiggle(double deltaUp, double deltaDown, double dwellS) {
        double base = getPosition();
        double up = Math.min(1.0, base + deltaUp);
        double down = Math.max(0.0, base - deltaDown);
        lastJiggleBase = base;
        jUp = up;
        jDown = down;
        jDwellS = dwellS;
        jPhase = 0;
        jiggleActive = true;
        jTimer.resetTimer();
    }

    public boolean updateJiggle() {
        return !jiggleActive;
    }

    public void stopJiggle() {
        jiggleActive = false;
    }

    private void addTelemetry() {
        tele.addLine("=== SPINDEXER ===")
                .addData("Mode", mode::name)
                .addData("Pos", "%.2f", spindexerServo.getPosition())
                .addData("Slot", "%d", getCurrentSlot())
                .addData("CmdSlot", "%d", commandedSlot)
                .addData("Moving", "%b", isMoving())
                .addData("JiggleActive", "%b", jiggleActive)
                .addData("JigglePhase", "%d", jPhase)
                .addData("LastJiggleBase", "%.2f", lastJiggleBase);
    }
}