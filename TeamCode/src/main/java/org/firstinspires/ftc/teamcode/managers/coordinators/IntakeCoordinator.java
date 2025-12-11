package org.firstinspires.ftc.teamcode.managers.coordinators;

import static org.firstinspires.ftc.teamcode.config.ColorCal.COLOR_MARGIN;
import static org.firstinspires.ftc.teamcode.config.ColorCal.MIN_BALL_CONF;

import org.firstinspires.ftc.teamcode.GamepadMap;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.SlotColorSensors;

public class IntakeCoordinator {

    private static final int CONFIRMATION_THRESHOLD = 3;

    private final Intake intake;
    private final SlotColorSensors sensors;

    private boolean running = false;
    private boolean reversing = false;
    private int confirmationCount = 0;

    public IntakeCoordinator(Intake intake, SlotColorSensors sensors) {
        this.intake = intake;
        this.sensors = sensors;
    }

    public SlotColorSensors.BallColor update(GamepadMap map) {
        handleInputs(map);
        applyState();
        return running ? checkForBall() : null;
    }

    public void forceStart() {
        running = true;
        reversing = false;
    }

    public void forceStop() {
        running = false;
        reversing = false;
        confirmationCount = 0;
    }

    public boolean isRunning() {
        return running;
    }

    private void handleInputs(GamepadMap map) {
        if (map.intakeToggle) {
            running = !running;
            reversing = false;
            confirmationCount = 0;
        }
        if (map.intakeReverseToggle) {
            reversing = !reversing;
        }
        if (map.intakeClearJam) {
            intake.clearJam();
        }
    }

    private void applyState() {
        if (!running) {
            intake.setAutoMode(Intake.AutoMode.OFF);
        } else if (reversing) {
            intake.setAutoMode(Intake.AutoMode.REVERSE);
        } else {
            intake.setAutoMode(Intake.AutoMode.FORWARD);
        }
    }

    private SlotColorSensors.BallColor checkForBall() {
        SlotColorSensors.Observation obs = sensors.getObservation(0);

        boolean detected = obs.valid
                && obs.ballConfidence >= MIN_BALL_CONF
                && Math.abs(obs.purpleConfidence - obs.greenConfidence) >= COLOR_MARGIN;

        if (detected) {
            confirmationCount++;
            if (confirmationCount >= CONFIRMATION_THRESHOLD) {
                confirmationCount = 0;
                return obs.color;
            }
        } else {
            confirmationCount = 0;
        }

        return null;
    }
}