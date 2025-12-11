package org.firstinspires.ftc.teamcode.managers.coordinators;

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

    public boolean update(GamepadMap map) {
        handleInputs(map);
        applyState();
        return running && checkForBall();
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

    private boolean checkForBall() {
        boolean detected = sensors.hasBall();

        if (detected) {
            confirmationCount++;
            if (confirmationCount >= CONFIRMATION_THRESHOLD) {
                confirmationCount = 0;
                return true;
            }
        } else {
            confirmationCount = 0;
        }

        return false;
    }
}