package org.firstinspires.ftc.teamcode.managers.coordinators;

import static org.firstinspires.ftc.teamcode.config.AutoUnifiedConfig.TELEOP_INTAKE_CONFIRM_CYCLES;

import org.firstinspires.ftc.teamcode.GamepadMap;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.SlotColorSensors;

public class IntakeCoordinator {

    private final Intake intake;
    private final SlotColorSensors sensors;

    private boolean running = false;
    private boolean reversing = false;
    private int confirmationCount = 0;

    // Desired state set by manager
    private boolean desiredRunning = false;
    private boolean desiredReversing = false;

    public IntakeCoordinator(Intake intake, SlotColorSensors sensors) {
        this.intake = intake;
        this.sensors = sensors;
    }

    /**
     * Set the desired intake state (used by manager when it has control).
     */
    public void setDesiredState(boolean running, boolean reversing) {
        this.desiredRunning = running;
        this.desiredReversing = reversing;
    }

    /**
     * Update coordinator state.
     * @param map gamepad inputs
     * @param managerControlsIntake if true, use manager's desired state instead of gamepad
     * @return true if ball was detected while running
     */
    public boolean update(GamepadMap map, boolean managerControlsIntake) {
        if (managerControlsIntake) {
            // Use manager's desired state
            this.running = desiredRunning;
            this.reversing = desiredReversing;
        } else {
            handleInputs(map);
        }
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
        // Jam clearing is automatic via Intake.detectAndClearJamIfNeeded()
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
            if (confirmationCount >= Math.max(1, TELEOP_INTAKE_CONFIRM_CYCLES)) {
                confirmationCount = 0;
                return true;
            }
        } else {
            confirmationCount = 0;
        }

        return false;
    }
}
