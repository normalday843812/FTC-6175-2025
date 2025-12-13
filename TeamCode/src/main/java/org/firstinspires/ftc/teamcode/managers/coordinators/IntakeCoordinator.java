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
    private boolean ballLatched = false;

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
        if (!running) {
            confirmationCount = 0;
            ballLatched = false;
            return false;
        }
        return checkForBall();
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
        // Manual mode: "hold" controls (GamepadMap provides these only for TeleopManager OFF).
        // - intakeReverseToggle: run intake in REVERSE (normal intake direction)
        // - intakeToggle: run intake in FORWARD (block/eject)
        boolean reverseHeld = map.intakeReverseToggle;
        boolean forwardHeld = map.intakeToggle;

        if (reverseHeld) {
            running = true;
            reversing = true;
        } else if (forwardHeld) {
            running = true;
            reversing = false;
        } else {
            running = false;
            reversing = false;
        }
        confirmationCount = 0;
        ballLatched = false;
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
        if (sensors == null) return false;

        // Teleop uses a dedicated intake sensor (index 2) for "a new ball is entering / stuck at intake".
        // Slot-0/front sensors are used separately to confirm the ball is actually seated before counting it.
        boolean detected = sensors.hasIntakeBall();
        if (!detected) {
            confirmationCount = 0;
            ballLatched = false;
            return false;
        }

        if (ballLatched) {
            return false;
        }

        confirmationCount++;
        if (confirmationCount >= Math.max(1, TELEOP_INTAKE_CONFIRM_CYCLES)) {
            ballLatched = true;
            confirmationCount = 0;
            return true;
        }
        return false;
    }
}
