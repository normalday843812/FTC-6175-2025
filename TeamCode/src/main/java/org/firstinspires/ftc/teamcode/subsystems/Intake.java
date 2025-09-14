package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.GamepadMap;

@Configurable
public class Intake {
    OpMode opmode;

    GamepadMap map;

    // Hardware
    private final DcMotor intakeMotor;

    // Constants
    public static double INTAKE_MOTOR_SPEED = 1.0;

    // Toggles
    private boolean prevIntakeToggle = false;
    private boolean runIntake = false;

    public Intake(DcMotor intakeMotor, GamepadMap map, OpMode opmode) {
        this.intakeMotor = intakeMotor;
        this.map = map;
        this.opmode = opmode;
    }

    public void OperateIntake() {
        handleToggles();
        intakeMotor.setPower(INTAKE_MOTOR_SPEED);
        addTelemetry();
    }

    private void handleToggles() {
        boolean intakeToggle = map.intakeToggle;
        if (intakeToggle && !prevIntakeToggle) {
            runIntake = !runIntake;
        }
        prevIntakeToggle = intakeToggle;
    }

    private void addTelemetry() {
        if (runIntake) {
            opmode.telemetry.addData("Intake:", "Running");
        }
    }
}
