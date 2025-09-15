package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.GamepadMap;

@Configurable
public class Intake {
    OpMode opmode;

    // Telemetry
    public static boolean TELEMETRY_ENABLED = true;

    GamepadMap map;

    // Hardware
    private final DcMotor intakeMotor;

    // Constants
    public static double INTAKE_MOTOR_SPEED = 1.0; // TODO

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
        intakeMotor.setPower(runIntake ? INTAKE_MOTOR_SPEED : 0.0);
        if (TELEMETRY_ENABLED) { addTelemetry(); }
    }

    private void handleToggles() {
        boolean intakeToggle = map.intakeToggle;
        if (intakeToggle && !prevIntakeToggle) {
            runIntake = !runIntake;
        }
        prevIntakeToggle = intakeToggle;
    }

    private void addTelemetry() {
        opmode.telemetry.addLine("--- INTAKE ---");
        opmode.telemetry.addData("Run Intake?:", runIntake);
        opmode.telemetry.addData("Intake Motor Power:", intakeMotor.getPower());
    }
}
