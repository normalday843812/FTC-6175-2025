package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.GamepadMap;

@Configurable
public class Shooter {
    OpMode opmode;

    // Telemetry
    public static boolean TELEMETRY_ENABLED = true;

    GamepadMap map;

    // Hardware
    private final DcMotor shooterMotor;

    // Constants
    public static double SHOOTER_MOTOR_SPEED = 1.5; // tune this

    // Toggles
    private boolean prevShooterToggle = false;
    private boolean runShooter = false;

    public Shooter(DcMotor shooterMotor, GamepadMap map, OpMode opmode) {
        this.shooterMotor = shooterMotor;
        this.map = map;
        this.opmode = opmode;
    }

    public void OperateShooter() {
        handleToggles();
        shooterMotor.setPower(runShooter ? SHOOTER_MOTOR_SPEED : 0.0);
        if (TELEMETRY_ENABLED) { addTelemetry(); }
    }

    private void handleToggles() {
        boolean shooterToggle = map.shooterToggle; // define later
        if (shooterToggle && !prevShooterToggle) {
            runShooter = !runShooter;
        }
        prevShooterToggle = shooterToggle;
    }

    private void addTelemetry() {
        opmode.telemetry.addLine("--- SHOOTER ---");
        opmode.telemetry.addData("Run Shooter?:", runShooter);
        opmode.telemetry.addData("Shooter Motor Power:", shooterMotor.getPower());
    }
}
