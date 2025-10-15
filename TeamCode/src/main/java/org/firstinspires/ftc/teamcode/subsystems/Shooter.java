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

    public Shooter(DcMotor shooterMotor, GamepadMap map, OpMode opmode) {
        this.shooterMotor = shooterMotor;
        this.map = map;
        this.opmode = opmode;
    }

    public void OperateShooter() {
        shooterMotor.setPower(map.shooterButton);
        if (TELEMETRY_ENABLED) { addTelemetry(); }
    }


    private void addTelemetry() {
        opmode.telemetry.addLine("--- SHOOTER ---");
        opmode.telemetry.addData("Shooter Motor Power:", shooterMotor.getPower());
    }
}
