package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.GamepadMap;
import org.firstinspires.ftc.teamcode.config.IntakeConfig;
import org.firstinspires.ftc.teamcode.util.TelemetryHelper;

public class Intake {
    private final DcMotor motor;
    private final GamepadMap map;
    private boolean running = false;
    private boolean reverse = false;
    private final TelemetryHelper tele;

    public Intake(DcMotor motor, GamepadMap map, OpMode opmode) {
        this.motor = motor;
        this.map = map;
        this.tele = new TelemetryHelper(opmode, IntakeConfig.TELEMETRY_ENABLED);
    }

    public void operate() {
        if (map.intakeToggle) running = !running;
        if (map.intakeReverseToggle) reverse = !reverse;
        double power = running ? (reverse ? IntakeConfig.REVERSE_PWR : IntakeConfig.FORWARD_PWR) : 0.0;
        motor.setPower(power);

        tele.addLine("--- INTAKE ---")
                .addData("Running:", "%b", running)
                .addData("Power:", "%.2f", motor.getPower());
    }
}
