package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.config.DriveConfig.TELEMETRY_ENABLED;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.GamepadMap;
import org.firstinspires.ftc.teamcode.config.IntakeConfig;
import org.firstinspires.ftc.teamcode.util.TelemetryHelper;

public class Intake {
    private final OpMode opmode;
    private final DcMotor motor;
    private final GamepadMap map;
    private boolean running=false;

    public Intake(DcMotor motor, GamepadMap map, OpMode opmode){
        this.motor=motor;
        this.map=map;
        this.opmode=opmode;
    }

    public void operate() {
        if (map.intakeToggle) running = !running;
        double power = 0.0;
        if (running) power = map.intakeReverseToggle ? IntakeConfig.REVERSE_PWR : IntakeConfig.FORWARD_PWR;
        motor.setPower(power);

        new TelemetryHelper(opmode, TELEMETRY_ENABLED)
                .addLine("--- INTAKE ---")
                .addData("Running:", "%b", running)
                .addData("Power:", "%.2f", motor.getPower());
    }
}
