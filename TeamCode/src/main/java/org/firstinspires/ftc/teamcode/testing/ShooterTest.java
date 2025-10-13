package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.GamepadMap;
import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.util.TelemetryHelper;

@TeleOp(name = "Shooter Test", group = "Testing")
public class ShooterTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        RobotHardware hw = new RobotHardware(this);

        GamepadMap map = new GamepadMap(this);

        hw.initShooter();
        Shooter shooter = new Shooter(
                hw.getShooterMotor(),
                map,
                this
        );

        if (isStopRequested()) return;

        waitForStart();

        while (opModeIsActive()) {
            map.update();
            shooter.operate();
            TelemetryHelper.update();
        }
    }
}
