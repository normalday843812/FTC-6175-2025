package org.firstinspires.ftc.teamcode.auto.testing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.util.TelemetryHelper;

@Autonomous(name = "Auto Shooter Test", group = "Testing")
public class TestAutoShooter extends LinearOpMode {
    @Override
    public void runOpMode() {
        RobotHardware hw = new RobotHardware(this);
        hw.initShooter();
        Shooter shooter = new Shooter(hw.getShooterMotor(), null, this);
        shooter.startAuto();
        shooter.setAutoRpm(3800);

        if (isStopRequested()) return;
        waitForStart();

        while (opModeIsActive()) {
            shooter.operate();
            TelemetryHelper.update();
            sleep(50);
        }
    }
}
