package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.GamepadMap;
import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.subsystems.ShooterYaw;
import org.firstinspires.ftc.teamcode.util.TelemetryHelper;

@TeleOp(name = "Shooter Yaw Test", group = "Testing")
public class ShooterYawTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        RobotHardware hw = new RobotHardware(this);

        GamepadMap map = new GamepadMap(this);

        hw.initShooterYaw();
        ShooterYaw shooterYaw = new ShooterYaw(hw.getShooterYawMotor(), map, this);

        if (isStopRequested()) return;

        waitForStart();

        shooterYaw.startTeleop();

        while (opModeIsActive()) {
            map.update();
            shooterYaw.operate();
            TelemetryHelper.update();
        }
    }
}
