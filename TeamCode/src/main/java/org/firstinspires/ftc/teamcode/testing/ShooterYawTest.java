package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.GamepadMap;
import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.subsystems.Mecanum;
import org.firstinspires.ftc.teamcode.subsystems.ShooterYaw;
import org.firstinspires.ftc.teamcode.util.TelemetryHelper;
import org.firstinspires.ftc.teamcode.vision.LLAprilTag;

@TeleOp(name = "Shooter Yaw Test", group = "Testing")
public class ShooterYawTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        RobotHardware hw = new RobotHardware(this);

        GamepadMap map = new GamepadMap(this);

        Mecanum drive = new Mecanum(this, map);
        drive.init();

        hw.initLimeLight(100);
        hw.setLimelightPipeline(0);
        LLAprilTag aprilTag = new LLAprilTag(hw.getLimelight(), this);

        hw.initShooterYaw();
        ShooterYaw shooterYaw = new ShooterYaw(hw.getShooterYawMotor(), aprilTag, map,
                drive.getFollower(), this);

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
