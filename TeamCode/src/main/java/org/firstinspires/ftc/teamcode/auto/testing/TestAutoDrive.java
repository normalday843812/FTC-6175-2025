package org.firstinspires.ftc.teamcode.auto.testing;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.subsystems.Mecanum;
import org.firstinspires.ftc.teamcode.util.TelemetryHelper;

@Autonomous(name = "Auto Drive test", group = "Testing")
public class TestAutoDrive extends LinearOpMode {
    @Override
    public void runOpMode() {
        Mecanum drive = new Mecanum(this, null);
        drive.init();
        drive.startAuto();
        drive.setStartingPose(new Pose(72, 72, Math.toRadians(90)));

        if (isStopRequested()) return;
        waitForStart();

        long t0 = System.currentTimeMillis();
        while (opModeIsActive()) {
            double t = (System.currentTimeMillis() - t0) / 1000.0;
            if (t < 3.0) drive.setAutoDrive(0.4, 0, 0, true, 0);
            else drive.setAutoDrive(0.0, 0, 0, true, 0);

            drive.operate();
            TelemetryHelper.update();
            sleep(20);
        }
    }
}