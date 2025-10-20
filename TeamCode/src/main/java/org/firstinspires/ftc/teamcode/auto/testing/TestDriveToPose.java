package org.firstinspires.ftc.teamcode.auto.testing;

import static org.firstinspires.ftc.teamcode.config.AutoMotionConfig.DEFAULT_TIMEOUT_S;
import static org.firstinspires.ftc.teamcode.config.AutoMotionConfig.DRIVE_STOP_DIST_IN;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.auto.motion.FixedFieldHeading;
import org.firstinspires.ftc.teamcode.auto.motion.HeadingController;
import org.firstinspires.ftc.teamcode.auto.motion.MotionController;
import org.firstinspires.ftc.teamcode.subsystems.Mecanum;
import org.firstinspires.ftc.teamcode.util.TelemetryHelper;

@Autonomous(name = "Test drive to pose", group = "Testing")
public class TestDriveToPose extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Mecanum drive = new Mecanum(this, null);
        drive.init();
        drive.setStartingPose(new Pose(72, 72, Math.toRadians(90)));
        drive.startAuto();

        HeadingController hc = new HeadingController();
        MotionController mc = new MotionController(drive, hc, new TelemetryHelper(this, true));

        Pose target = new Pose(90, 90, Math.toRadians(90));

        if (isStopRequested()) return;
        waitForStart();

        long t0 = System.currentTimeMillis();
        while (opModeIsActive()) {
            double elapsed = (System.currentTimeMillis() - t0) / 1000.0;

            boolean done = mc.driveToPose(target,
                    DRIVE_STOP_DIST_IN,
                    DEFAULT_TIMEOUT_S - elapsed,
                    new FixedFieldHeading(90));

            drive.operate();
            TelemetryHelper.update();
            if (done) break;
            sleep(20);
        }
    }
}