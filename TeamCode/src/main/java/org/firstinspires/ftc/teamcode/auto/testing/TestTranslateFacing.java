package org.firstinspires.ftc.teamcode.auto.testing;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.auto.motion.FixedFieldHeading;
import org.firstinspires.ftc.teamcode.auto.motion.HeadingController;
import org.firstinspires.ftc.teamcode.auto.motion.MotionController;
import org.firstinspires.ftc.teamcode.subsystems.Mecanum;
import org.firstinspires.ftc.teamcode.util.TelemetryHelper;

@Autonomous(name = "Test translate facing", group = "Testing")
public class TestTranslateFacing extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Mecanum drive = new Mecanum(this, null);
        drive.init();
        drive.setStartingPose(new Pose(72, 72, Math.toRadians(90)));
        drive.startAuto();

        HeadingController hc = new HeadingController();
        MotionController mc = new MotionController(drive, hc, new TelemetryHelper(this, true));

        if (isStopRequested()) return;
        waitForStart();

        long t0 = System.currentTimeMillis();
        while (opModeIsActive()) {
            double t = (System.currentTimeMillis() - t0) / 1000.0;
            // Drives a square
            double vx, vy;

            if (t < 3) {
                vx = 0.4;
                vy = 0.0;
            } else if (t < 6) {
                vx = 0.0;
                vy = 0.4;
            } else if (t < 9) {
                vx = -0.4;
                vy = 0.0;
            } else if (t < 12) {
                vx = 0.0;
                vy = -0.4;
            } else {
                vx = 0.0;
                vy = 0.0;
            }

            mc.translateFacing(vx, vy, new FixedFieldHeading(90, "Face North"));
            drive.operate();
            TelemetryHelper.update();
            sleep(20);
        }
    }
}