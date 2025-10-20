package org.firstinspires.ftc.teamcode.auto.testing;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.auto.motion.FixedFieldHeading;
import org.firstinspires.ftc.teamcode.auto.motion.HeadingController;
import org.firstinspires.ftc.teamcode.auto.motion.MotionController;
import org.firstinspires.ftc.teamcode.subsystems.Mecanum;
import org.firstinspires.ftc.teamcode.util.TelemetryHelper;

@Autonomous(name = "Heading lock test", group = "Testing")
public class TestHeadingLock extends LinearOpMode {
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

            if (t < 5.0) {
                mc.holdHeading(new FixedFieldHeading(90, "Face North"));
            } else if (t < 10.0) {
                mc.holdHeading(new FixedFieldHeading(0, "Face East"));
            } else if (t < 15.0) {
                mc.holdHeading(new FixedFieldHeading(180, "Face West"));
            } else {
                mc.holdHeading(new FixedFieldHeading(270, "Face South"));
            }

            drive.operate();
            TelemetryHelper.update();
            sleep(20);
        }
    }
}