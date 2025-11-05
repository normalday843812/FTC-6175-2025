package org.firstinspires.ftc.teamcode.auto.testing;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.GamepadMap;
import org.firstinspires.ftc.teamcode.auto.motion.FixedFieldHeading;
import org.firstinspires.ftc.teamcode.auto.motion.MotionController;
import org.firstinspires.ftc.teamcode.subsystems.Mecanum;
import org.firstinspires.ftc.teamcode.util.TelemetryHelper;
import org.firstinspires.ftc.teamcode.config.AutoUnifiedConfig;

@Autonomous(name = "Back/Forth Test", group = "Testing")
public class BackForthTest extends LinearOpMode {

    private enum State {
        TO_B,
        WAIT_B,
        TO_A,
        WAIT_A
    }

    @Override
    public void runOpMode() throws InterruptedException {
        // drive + telemetry like in your auto
        GamepadMap map = new GamepadMap(this);
        Mecanum drive = new Mecanum(this, map);
        drive.init();

        TelemetryHelper tele = new TelemetryHelper(this, AutoUnifiedConfig.TELEMETRY_ENABLED);
        MotionController motion = new MotionController(drive, tele);

        // A: start pose 72, 8, 90°
        Pose poseA = new Pose(72, 8, Math.toRadians(90));
        // B: target pose 72, 100, 180° (we will tell the path to end at 180)
        Pose poseB = new Pose(72, 100, Math.toRadians(180));

        // fixed headings for each leg
        FixedFieldHeading headA = new FixedFieldHeading(90, "HeadA");
        FixedFieldHeading headB = new FixedFieldHeading(180, "HeadB");

        drive.setStartingPose(poseA);
        drive.startAuto();

        State s = State.TO_B;
        long waitStart = 0;
        final long WAIT_MS = 300; // small settle time

        waitForStart();
        if (isStopRequested()) return;

        // kick off first move
        motion.followToPose(poseB, headB.getTargetHeadingDeg(null));

        while (opModeIsActive()) {
            drive.operate();

            switch (s) {
                case TO_B:
                    if (!motion.isBusy()) {
                        s = State.WAIT_B;
                        waitStart = System.currentTimeMillis();
                    }
                    break;

                case WAIT_B:
                    if (System.currentTimeMillis() - waitStart >= WAIT_MS) {
                        motion.followToPose(poseA, headA.getTargetHeadingDeg(null));
                        s = State.TO_A;
                    }
                    break;

                case TO_A:
                    if (!motion.isBusy()) {
                        s = State.WAIT_A;
                        waitStart = System.currentTimeMillis();
                    }
                    break;

                case WAIT_A:
                    if (System.currentTimeMillis() - waitStart >= WAIT_MS) {
                        motion.followToPose(poseB, headB.getTargetHeadingDeg(null));
                        s = State.TO_B;
                    }
                    break;
            }

            TelemetryHelper.update();
            sleep(20);
        }
    }
}
